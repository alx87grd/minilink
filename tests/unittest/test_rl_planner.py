"""The RL planner, its environment semantics, the neural policy block, and Monte Carlo evaluation."""

import warnings

import numpy as np
import pytest

from minilink import Pendulum
from minilink.core.costs import CostFunction
from minilink.dynamics.catalog.pendulum.pendulum import PendulumWithNoisePort
from minilink.planning.distributions import Gaussian, Uniform
from minilink.planning.problems import PlanningProblem, StochasticPlanningProblem
from minilink.planning.results import PolicyPlan, TrajectoryPlan

pytest.importorskip("jax")
import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.control.neural import NeuralPolicyController  # noqa: E402
from minilink.planning.evaluation import (  # noqa: E402
    MonteCarloEvaluator,
    score_trajectory,
)
from minilink.planning.reinforcement_learning import (  # noqa: E402
    PPO,
    ReinforcementLearningPlanner,
    RolloutEnvironment,
)

pytestmark = [pytest.mark.optional, pytest.mark.jax]


class HangCost(CostFunction):
    """Quadratic about the hanging equilibrium; a unit terminal cost."""

    def g(self, x, u, t=0.0, params=None):
        return x[0] ** 2 + 0.1 * x[1] ** 2 + 0.01 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 1.0


def bounded_pendulum(plant=None):
    plant = Pendulum() if plant is None else plant
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    plant.state.lower_bound = np.array([-np.pi, -8.0])
    plant.state.upper_bound = np.array([np.pi, 8.0])
    return plant


def problem(**kwargs):
    plant = bounded_pendulum()
    return StochasticPlanningProblem(
        plant,
        cost=HangCost(),
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        **kwargs,
    )


# --- environment semantics ---


def test_environment_infinite_horizon_truncates_and_bootstraps_by_default():
    env = RolloutEnvironment(problem(tf=np.inf), dt=0.1, episode_length=0.3)
    assert not env.finite_horizon and not env.charge_exit
    x, t = jnp.zeros(2), 0.0
    for _ in range(2):
        x, t, r, terminated, truncated = env.step(
            x, t, jnp.zeros(1), jax.random.PRNGKey(0)
        )
        assert not bool(terminated) and not bool(truncated)
    x, t, r, terminated, truncated = env.step(x, t, jnp.zeros(1), jax.random.PRNGKey(0))
    assert bool(truncated) and not bool(terminated)  # episode length reached
    # leaving the box: truncated (bootstrap), no charge
    _, _, r_exit, terminated, truncated = env.step(
        jnp.array([3.1, 7.9]), 0.0, jnp.array([4.0]), jax.random.PRNGKey(0)
    )
    assert bool(truncated) and not bool(terminated)


def test_environment_finite_horizon_charges_h_and_exit_cost():
    env = RolloutEnvironment(
        problem(tf=0.2, on_exit="terminate", exit_cost=50.0), dt=0.1
    )
    assert env.finite_horizon and env.charge_exit
    x, t, r1, terminated, _ = env.step(
        jnp.zeros(2), 0.0, jnp.zeros(1), jax.random.PRNGKey(0)
    )
    assert not bool(terminated)
    x, t, r2, terminated, truncated = env.step(
        x, t, jnp.zeros(1), jax.random.PRNGKey(0)
    )
    assert bool(terminated) and not bool(truncated)
    np.testing.assert_allclose(
        float(r2 - r1), -1.0, atol=1e-6
    )  # the terminal cost h = 1
    # a charged exit: terminated, reward carries the -50 penalty
    _, _, r_exit, terminated, truncated = env.step(
        jnp.array([3.1, 7.9]), 0.0, jnp.array([4.0]), jax.random.PRNGKey(0)
    )
    assert bool(terminated) and float(r_exit) < -49.0


# --- policy block ---


def test_neural_policy_controller_normalizes_and_composes():
    plant = bounded_pendulum()
    ctl = NeuralPolicyController(plant, hidden=(8,), seed=0)
    z = ctl.observe(np.array([np.pi, 8.0]))
    np.testing.assert_allclose(z, [1.0, 1.0])
    u = ctl.action(np.zeros(2))
    assert u.shape == (1,) and -4.0 <= u[0] <= 4.0
    # traces under JAX with an explicit params pytree
    u_jax = jax.jit(lambda p, x: ctl.action(x, p))(ctl.params, jnp.zeros(2))
    np.testing.assert_allclose(np.asarray(u_jax), u, atol=1e-6)
    cl_sys = ctl @ plant
    traj = cl_sys.compute_trajectory(tf=0.2, dt=0.1, verbose=False)
    assert np.all(np.isfinite(traj.x))


# --- planner ---


def test_planner_learns_returns_a_policy_plan_and_a_controller():
    prob = problem(tf=np.inf)
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=32, batch_size=32, verbose=0
    )
    assert isinstance(planner.algorithm, PPO)
    assert planner.gamma == 0.99  # undiscounted cost -> the usual default
    plan = planner.solve(timesteps=256)
    assert isinstance(plan, PolicyPlan) and plan.metadata.success
    assert planner.num_timesteps == 256 and len(planner.history) == 2
    ctl = planner.get_controller()
    assert isinstance(ctl, NeuralPolicyController)
    # the controller carries the trained weights
    actor = planner.algorithm.params(planner.train_state)["actor"]
    np.testing.assert_allclose(ctl.params["mlp"]["W0"], np.asarray(actor["W0"]))
    tp = planner.solve_trajectory_from(np.zeros(2), tf=0.5)
    assert isinstance(tp, TrajectoryPlan) and tp.trajectory.n_samples == 6
    J, _ = score_trajectory(prob, tp.trajectory)
    np.testing.assert_allclose(tp.metadata.cost, J)
    u, _ = planner.predict(np.zeros((3, 2)))
    assert u.shape == (3, 1)


def test_planner_reads_the_cost_discount_and_rejects_bad_batches():
    class Discounted(HangCost):
        discount_rate = 1.0

    plant = bounded_pendulum()
    prob = StochasticPlanningProblem(
        plant, cost=Discounted(), x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5])
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, n_envs=2, n_steps=16, batch_size=32, verbose=0
    )
    np.testing.assert_allclose(planner.gamma, np.exp(-0.1))
    # the step reward is −g dt; gamma carries the discount (not exp(−ρ t) in r)
    x = jnp.array([1.0, 0.0])
    u = jnp.zeros(1)
    g0 = float(Discounted().g(np.array([1.0, 0.0]), np.array([0.0])))
    _, _, r0, _, _ = planner.env.step(x, 0.0, u, jax.random.PRNGKey(0))
    _, _, r_later, _, _ = planner.env.step(x, 5.0, u, jax.random.PRNGKey(0))
    np.testing.assert_allclose(float(r0), -g0 * 0.1, atol=1e-6)
    np.testing.assert_allclose(float(r_later), float(r0), atol=1e-6)
    with pytest.raises(ValueError):
        ReinforcementLearningPlanner(prob, dt=0.1, algorithm="sac?", verbose=0)


def test_the_discount_has_one_owner_and_an_announced_default():
    class Discounted(HangCost):
        discount_rate = 1.0

    undiscounted = problem(tf=np.inf)
    discounted = StochasticPlanningProblem(
        bounded_pendulum(),
        cost=Discounted(),
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
    )
    small = dict(dt=0.1, hidden=(8, 8), n_envs=4, n_steps=8, batch_size=32, verbose=0)

    def build(prob, **kwargs):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            planner = ReinforcementLearningPlanner(prob, **{**small, **kwargs})
        return planner, [w for w in caught if "undiscounted" in str(w.message)]

    # nothing declared: the default, announced with its horizon
    planner, announced = build(undiscounted)
    assert planner.gamma == 0.99 and len(announced) == 1
    assert "effective horizon of 9.9 s at dt=0.1" in str(announced[0].message)

    # a declared rate, the planner's gamma, or the algorithm's own: silent
    planner, announced = build(discounted)
    np.testing.assert_allclose(planner.gamma, np.exp(-0.1))
    assert not announced
    planner, announced = build(undiscounted, gamma=0.95)
    assert planner.gamma == 0.95 and not announced
    planner, announced = build(undiscounted, algorithm=PPO(gamma=0.9, batch_size=32))
    assert planner.gamma == planner.algorithm.gamma == 0.9 and not announced
    assert "gamma" not in vars(planner)  # the planner keeps no second copy

    with pytest.raises(ValueError, match="two discounts"):
        build(undiscounted, algorithm=PPO(gamma=0.9, batch_size=32), gamma=0.95)


def test_nominal_trajectory_is_the_undisturbed_rollout_from_the_start():
    small = dict(dt=0.1, hidden=(8, 8), n_envs=4, n_steps=8, batch_size=32, verbose=0)

    # no disturbance port: the same law on the same plant as a trajectory solve
    prob = problem(tf=np.inf)
    planner = ReinforcementLearningPlanner(prob, **small)
    key = np.asarray(planner.key)
    nominal = planner.nominal_trajectory(tf=0.5)
    np.testing.assert_array_equal(np.asarray(planner.key), key)  # no draw consumed
    solved = planner.solve_trajectory_from(prob.x_start, tf=0.5).trajectory
    np.testing.assert_array_equal(nominal.x, solved.x)
    np.testing.assert_array_equal(nominal.u, solved.u)

    # a disturbance port: the nominal rollout holds it at its nominal value
    noisy = StochasticPlanningProblem(
        bounded_pendulum(PendulumWithNoisePort()),
        cost=HangCost(),
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        disturbances={"w": Gaussian([0.0], [1.0])},
    )
    planner = ReinforcementLearningPlanner(noisy, **small)
    first, second = (
        planner.nominal_trajectory(tf=0.5),
        planner.nominal_trajectory(tf=0.5),
    )
    np.testing.assert_array_equal(first.x, second.x)
    disturbed = planner.solve_trajectory_from(noisy.x_start, tf=0.5).trajectory
    assert not np.array_equal(first.x, disturbed.x)


def test_solve_keeps_the_full_evaluation_behind_the_reported_cost():
    planner = ReinforcementLearningPlanner(
        problem(tf=np.inf),
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=16,
        batch_size=32,
        verbose=0,
    )
    assert planner.last_evaluation is None
    plan = planner.solve(timesteps=64, n_trials=4)
    report = planner.last_evaluation
    assert report.J.shape == (4,)
    assert plan.metadata.cost == report.mean
    assert plan.metadata.stats["failure_rate"] == report.failure_rate


def test_plain_planning_problem_trains_from_its_single_start():
    plant = bounded_pendulum()
    prob = PlanningProblem(plant, x_start=[0.2, 0.0], cost=HangCost(), tf=np.inf)
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, n_envs=2, n_steps=16, batch_size=32, verbose=0
    )
    np.testing.assert_allclose(np.asarray(planner.carry[0]), [[0.2, 0.0], [0.2, 0.0]])
    planner.learn(32)
    assert planner.num_timesteps == 32


# --- Monte Carlo evaluation ---


def test_monte_carlo_backends_agree_on_a_fixed_law():
    prob = problem(tf=np.inf)
    plant = prob.sys

    ctl = NeuralPolicyController(plant, hidden=(8,), seed=3)  # any fixed static law
    report_jax = MonteCarloEvaluator(
        prob, dt=0.05, n_trials=6, episode_length=1.0, backend="jax", seed=0
    ).evaluate(ctl)
    report_np = MonteCarloEvaluator(
        prob, dt=0.05, n_trials=6, episode_length=1.0, backend="numpy", seed=0
    ).evaluate(ctl)
    assert report_jax.J.shape == (6,) and report_np.J.shape == (6,)
    assert report_jax.failure_rate == 0.0 and report_np.failure_rate == 0.0
    # different draws (JAX vs NumPy streams), same order of magnitude
    assert (
        abs(report_jax.mean - report_np.mean)
        < 0.5 * max(report_jax.mean, report_np.mean) + 0.1
    )
    assert "trials" in str(report_jax)


# --- the off-policy family through the same planner ---


def test_sac_trains_through_the_same_planner_loop():
    from minilink.planning.reinforcement_learning import SAC

    prob = problem(tf=np.inf)
    planner = ReinforcementLearningPlanner(
        prob,
        dt=0.1,
        hidden=(8, 8),
        algorithm="sac",
        n_envs=2,
        n_steps=8,
        batch_size=16,
        learning_starts=16,
        buffer_size=64,
        gradient_steps=4,
        verbose=0,
    )
    assert isinstance(planner.algorithm, SAC) and not planner.algorithm.on_policy
    assert planner.controller.squash == "tanh"
    planner.learn(48)  # three collections: fill the buffer, then two update rounds
    assert planner.replay.size == 48
    assert "critic_loss" in planner.history[-1] and "alpha" in planner.history[-1]
    u = planner.get_controller().action(np.zeros(2))
    assert -4.0 <= u[0] <= 4.0
    tp = planner.solve_trajectory_from(np.zeros(2), tf=0.3)
    assert isinstance(tp, TrajectoryPlan)


# --- the learned law is a System: compile, differentiate, linearize ---


def test_neural_closed_loop_compiles_and_differentiates_under_jax():
    plant = bounded_pendulum()
    ctl = NeuralPolicyController(plant, hidden=(8,), seed=0)
    cl_sys = ctl @ plant
    evaluator = cl_sys.compile(backend="jax", verbose=False)
    x = jnp.array([0.3, 0.0])
    A = jax.jacfwd(lambda x: evaluator.f_trace(x, jnp.zeros(0), 0.0))(x)
    assert A.shape == (2, 2) and bool(jnp.all(jnp.isfinite(A)))
    A_lin = cl_sys.linearize(np.array([0.3, 0.0])).A()
    np.testing.assert_allclose(np.asarray(A), A_lin, atol=1e-6)

    # gradient of a one-step cost with respect to the policy weights, through the diagram
    def J(params):
        x_next = evaluator.rk4_step_trace_p(x, jnp.zeros(0), 0.0, 0.05, params)
        return jnp.sum(x_next**2)

    grads = jax.grad(J)({"ctl": ctl.params, "sys": plant.params})
    assert all(bool(jnp.all(jnp.isfinite(g))) for g in jax.tree_util.tree_leaves(grads))


def test_solve_reports_the_monte_carlo_score_and_rejects_other_criteria():
    prob = problem(tf=np.inf)
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32, verbose=0
    )
    plan = planner.solve(timesteps=64, n_trials=4)
    assert np.isfinite(plan.metadata.cost) and plan.metadata.success
    assert 0.0 <= plan.metadata.stats["failure_rate"] <= 1.0
    assert "truncates" in planner.env.describe()

    worst = StochasticPlanningProblem(
        bounded_pendulum(),
        cost=HangCost(),
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        criterion="worst_case",
    )
    with pytest.raises(NotImplementedError):
        ReinforcementLearningPlanner(worst, dt=0.1, verbose=0)


def test_planner_trains_with_randomized_parameters():
    from minilink.planning.distributions import Particles

    plant = bounded_pendulum()
    prob = StochasticPlanningProblem(
        plant,
        cost=HangCost(),
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        params_distribution={"m": Particles([[0.5], [2.0]])},
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32, verbose=0
    )
    assert (
        planner.env.randomizes_params
        and "randomized params ['m']" in planner.env.describe()
    )
    theta = planner.carry[3]
    assert set(np.unique(np.asarray(theta["m"]))) <= {0.5, 2.0}
    planner.learn(64)
    assert np.all(np.isfinite(np.asarray(planner.carry[0])))


# --- an inner loop as the plant: the action port is its reference, params are nested ---


def test_outer_loop_on_an_inner_loop_with_nested_params():
    from minilink.control import StateFeedbackController
    from minilink.planning.distributions import Particles
    from minilink.planning.evaluation import MonteCarloEvaluator
    from minilink.planning.problems import lookup_param, merge_params

    plant = bounded_pendulum()
    inner = StateFeedbackController(K=[[20.0, 4.0]], xbar=[0.0, 0.0]) @ plant
    inner.inputs["r"].lower_bound = np.array([-1.0, -2.0])
    inner.inputs["r"].upper_bound = np.array([1.0, 2.0])
    inner.state.lower_bound = plant.state.lower_bound
    inner.state.upper_bound = plant.state.upper_bound
    assert "u" not in inner.inputs and list(inner.inputs) == ["r"]

    prob = StochasticPlanningProblem(
        inner,
        cost=HangCost(),
        x0_distribution=Uniform([-0.3, -0.3], [0.3, 0.3]),
        params_distribution={"sys.m": Particles([[0.5], [2.0]])},  # the plant inside
    )
    assert lookup_param(inner.params, "sys.m") == 1.0
    draw = prob.sample_params(0)
    assert set(draw) == {"sys"} and float(draw["sys"]["m"]) in (0.5, 2.0)
    merged = merge_params(inner.params, draw)
    assert merged["ctl"] is not None and merged["sys"]["m"] == float(draw["sys"]["m"])
    assert inner.params["sys"]["m"] == 1.0  # untouched

    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32, verbose=0
    )
    assert planner.env.action_port == "r" and planner.env.m == 2
    planner.learn(64)
    ctl = planner.get_controller()
    u = ctl.action(np.zeros(2))
    assert u.shape == (2,) and -1.0 <= u[0] <= 1.0 and -2.0 <= u[1] <= 2.0
    report = MonteCarloEvaluator(prob, dt=0.1, n_trials=3, episode_length=0.5).evaluate(
        ctl
    )
    assert np.all(np.isfinite(report.J))


def test_a_step_that_blows_up_ends_the_episode_finitely():
    from minilink.core.system import DynamicSystem

    class Explosive(DynamicSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=1, output_dim=1)
            self.inputs["u"].lower_bound = np.array([-1.0])
            self.inputs["u"].upper_bound = np.array([1.0])
            self.state.lower_bound = np.array([-10.0])
            self.state.upper_bound = np.array([10.0])

        def f(self, x, u, t=0.0, params=None):
            return 1e6 * x  # stiff enough to overflow an RK4 step of 1 s

        def h(self, x, u, t=0.0, params=None):
            return x

    class Zero(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return 1.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    prob = StochasticPlanningProblem(
        Explosive(), cost=Zero(), tf=np.inf, x0_distribution=Uniform([1.0], [2.0])
    )
    env = RolloutEnvironment(prob, dt=1.0, episode_length=5.0)
    x_next, t, r, terminated, truncated = env.step(
        jnp.array([1.0]), 0.0, jnp.zeros(1), jax.random.PRNGKey(0)
    )
    assert (
        bool(jnp.all(jnp.isfinite(x_next)))
        and bool(truncated)
        and np.isfinite(float(r))
    )
