"""The RL planner, its environment semantics, the neural policy block, and Monte Carlo evaluation."""

import warnings

import numpy as np
import pytest

from minilink import Pendulum, StepSystem, WhiteNoise
from minilink.core.backends import array_module
from minilink.core.costs import CostFunction, QuadraticCost, StepCost
from minilink.core.distributions import Uniform
from minilink.dynamics.catalog.pendulum.pendulum import NoisyPendulum
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution
from minilink.planning.step_problems import StepPlanningProblem

pytest.importorskip("jax")
import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.control.neural import NeuralPolicy  # noqa: E402
from minilink.learning.reinforcement_learning import (  # noqa: E402
    PPO,
    ReinforcementLearningPlanner,
    RolloutEnvironment,
)
from minilink.planning import evaluate  # noqa: E402

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
    return PlanningProblem(
        plant,
        cost=HangCost(),
        U=plant.input_box,
        X=plant.state.box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        **kwargs,
    )


# --- environment semantics ---


def test_environment_infinite_horizon_truncates_at_the_training_zone_by_default():
    # no X declared: the plant's state box is the training zone, not a constraint
    free = PlanningProblem(
        bounded_pendulum(),
        cost=HangCost(),
        horizon=np.inf,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
    )
    env = RolloutEnvironment(free, dt=0.1, episode_length=0.3)
    assert not env.finite_horizon and np.isfinite(
        env.failure_cost
    )  # bound over the zone
    x, t = jnp.zeros(2), 0.0
    for _ in range(2):
        x, t, r, terminated, truncated = env.step(
            x, t, jnp.zeros(1), jax.random.PRNGKey(0)
        )
        assert not bool(terminated) and not bool(truncated)
    x, t, r, terminated, truncated = env.step(x, t, jnp.zeros(1), jax.random.PRNGKey(0))
    assert bool(truncated) and not bool(terminated)  # episode length reached
    # leaving the training zone: truncated (bootstrap), no charge
    _, _, r_exit, terminated, truncated = env.step(
        jnp.array([3.1, 7.9]), 0.0, jnp.array([4.0]), jax.random.PRNGKey(0)
    )
    assert bool(truncated) and not bool(terminated)
    expected = -0.1 * float(env.cost.g(jnp.array([3.1, 7.9]), jnp.array([4.0]), 0.0))
    np.testing.assert_allclose(
        float(r_exit), expected
    )  # the running cost only, no failure_cost


def test_environment_refuses_a_law_it_would_not_draw():
    """An episode draws the plant's laws only: a law on another number is refused by name."""
    plant = bounded_pendulum()
    cost = QuadraticCost.from_system(plant)
    lawful = PlanningProblem(
        plant,
        cost=cost,
        U=plant.input_box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        distributions={
            "plant": {"m": Uniform(0.8, 1.2)},
            "discount_rate": Uniform(0.1, 0.5),
        },
    )
    with pytest.raises(ValueError, match=r"plant's laws only.*\['discount_rate'\]"):
        RolloutEnvironment(lawful, dt=0.1)


def test_environment_finite_horizon_charges_h_and_the_failure_cost():
    env = RolloutEnvironment(
        problem(horizon=0.2, penalize_failure=True, failure_cost=50.0), dt=0.1
    )
    assert env.finite_horizon and env.failure_cost == 50.0
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
    ctl = NeuralPolicy(plant, hidden=(8,), seed=0)
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
    prob = problem(horizon=np.inf)
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=32, batch_size=32
    )
    assert isinstance(planner.algorithm, PPO)
    assert planner.gamma == 0.99  # undiscounted cost -> the usual default
    solution = planner.solve(n_timesteps=256)
    assert isinstance(solution, PlanningSolution) and solution.success
    assert solution.record.n_timesteps == 256 and solution.record.algorithm == "PPO"
    assert solution.trajectory is None and solution.evaluation is None  # not asked
    assert planner.num_timesteps == 256 and len(planner.history) == 2
    ctl = planner.controller
    assert isinstance(ctl, NeuralPolicy) and isinstance(
        solution.controller, NeuralPolicy
    )
    # the controller and the solution's law carry the trained weights
    actor = planner.algorithm.params(planner.train_state)["policy"]["mlp"]
    np.testing.assert_allclose(ctl.params["mlp"]["W0"], np.asarray(actor["W0"]))
    np.testing.assert_allclose(
        solution.controller.params["mlp"]["W0"], np.asarray(actor["W0"])
    )
    u, _ = planner.predict(np.zeros((3, 2)))
    assert u.shape == (3, 1)


def test_predict_draws_unless_deterministic():
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf), dt=0.1, hidden=(8, 8), n_envs=4, n_steps=8
    )
    x = np.array([[0.1, 0.0], [-0.2, 0.3], [0.4, -0.1], [0.0, 0.2]])
    mean_1, _ = planner.predict(x)
    mean_2, _ = planner.predict(x)
    draw_1, _ = planner.predict(x, deterministic=False)
    draw_2, _ = planner.predict(x, deterministic=False)
    np.testing.assert_array_equal(mean_1, mean_2)
    assert draw_1.shape == mean_1.shape == (4, 1)
    assert not np.array_equal(draw_1, draw_2)
    assert np.all(np.abs(draw_1) <= 4.0)  # mapped onto the input port's bounds
    one, _ = planner.predict(x[0], deterministic=False)
    assert one.shape == (1,)


def test_training_reports_only_when_the_verb_asks(capsys):
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=8,
        batch_size=32,
    )
    assert capsys.readouterr().out == ""
    planner.learn(32, verbose=True)
    logged = capsys.readouterr().out
    assert "ReinforcementLearningPlanner: " in logged and "steps" in logged
    planner.solve(n_timesteps=32)
    assert capsys.readouterr().out == ""
    planner.solve(n_timesteps=32, verbose=True)
    assert "steps" in capsys.readouterr().out


def test_planner_reads_the_problem_discount_and_rejects_bad_batches():
    plant = bounded_pendulum()
    prob = PlanningProblem(
        plant,
        cost=HangCost(),
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        discount_rate=1.0,
        U=plant.input_box,
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, n_envs=2, n_steps=16, batch_size=32
    )
    np.testing.assert_allclose(planner.gamma, np.exp(-0.1))
    # the step reward is −g dt, the running cost at the period's start: gamma
    # carries the discount between ticks, never exp(−ρ t) in r
    x = jnp.array([1.0, 0.0])
    u = jnp.zeros(1)
    g0 = float(planner.env.cost.g(x, u, 0.0))
    _, _, r0, _, _ = planner.env.step(x, 0.0, u, jax.random.PRNGKey(0))
    _, _, r_later, _, _ = planner.env.step(x, 5.0, u, jax.random.PRNGKey(0))
    np.testing.assert_allclose(float(r0), -g0 * 0.1, atol=1e-6)
    np.testing.assert_allclose(float(r_later), float(r0), atol=1e-6)
    with pytest.raises(ValueError):
        ReinforcementLearningPlanner(prob, dt=0.1, method="sac?")


def test_the_discount_has_one_owner_and_an_announced_default():
    undiscounted = problem(horizon=np.inf)
    discounted = PlanningProblem(
        bounded_pendulum(),
        cost=HangCost(),
        U=bounded_pendulum().input_box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        discount_rate=1.0,
    )
    small = dict(dt=0.1, hidden=(8, 8), n_envs=4, n_steps=8, batch_size=32)

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
    planner, announced = build(undiscounted, method=PPO(gamma=0.9, batch_size=32))
    assert planner.gamma == planner.algorithm.gamma == 0.9 and not announced
    assert "gamma" not in vars(planner)  # the planner keeps no second copy

    with pytest.raises(ValueError, match="two discounts"):
        build(undiscounted, method=PPO(gamma=0.9, batch_size=32), gamma=0.95)


def test_nominal_trajectory_is_the_undisturbed_rollout_from_the_start():
    small = dict(dt=0.1, hidden=(8, 8), n_envs=4, n_steps=8, batch_size=32)

    # no disturbance port: the law from the start, no draw consumed
    prob = problem(horizon=np.inf)
    planner = ReinforcementLearningPlanner(prob, **small)
    key = np.asarray(planner.key)
    nominal = planner.nominal_trajectory(tf=0.5)
    np.testing.assert_array_equal(np.asarray(planner.key), key)
    np.testing.assert_allclose(nominal.x[:, 0], prob.x0)

    # a disturbance source: the nominal rollout reads it at its mean
    noisy = PlanningProblem(
        bounded_pendulum(NoisyPendulum()),
        cost=HangCost(),
        U=bounded_pendulum(NoisyPendulum()).input_box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        disturbances={"d": WhiteNoise(1, psd=0.1, sample_period=0.1)},
    )
    planner = ReinforcementLearningPlanner(noisy, **small)
    first, second = (
        planner.nominal_trajectory(tf=0.5),
        planner.nominal_trajectory(tf=0.5),
    )
    np.testing.assert_array_equal(first.x, second.x)


def test_solve_with_evaluate_carries_the_rollout_and_the_full_evaluation():
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=16,
        batch_size=32,
        episode_length=0.5,  # episodes end inside a collection: the record's cost is a number
    )
    solution = planner.solve(n_timesteps=64, evaluate=True, n_trials=4)
    assert solution.evaluation.J.shape == (4,)
    assert 0.0 <= solution.evaluation.feasibility_rate <= 1.0
    np.testing.assert_array_equal(solution.trajectory.x, planner.nominal_trajectory().x)
    assert np.isfinite(solution.record.cost) and "plant steps" in str(solution.record)
    assert solution.record.history[-1]["cost"] == -planner.history[-1]["ep_return_mean"]
    assert "feedback" in str(solution)
    # the critic estimates the training discount, not the problem's: no cost-to-go
    assert solution.cost_to_go is None


def test_plain_planning_problem_trains_from_its_single_start():
    plant = bounded_pendulum()
    prob = PlanningProblem(
        plant, x0=[0.2, 0.0], cost=HangCost(), horizon=np.inf, U=plant.input_box
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, n_envs=2, n_steps=16, batch_size=32
    )
    np.testing.assert_allclose(np.asarray(planner.carry[0]), [[0.2, 0.0], [0.2, 0.0]])
    planner.learn(32)
    assert planner.num_timesteps == 32


# --- the evaluation of a learned law ---


def test_the_backends_score_a_fixed_law_on_their_own_draws():
    prob = problem(horizon=np.inf)
    plant = prob.sys

    ctl = NeuralPolicy(plant, hidden=(8,), seed=3)  # any fixed static law
    settings = dict(dt=0.05, n_trials=6, episode_length=1.0, key=0)
    report_jax = evaluate(prob, ctl, backend="jax", **settings)
    report_np = evaluate(prob, ctl, backend="numpy", **settings)
    assert report_jax.J.shape == (6,) and report_np.J.shape == (6,)
    assert report_jax.feasibility_rate == 1.0 and report_np.feasibility_rate == 1.0
    # different draws (JAX vs NumPy streams), same order of magnitude
    mean_jax, mean_np = float(report_jax.mean), float(report_np.mean)
    assert abs(mean_jax - mean_np) < 0.5 * max(mean_jax, mean_np) + 0.1
    assert "runs" in str(report_jax)


def test_the_evaluation_refuses_a_period_that_does_not_divide_the_horizon():
    prob = problem(horizon=0.25)
    ctl = NeuralPolicy(prob.sys, hidden=(8,), seed=3)
    for backend in ("numpy", "jax"):
        with pytest.raises(ValueError, match="divide"):
            evaluate(prob, ctl, dt=0.1, n_trials=2, backend=backend)


# --- the off-policy family through the same planner ---


def test_sac_trains_through_the_same_planner_loop():
    from minilink.learning.reinforcement_learning import SAC

    prob = problem(horizon=np.inf)
    planner = ReinforcementLearningPlanner(
        prob,
        dt=0.1,
        hidden=(8, 8),
        method="sac",
        n_envs=2,
        n_steps=8,
        batch_size=16,
        learning_starts=16,
        buffer_size=64,
        gradient_steps=4,
    )
    assert isinstance(planner.algorithm, SAC) and not planner.algorithm.on_policy
    assert planner.actor.squash == "tanh"
    planner.learn(48)  # three collections: fill the buffer, then two update rounds
    assert planner.replay.size == 48
    assert "critic_loss" in planner.history[-1] and "alpha" in planner.history[-1]
    u = planner.actor.action(np.zeros(2))
    assert -4.0 <= u[0] <= 4.0


def test_squashed_head_log_density_is_finite_outside_the_square():
    from minilink.learning.reinforcement_learning.policy import SquashedGaussianHead

    head = SquashedGaussianHead(n_features=2, m=2, hidden=(4,))
    params = head.init()
    mu = jnp.zeros(2)
    z = jnp.zeros(2)
    for a in ([1.5, 0.0], [-2.0, 0.3], [0.2, -0.4]):
        logp = head.log_prob(params, mu, z, jnp.asarray(a))
        assert bool(jnp.isfinite(logp)), a


def test_sac_reused_across_plants_targets_each_plant_input_dimension():
    from minilink.core.costs import QuadraticCost
    from minilink.core.system import DynamicSystem
    from minilink.learning.reinforcement_learning import SAC

    class ThreeAxisPoint(DynamicSystem):
        """A point driven in velocity on three axes: dx = u."""

        def __init__(self):
            super().__init__(state_dim=3, input_dim=3, output_dim=3)
            self.state.lower_bound = -2.0 * np.ones(3)
            self.state.upper_bound = 2.0 * np.ones(3)
            self.inputs["u"].lower_bound = -np.ones(3)
            self.inputs["u"].upper_bound = np.ones(3)

        def f(self, x, u, t=0.0, params=None):
            return 1.0 * u

    point = ThreeAxisPoint()
    three_inputs = PlanningProblem(
        point,
        cost=QuadraticCost.from_system(point),
        X=point.state.box,
        x0=Uniform(-0.5 * np.ones(3), 0.5 * np.ones(3)),
        U=point.input_box,
    )

    def sac_planner(prob, sac):
        return ReinforcementLearningPlanner(
            prob,
            dt=0.1,
            hidden=(8,),
            method=sac,
            gamma=0.99,
            n_envs=2,
            n_steps=8,
        )

    def sac():
        return SAC(batch_size=16, learning_starts=16, buffer_size=64, gradient_steps=2)

    reused = sac()
    sac_planner(problem(horizon=np.inf), reused)  # bound first to a one-input pendulum
    assert reused.target_entropy is None  # the user's choice, kept
    planner = sac_planner(three_inputs, reused)
    fresh = sac_planner(three_inputs, sac())
    planner.learn(32)
    fresh.learn(32)
    np.testing.assert_allclose(
        np.asarray(planner.train_state["log_alpha"]),
        np.asarray(fresh.train_state["log_alpha"]),
    )


def test_replay_buffer_refuses_a_draw_before_any_transition():
    from minilink.learning.reinforcement_learning.collect import ReplayBuffer

    buffer = ReplayBuffer(8, n=2, m=1)
    with pytest.raises(ValueError, match="replay buffer is empty"):
        buffer.sample(jax.random.PRNGKey(0), 4)
    one = {
        "x": jnp.zeros((1, 2)),
        "a": jnp.zeros((1, 1)),
        "reward": jnp.ones(1),
        "x_next": jnp.zeros((1, 2)),
        "terminated": jnp.zeros(1),
    }
    buffer.add(one)
    assert buffer.sample(jax.random.PRNGKey(0), 4)["reward"].shape == (4,)


# --- the learned law is a System: compile, differentiate, linearize ---


def test_neural_closed_loop_compiles_and_differentiates_under_jax():
    plant = bounded_pendulum()
    ctl = NeuralPolicy(plant, hidden=(8,), seed=0)
    cl_sys = ctl @ plant
    evaluator = cl_sys.compile(backend="jax", verbose=False)
    x = jnp.array([0.3, 0.0])
    A = jax.jacfwd(lambda x: evaluator.f(x, jnp.zeros(0), 0.0))(x)
    assert A.shape == (2, 2) and bool(jnp.all(jnp.isfinite(A)))
    A_lin = cl_sys.linearize(np.array([0.3, 0.0])).A()
    np.testing.assert_allclose(np.asarray(A), A_lin, atol=1e-6)

    # gradient of a one-step cost with respect to the policy weights, through the diagram
    def J(params):
        x_next = evaluator.step(x, jnp.zeros(0), 0.0, params, dt=0.05)
        return jnp.sum(x_next**2)

    grads = jax.grad(J)({"controller": ctl.params, "plant": plant.params})
    assert all(bool(jnp.all(jnp.isfinite(g))) for g in jax.tree_util.tree_leaves(grads))


def test_solve_reports_the_monte_carlo_score_and_rejects_other_criteria():
    prob = problem(horizon=np.inf)
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32
    )
    solution = planner.solve(n_timesteps=64, evaluate=True, n_trials=4)
    assert np.isfinite(solution.evaluation.mean) and solution.success
    assert 0.0 <= solution.evaluation.feasibility_rate <= 1.0
    assert "truncates" in planner.env.describe()

    worst = PlanningProblem(
        bounded_pendulum(),
        cost=HangCost(),
        U=bounded_pendulum().input_box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        criterion="worst_case",
    )
    with pytest.raises(NotImplementedError):
        ReinforcementLearningPlanner(worst, dt=0.1)


def test_planner_trains_with_randomized_parameters():
    from minilink.core.distributions import Particles

    plant = bounded_pendulum()
    prob = PlanningProblem(
        plant,
        cost=HangCost(),
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        distributions={"plant": {"m": Particles([[0.5], [2.0]])}},
        U=plant.input_box,
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32
    )
    assert (
        planner.env.randomizes_params
        and "randomized params ['m']" in planner.env.describe()
    )
    episode = planner.carry[3]
    assert set(np.unique(np.asarray(episode["plant"]["m"]))) <= {0.5, 2.0}
    planner.learn(64)
    assert np.all(np.isfinite(np.asarray(planner.carry[0])))


# --- an inner loop as the plant: the action port is its reference, params are nested ---


def test_outer_loop_on_an_inner_loop_with_nested_params():
    from minilink.control import StateFeedback
    from minilink.core.distributions import Particles
    from minilink.core.params import lookup, merge_tree

    plant = bounded_pendulum()
    inner = StateFeedback(K=[[20.0, 4.0]], x_bar=[0.0, 0.0]) @ plant
    inner.inputs["r"].lower_bound = np.array([-1.0, -2.0])
    inner.inputs["r"].upper_bound = np.array([1.0, 2.0])
    inner.state.lower_bound = plant.state.lower_bound
    inner.state.upper_bound = plant.state.upper_bound
    assert "u" not in inner.inputs and list(inner.inputs) == ["r"]

    prob = PlanningProblem(
        inner,
        cost=HangCost(),
        x0=Uniform([-0.3, -0.3], [0.3, 0.3]),
        distributions={
            "plant": {"plant": {"m": Particles([[0.5], [2.0]])}}
        },  # the plant inside
        U=inner.input_box,
    )
    assert lookup(inner.params, "plant.m") == 1.0
    draw = prob.sample_params(0)["plant"]
    assert set(draw) == {"plant"} and float(draw["plant"]["m"]) in (0.5, 2.0)
    merged = merge_tree(inner.params, draw)
    assert merged["controller"] is not None and merged["plant"]["m"] == float(
        draw["plant"]["m"]
    )
    assert inner.params["plant"]["m"] == 1.0  # untouched

    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32
    )
    assert planner.env.action_port == "r" and planner.env.m == 2
    planner.learn(64)
    ctl = planner.actor
    u = ctl.action(np.zeros(2))
    assert u.shape == (2,) and -1.0 <= u[0] <= 1.0 and -2.0 <= u[1] <= 2.0
    report = evaluate(prob, ctl, dt=0.1, n_trials=3, episode_length=0.5)
    assert np.all(np.isfinite(report.J))


def test_a_step_that_blows_up_ends_the_episode_finitely():
    from minilink.core.system import DynamicSystem

    class Explosive(DynamicSystem):
        def __init__(self):
            super().__init__(state_dim=1, input_dim=1, output_dim=1)
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

    prob = PlanningProblem(
        Explosive(), cost=Zero(), horizon=np.inf, x0=Uniform([1.0], [2.0])
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


# --- the policy-gradient ladder, and the pieces the algorithms share ---


@pytest.mark.parametrize("algorithm", ["reinforce", "actor_critic"])
def test_reinforce_and_actor_critic_train_through_the_same_planner(algorithm):
    from minilink.learning.reinforcement_learning import REINFORCE, ActorCritic

    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        hidden=(8, 8),
        method=algorithm,
        n_envs=4,
        n_steps=16,
        episode_length=0.5,
    )
    expected = {"reinforce": REINFORCE, "actor_critic": ActorCritic}[algorithm]
    assert isinstance(planner.algorithm, expected) and planner.algorithm.on_policy
    if algorithm == "reinforce":
        assert planner.critic is None and planner.n_steps == 5  # one episode length
    else:
        assert planner.critic is not None and planner.n_steps == 16
    before = np.asarray(planner.actor.params["mlp"]["W0"]).copy()
    planner.learn(2 * planner.n_steps * planner.n_envs)
    assert len(planner.history) == 2
    assert not np.array_equal(planner.actor.params["mlp"]["W0"], before)
    solution = planner.solve(n_timesteps=planner.n_steps * planner.n_envs)
    assert (
        solution.success
        and solution.record.algorithm == type(planner.algorithm).__name__
    )


def test_gae_limits_are_the_td_error_and_the_return_minus_the_baseline():
    from minilink.learning.reinforcement_learning import gae, returns_to_go

    key = jax.random.PRNGKey(0)
    reward, value, last_value = (
        jax.random.normal(k, shape)
        for k, shape in zip(jax.random.split(key, 3), ((6, 2), (6, 2), (2,)))
    )
    done = jnp.zeros((6, 2)).at[3, 0].set(1.0)
    gamma = 0.9
    value_next = jnp.concatenate([value[1:], last_value[None]])
    delta = reward + gamma * value_next * (1.0 - done) - value

    np.testing.assert_allclose(gae(delta, done, gamma, 0.0), delta)
    G, returns = last_value, []
    for k in reversed(range(6)):
        G = reward[k] + gamma * G * (1.0 - done[k])
        returns.append(G)
    returns = jnp.stack(returns[::-1])
    np.testing.assert_allclose(
        gae(delta, done, gamma, 1.0), returns - value, atol=1e-12
    )
    # the Monte Carlo return of an episode stops at its end
    R = returns_to_go(reward, done, gamma)
    np.testing.assert_allclose(R[3, 0], reward[3, 0])
    np.testing.assert_allclose(R[2, 0], reward[2, 0] + gamma * reward[3, 0])


def test_stochastic_policy_and_critics_take_one_state_or_a_batch():
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        hidden=(8,),
        n_envs=2,
        n_steps=8,
        batch_size=16,
    )
    pi, V = planner.exploration_policy, planner.critic
    theta, w = planner.algorithm.params(planner.train_state).values()
    x = jnp.array([[0.1, 0.2], [-0.3, 0.4], [0.5, -0.6]])
    keys = jax.random.split(jax.random.PRNGKey(1), 3)

    a = pi.sample(theta, x, keys)
    logp = pi.log_prob(theta, x, a)
    assert a.shape == (3, 1) and logp.shape == (3,)
    for i in range(3):
        np.testing.assert_allclose(pi.sample(theta, x[i], keys[i]), a[i])
        np.testing.assert_allclose(pi.log_prob(theta, x[i], a[i]), logp[i])
        np.testing.assert_allclose(V.value(w, x[i]), V.value(w, x)[i])
    # the plant input of a normalized action spans the port bounds
    np.testing.assert_allclose(pi.input(jnp.array([1.0])), [4.0])
    np.testing.assert_allclose(pi.input(jnp.array([-3.0])), [-4.0])
    assert pi.deterministic is planner.actor


def test_numpy_and_jax_environments_take_the_same_step():
    prob = problem(horizon=np.inf)
    env_jax = RolloutEnvironment(prob, dt=0.1, episode_length=1.0)
    env_np = RolloutEnvironment(prob, dt=0.1, episode_length=1.0, backend="numpy")
    x, u = np.array([0.3, -0.2]), np.array([1.5])
    x_jax, t_jax, r_jax, term_jax, trunc_jax = env_jax.step(
        jnp.asarray(x), 0.0, jnp.asarray(u), jax.random.PRNGKey(0)
    )
    x_np, t_np, r_np, term_np, trunc_np = env_np.step(
        x, 0.0, u, np.random.default_rng(0)
    )
    np.testing.assert_allclose(np.asarray(x_jax), x_np, atol=1e-12)
    np.testing.assert_allclose(float(r_jax), float(r_np), atol=1e-12)
    assert bool(term_jax) == bool(term_np) and bool(trunc_jax) == bool(trunc_np)
    assert isinstance(x_np, np.ndarray)


def test_cost_to_go_comes_from_the_critic_only_when_the_discounts_match():
    prob = PlanningProblem(
        bounded_pendulum(),
        cost=HangCost(),
        U=bounded_pendulum().input_box,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        discount_rate=1.0,
    )
    planner = ReinforcementLearningPlanner(
        prob, dt=0.1, hidden=(8, 8), n_envs=4, n_steps=16, batch_size=32
    )
    solution = planner.solve(n_timesteps=64)
    x = np.array([0.2, 0.0])
    w = planner.algorithm.params(planner.train_state)["critic"]
    np.testing.assert_allclose(
        solution.cost_to_go(x), -float(planner.critic.value(w, jnp.asarray(x)))
    )

    # The critic is a callable of x alone: the planner reads it at any time
    assert planner.cost_to_go(x, t=3.0) == planner.cost_to_go(x)

    # the same problem trained at another discount estimates something else
    other = ReinforcementLearningPlanner(
        prob,
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=16,
        batch_size=32,
        gamma=0.5,
    )
    assert other.solve(n_timesteps=64).cost_to_go is None
    with pytest.raises(
        ValueError, match="ReinforcementLearningPlanner gives no cost-to-go"
    ):
        other.cost_to_go(x)


def test_the_planner_trains_on_a_substepped_step_model():
    """Two RK4 substeps per control period: the environment's step is the step model's."""
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        substeps=2,
        hidden=(8, 8),
        n_envs=4,
        n_steps=8,
        batch_size=32,
    )
    solution = planner.solve(n_timesteps=32)
    env = planner.env
    x, u = np.array([0.3, 0.0]), np.array([0.5])
    expected = env.sys.compile(verbose=False).step(x, u, 0.0, dt=0.1, substeps=2)
    np.testing.assert_allclose(
        np.asarray(env.plant_step(x, u, 0.0)), np.asarray(expected)
    )
    assert solution.controller is not None


def test_angle_features_map_each_angle_to_its_cosine_and_sine():
    from minilink.control import angle_features

    features = angle_features([0], scales={1: 0.1})
    x = np.array([np.pi / 3, 4.0])

    # z = [cos θ, sin θ, 0.1 ω]: periodic in θ, the rate scaled, on NumPy and under jit
    z = features(x)
    np.testing.assert_allclose(z, [0.5, np.sqrt(3) / 2, 0.4])
    np.testing.assert_allclose(features(x + [2 * np.pi, 0.0]), z, atol=1e-12)
    np.testing.assert_allclose(jax.jit(features)(jnp.asarray(x)), z, atol=1e-12)


def test_the_heads_draw_at_a_counter_key_as_the_seeded_policy_block_does():
    """A head drawing at ``CounterKey(seed, k)`` makes the block's draw at that seed and sample."""
    from minilink import Gain
    from minilink.control import GaussianPolicy
    from minilink.core.backends import require_jax
    from minilink.core.distributions import CounterKey
    from minilink.learning.reinforcement_learning.policy import GaussianHead

    require_jax()  # the library's 64-bit policy, before the first JAX array
    law = Gain(np.array([[2.0, 1.0]]))
    block = GaussianPolicy(law, np.log([0.5]), sample_period=0.05, seed=7)
    head = GaussianHead(1)
    x = np.array([0.3, -0.2])
    mu = jnp.asarray(law.outputs["y"].compute(None, x, 0.0))
    for k in (0, 1, 13):
        a = head.sample(
            {"log_std": jnp.log(jnp.array([0.5]))}, mu, None, CounterKey(7, k)
        )
        np.testing.assert_allclose(
            np.asarray(a), block.h(None, x, k * 0.05 + 0.01), rtol=1e-12
        )


def test_the_collector_draws_each_episode_at_its_own_policy_seed():
    """The carry holds one policy seed per plant; a finished episode draws a fresh one."""
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        episode_length=0.3,
        hidden=(8, 8),
        n_envs=4,
        n_steps=8,
        batch_size=32,
    )
    seeds = np.asarray(planner.carry[4])
    assert seeds.shape == (4,) and len(set(seeds.tolist())) == 4
    planner.solve(n_timesteps=32)
    assert not np.array_equal(np.asarray(planner.carry[4]), seeds)


def test_the_record_states_a_training_discount_that_is_not_the_scored_one():
    """The problem's discount by default; an override is a training choice the record names."""
    kwargs = dict(dt=0.1, hidden=(8, 8), n_envs=2, n_steps=8, batch_size=16)
    prob = problem(horizon=np.inf, discount_rate=0.5)
    derived = ReinforcementLearningPlanner(prob, **kwargs).solve(n_timesteps=16).record
    assert derived.gamma == derived.scored_gamma == np.exp(-prob.discount_rate * 0.1)
    assert not derived.discount_differs and "scored at" not in str(derived)

    override = ReinforcementLearningPlanner(prob, gamma=0.9, **kwargs)
    record = override.solve(n_timesteps=16).record
    assert record.gamma == 0.9 and record.discount_differs
    assert "trained at gamma=0.9" in str(record)


def test_environment_reward_stays_finite_when_a_step_blows_up():
    env = RolloutEnvironment(
        problem(horizon=0.2, penalize_failure=True, failure_cost=50.0),
        dt=0.1,
        backend="numpy",
    )
    env.face.transition_at = lambda x, u, t, tree: (np.array([np.inf, 0.0]), np.inf)
    x = np.array([0.1, 0.0])
    x_next, _, reward, terminated, _ = env.step(x, 0.0, np.array([0.0]), None)
    assert np.all(np.isfinite(x_next)) and np.isfinite(reward) and bool(terminated)


def test_training_again_leaves_an_earlier_solution_as_it_was():
    """Each solution keeps the law it was solved with; the planner trains its own."""
    planner = ReinforcementLearningPlanner(
        problem(horizon=np.inf),
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=32,
        batch_size=32,
    )
    first = planner.solve(n_timesteps=128)
    W0 = np.array(first.controller.params["mlp"]["W0"])
    second = planner.solve(n_timesteps=128)

    assert first.controller is not second.controller
    np.testing.assert_array_equal(first.controller.params["mlp"]["W0"], W0)
    assert not np.array_equal(second.controller.params["mlp"]["W0"], W0)
    x = np.array([0.3, 0.0])
    u_first = first.controller.outputs["u"].compute(None, x, 0.0)
    np.testing.assert_allclose(u_first, first.controller.action(x))


# --- a native step problem ---


class GridWalker(StepSystem):
    """A walker on a 7 × 7 grid of cells: ``x_{k+1} = x_k + round(u_k)``, a move to a neighbour or none, clipped to the grid."""

    def __init__(self):
        super().__init__(state_dim=2, input_dim=2, output_dim=2)
        self.state.lower_bound[:] = [0.0, 0.0]
        self.state.upper_bound[:] = [6.0, 6.0]
        self.inputs["u"].lower_bound[:] = [-1.0, -1.0]
        self.inputs["u"].upper_bound[:] = [1.0, 1.0]

    def step(self, x, u, k=0, params=None):
        xp = array_module(x)

        x_next = xp.clip(x + xp.round(u), 0.0, 6.0)

        return x_next


class MovesLeft(StepCost):
    """``c = max |x − goal|``: the moves left to the goal, diagonal moves allowed."""

    def __init__(self, goal):
        self.params = {"goal": np.asarray(goal, dtype=float)}

    def c(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        c = xp.max(xp.abs(x - params["goal"]))

        return c

    def h(self, x, k=0, params=None):
        return 0.0 * x[0]


def grid_world(**fields):
    walker = GridWalker()
    return StepPlanningProblem(
        walker,
        cost=MovesLeft([5.0, 3.0]),
        x0=[0.0, 0.0],
        X=walker.state.box,
        U=walker.input_box,
        **fields,
    )


def test_an_rl_planner_trains_on_a_native_grid_world_through_its_face():
    problem = grid_world(gamma=0.9)
    planner = ReinforcementLearningPlanner(
        problem, n_envs=8, n_steps=32, hidden=(32,), episode_length=12, seed=0
    )

    # The environment reads the face: ticks of 1, its γ, its cost bound over an episode
    face = problem.compile()
    env = planner.env
    assert (planner.dt, env.n_steps_per_episode, env.reward) == (1.0, 12, "stage")
    assert planner.gamma == face.gamma == 0.9
    assert env.failure_cost == face.failure_cost_over(12, 0.9)

    # The face scores a law: J = Σ γᵏ c_k over 12 ticks from the start
    def score(controller):
        return face.run(lambda x, k: np.asarray(controller.action(x)), n_steps=12)

    untrained = score(planner.actor)
    solution = planner.solve(n_timesteps=5_000)
    trained = score(solution.controller)

    assert solution.success and solution.record.reward == "stage"
    assert solution.record.gamma == solution.record.scored_gamma == 0.9
    assert solution.cost_to_go is not None
    assert trained[3] < untrained[3]
    assert np.max(np.abs(trained[0][-1] - [5.0, 3.0])) < np.max(
        np.abs(untrained[0][-1] - [5.0, 3.0])
    )

    # Its score is its face's rollout, not the continuous evaluator's: refused before
    # any training
    steps = planner.num_timesteps
    with pytest.raises(NotImplementedError, match="compile\\(\\).run"):
        planner.solve(n_timesteps=256, evaluate=True)
    assert planner.num_timesteps == steps


def test_an_undiscounted_grid_world_trains_at_the_default_discount_in_ticks():
    with pytest.warns(UserWarning, match="ticks; declare gamma on the step problem"):
        planner = ReinforcementLearningPlanner(
            grid_world(), n_envs=2, n_steps=8, hidden=(4,), episode_length=6
        )
    assert planner.gamma == 0.99 and planner.env.face.gamma == 1.0
    with pytest.raises(ValueError, match="own clock"):
        ReinforcementLearningPlanner(grid_world(gamma=0.9), dt=0.1, hidden=(4,))
