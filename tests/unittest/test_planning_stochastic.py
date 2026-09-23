"""Cost horizon/discount, the problem exit rule, distributions, and the stochastic problem."""

import numpy as np
import pytest

from minilink import Pendulum
from minilink.core.costs import CostFunction, QuadraticCost
from minilink.core.distributions import Gaussian, Particles, Sampler, Uniform
from minilink.core.trajectory import Trajectory
from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner
from minilink.planning.problems import (
    PlanningProblem,
    StochasticPlanningProblem,
    as_stochastic,
)


def pendulum():
    plant = Pendulum()
    plant.state.lower_bound = np.array([-2 * np.pi, -10.0])
    plant.state.upper_bound = np.array([2 * np.pi, 10.0])
    return plant


def quadratic(plant):
    return QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1))


# --- R1: cost horizon and discount, problem exit rule ---


def test_horizon_is_the_problem_tf_and_the_cost_only_discounts():
    plant = pendulum()
    cost = quadratic(plant)
    assert PlanningProblem(plant, cost=cost, tf=5.0).horizon_kind() == "finite"
    assert PlanningProblem(plant, cost=cost, tf=np.inf).horizon_kind() == "infinite"
    assert PlanningProblem(plant, cost=cost).horizon_kind() == "infinite"
    assert not hasattr(CostFunction, "horizon")  # one owner: the problem's tf

    class Discounted(CostFunction):
        discount_rate = 0.5

        def g(self, x, u, t=0.0, params=None):
            return 0.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    np.testing.assert_allclose(Discounted().discount_factor(0.1), np.exp(-0.05))
    assert cost.discount_factor(0.1) == 1.0


def test_problem_exit_rule_and_penalty():
    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), tf=np.inf)
    assert problem.infeasible_cost is None
    assert problem.infeasible_penalty(np.zeros(2)) is None
    assert problem.X.contains(np.array([1e9, -1e9]))  # unconstrained by default
    assert problem.horizon_kind() == "infinite"

    scalar = PlanningProblem(plant, tf=2.0, infeasible_cost=50)
    assert scalar.infeasible_cost == 50.0
    assert scalar.infeasible_penalty(np.zeros(2), 1.0) == 50.0
    assert scalar.horizon_kind() == "finite"

    shaped = PlanningProblem(
        plant, infeasible_cost=lambda x, t: 10.0 * float(x[0] ** 2)
    )
    assert shaped.infeasible_penalty(np.array([2.0, 0.0])) == 40.0


def test_dp_reads_the_problem_infeasible_cost():
    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), infeasible_cost=42.0)
    planner = DynamicProgrammingPlanner(problem, x_grid=(5, 5), u_grid=(3,), dt=0.05)
    assert planner.options.out_of_bound_cost == 42.0
    explicit = DynamicProgrammingPlanner(
        problem, x_grid=(5, 5), u_grid=(3,), dt=0.05, out_of_bound_cost=7.0
    )
    assert explicit.options.out_of_bound_cost == 7.0


def test_dp_reads_the_cost_discount_rate():
    class Discounted(CostFunction):
        discount_rate = 1.0

        def g(self, x, u, t=0.0, params=None):
            return 0.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    plant = pendulum()
    problem = PlanningProblem(plant, cost=Discounted())
    planner = DynamicProgrammingPlanner(problem, x_grid=(5, 5), u_grid=(3,), dt=0.05)
    np.testing.assert_allclose(planner.options.alpha, np.exp(-0.05))
    explicit = DynamicProgrammingPlanner(
        problem, x_grid=(5, 5), u_grid=(3,), dt=0.05, alpha=0.9
    )
    assert explicit.options.alpha == 0.9


# --- R2: distributions and the stochastic problem ---


def test_distributions_sample_on_numpy():
    rng = np.random.default_rng(0)
    gauss = Gaussian([1.0, 2.0], 0.1)
    assert gauss.sample(rng).shape == (2,)
    assert gauss.sample(rng, n=5).shape == (5, 2)
    np.testing.assert_allclose(gauss.mean, [1.0, 2.0])

    box = Uniform([-1.0, 0.0], [1.0, 2.0])
    samples = box.sample(3, n=100)
    assert samples.shape == (100, 2)
    assert box.support.contains(samples[0])
    np.testing.assert_allclose(box.mean, [0.0, 1.0])

    pts = Particles([[0.0, 0.0], [1.0, 1.0]])
    assert pts.sample(rng, n=4).shape == (4, 2)

    custom = Sampler(lambda key: np.array([1.0, -1.0]), mean=[1.0, -1.0])
    assert custom.sample(rng, n=2).shape == (2, 2)


@pytest.mark.optional
@pytest.mark.jax
def test_distributions_sample_and_trace_on_jax():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    key = jax.random.PRNGKey(0)
    for dist in (
        Gaussian([0.0, 0.0], [1.0, 2.0]),
        Uniform([-1.0, -1.0], [1.0, 1.0]),
        Particles([[0.0, 1.0], [2.0, 3.0]]),
        Sampler(lambda k: jax.random.normal(k, (2,)), mean=[0.0, 0.0]),
    ):
        one = dist.sample(key)
        assert one.shape == (2,)
        many = jax.jit(lambda k: dist.sample(k, n=8))(key)
        assert many.shape == (8, 2)
        assert jnp.all(jnp.isfinite(many))


def test_stochastic_problem_is_a_planning_problem_with_a_nominal_bridge():
    plant = pendulum()
    problem = StochasticPlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=np.inf,
        x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
        params_distribution={"m": Uniform([0.8], [1.2])},
        infeasible_cost=100.0,
    )
    assert isinstance(problem, PlanningProblem)
    np.testing.assert_allclose(problem.x_start, [0.0, 0.0])
    assert problem.X0.contains(np.array([1.0, 0.5]))  # the support of the draw
    assert problem.sample_x0(0, n=4).shape == (4, 2)
    assert set(problem.sample_params(0)) == {"m"}

    nominal = problem.nominal()
    assert type(nominal) is PlanningProblem
    np.testing.assert_allclose(nominal.x_start, [0.0, 0.0])
    assert nominal.infeasible_cost == 100.0

    with pytest.raises(ValueError):
        StochasticPlanningProblem(plant, x0_distribution=Gaussian([0.0], 1.0))
    with pytest.raises(ValueError):
        StochasticPlanningProblem(
            plant,
            x0_distribution=Gaussian([0.0, 0.0], 1.0),
            params_distribution={"nope": Uniform([0.0], [1.0])},
        )


def test_as_stochastic_is_the_inverse_bridge_of_nominal():
    plant = pendulum()
    deterministic = PlanningProblem(
        plant,
        x_start=[0.3, -0.1],
        cost=quadratic(plant),
        tf=5.0,
        infeasible_cost=10.0,
    )
    stochastic = as_stochastic(deterministic)
    assert type(stochastic) is StochasticPlanningProblem
    # every draw is the single start, and nothing else is random
    np.testing.assert_allclose(stochastic.sample_x0(0, n=3), [[0.3, -0.1]] * 3)
    assert stochastic.sample_params(0) == {} and stochastic.sample_disturbances(0) == {}
    assert stochastic.criterion == "expectation"
    assert (stochastic.tf, stochastic.infeasible_cost) == (
        5.0,
        10.0,
    )
    np.testing.assert_allclose(stochastic.nominal().x_start, [0.3, -0.1])
    assert as_stochastic(stochastic) is stochastic


# --- the shared scoring contract ---


def test_evaluate_trajectory_applies_the_discount_rate():
    from minilink.core.trajectory import Trajectory

    class Unit(CostFunction):
        discount_rate = 1.0

        def g(self, x, u, t=0.0, params=None):
            return 1.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    t = np.linspace(0.0, 1.0, 101)
    traj = Trajectory(t=t, x=np.zeros((2, t.size)), u=np.zeros((1, t.size)))
    J = Unit().evaluate_trajectory(traj).signals["cost"][0, -1]
    np.testing.assert_allclose(J, 1.0 - np.exp(-1.0), atol=1e-4)  # int_0^1 e^{-t} dt


def test_score_trajectory_cuts_at_the_exit_and_charges_the_problem_price():
    from minilink.core.trajectory import Trajectory
    from minilink.planning.evaluation import score_trajectory

    plant = pendulum()
    plant.state.lower_bound = np.array([-1.0, -10.0])
    plant.state.upper_bound = np.array([1.0, 10.0])

    class Unit(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return 1.0

        def h(self, x, t=0.0, params=None):
            return 7.0

    t = np.linspace(0.0, 1.0, 11)
    x = np.zeros((2, t.size))
    u = np.zeros((1, t.size))
    inside = Trajectory(t=t, x=x, u=u)
    x_exit = x.copy()
    x_exit[0, 5:] = 2.0  # leaves the box at t = 0.5
    exits = Trajectory(t=t, x=x_exit, u=u)

    finite = PlanningProblem(
        plant, cost=Unit(), tf=1.0, X=plant.state.box, infeasible_cost=50.0
    )
    J, failed = score_trajectory(finite, inside)
    assert not failed and np.isclose(J, 1.0 + 7.0)  # running cost + h at tf
    J, failed = score_trajectory(finite, exits)
    assert failed and np.isclose(J, 0.5 + 50.0)  # cut at the exit sample, charged, no h

    unpriced = PlanningProblem(plant, cost=Unit(), tf=np.inf, X=plant.state.box)
    J, failed = score_trajectory(unpriced, exits)
    assert failed and np.isinf(J)  # no price declared, none derived: infinity itself
    J, failed = score_trajectory(unpriced, exits, infeasible_cost=30.0)
    assert failed and np.isclose(J, 0.5 + 30.0)  # the evaluator's derived bound
    free = PlanningProblem(plant, cost=Unit(), tf=np.inf)  # unconstrained: no failure
    J, failed = score_trajectory(free, exits)
    assert not failed and np.isclose(J, 1.0)


def test_every_backend_scores_the_constraint_set_on_its_parameters_and_time():
    """A parametric, time-varying X is read with problem.params.sets at each sample time."""
    from minilink.control import StateFeedbackController
    from minilink.core.backends import array_module
    from minilink.core.sets import Set
    from minilink.planning.evaluation import MonteCarloEvaluator
    from minilink.planning.problems import ProblemParameters

    class Band(Set):
        """|z_i| <= r - 0.2 t: a band narrowing from its half-width r (1 by default)."""

        def margin(self, z, t=0.0, params=None):
            r0 = 1.0 if params is None else params["r"]
            xp = array_module(z)

            r = r0 - 0.2 * t

            return xp.concatenate([z + r, r - z])

    plant = pendulum()
    ctl = StateFeedbackController(K=[[-20.0, 0.0]])  # u = 20 theta: the loop diverges
    backends = ("numpy", "simulator") + (("jax",) if jax_available() else ())
    for r, leaves in ((0.5, True), (3.0, False)):
        problem = PlanningProblem(
            plant,
            x_start=[0.1, 0.0],
            cost=quadratic(plant),
            tf=np.inf,
            X=Band(),
            params=ProblemParameters(sets={"r": r}),
            infeasible_cost=1.0,
        )
        reports = {
            backend: MonteCarloEvaluator(
                problem, dt=0.01, n_trials=1, episode_length=1.0, backend=backend
            ).evaluate(ctl)
            for backend in backends
        }
        assert all(bool(report.failed[0]) == leaves for report in reports.values())
        if "jax" in reports:
            np.testing.assert_allclose(reports["numpy"].J, reports["jax"].J, rtol=1e-6)
        # the simulator integrates the continuous-time loop: same contract, O(dt) apart
        np.testing.assert_allclose(reports["simulator"].J, reports["numpy"].J, rtol=0.1)


@pytest.mark.optional
@pytest.mark.jax
def test_monte_carlo_backends_share_the_score_on_identical_starts():
    pytest.importorskip("jax")
    from minilink.control import NeuralPolicyController
    from minilink.core.distributions import Particles
    from minilink.planning.evaluation import MonteCarloEvaluator

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])

    class Discounted(CostFunction):
        discount_rate = 0.2

        def g(self, x, u, t=0.0, params=None):
            return x[0] ** 2 + 0.01 * u[0] ** 2

        def h(self, x, t=0.0, params=None):
            return 3.0

    ctl = NeuralPolicyController(plant, hidden=(8,), seed=5)  # bounded by construction
    one_start = Particles([[2.5, 0.0]])
    for kwargs in (
        {"tf": np.inf},
        {"tf": 1.0, "infeasible_cost": 20.0},
    ):
        problem = StochasticPlanningProblem(
            plant, cost=Discounted(), x0_distribution=one_start, **kwargs
        )
        reports = {
            backend: MonteCarloEvaluator(
                problem, dt=0.05, n_trials=3, episode_length=1.0, backend=backend
            ).evaluate(ctl)
            for backend in ("jax", "numpy", "simulator")
        }
        np.testing.assert_allclose(reports["jax"].J, reports["numpy"].J, rtol=1e-6)
        # the simulator integrates the continuous-time loop: same contract, O(dt) apart
        np.testing.assert_allclose(reports["simulator"].J, reports["numpy"].J, rtol=0.1)
        assert reports["jax"].value("worst_case") == reports["jax"].worst


@pytest.mark.optional
@pytest.mark.jax
def test_monte_carlo_applies_the_law_beyond_the_port_bounds():
    """Port bounds are information: no backend clips a law that exceeds them."""
    pytest.importorskip("jax")
    from minilink.control import StateFeedbackController
    from minilink.core.distributions import Particles
    from minilink.planning.evaluation import MonteCarloEvaluator

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-0.5])
    plant.inputs["u"].upper_bound = np.array([0.5])
    ctl = StateFeedbackController(K=[[30.0, 8.0]])  # |u| = 15 N m at the start

    class Effort(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return u[0] ** 2

        def h(self, x, t=0.0, params=None):
            return 0.0

    problem = StochasticPlanningProblem(
        plant, cost=Effort(), tf=np.inf, x0_distribution=Particles([[0.5, 0.0]])
    )
    reports = {
        backend: MonteCarloEvaluator(
            problem, dt=0.01, n_trials=1, episode_length=1.0, backend=backend
        ).evaluate(ctl)
        for backend in ("jax", "numpy", "simulator")
    }
    saturated_bound = 0.5**2 * 1.0
    assert reports["numpy"].J[0] > 10 * saturated_bound
    np.testing.assert_allclose(reports["jax"].J, reports["numpy"].J, rtol=1e-6)
    np.testing.assert_allclose(reports["simulator"].J, reports["numpy"].J, rtol=0.1)


def test_simulator_backend_scores_a_controller_with_internal_state():
    """The controller state is stacked first; the score reads the plant's own trajectory."""
    from minilink import PID, SingleMass
    from minilink.control import StateFeedbackController
    from minilink.core.distributions import Particles
    from minilink.planning.evaluation import MonteCarloEvaluator

    mass = SingleMass()
    problem = StochasticPlanningProblem(
        mass,
        cost=quadratic(mass),
        tf=np.inf,
        x0_distribution=Particles([[0.5, 0.0]]),
    )
    settings = dict(dt=0.01, n_trials=1, episode_length=2.0)
    J_pid = (  # a proportional law with filter states
        MonteCarloEvaluator(problem, backend="simulator", **settings)
        .evaluate(PID(Kp=4.0))
        .J
    )
    J_static = (
        MonteCarloEvaluator(problem, backend="numpy", **settings)
        .evaluate(StateFeedbackController(K=[[4.0, 0.0]]))
        .J
    )
    np.testing.assert_allclose(J_pid, J_static, rtol=0.05)


@pytest.mark.optional
@pytest.mark.jax
def test_randomized_parameters_reach_the_dynamics_on_both_backends():
    pytest.importorskip("jax")
    from minilink.control import NeuralPolicyController
    from minilink.core.distributions import Particles
    from minilink.planning.evaluation import MonteCarloEvaluator

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    ctl = NeuralPolicyController(plant, hidden=(8,), seed=5)
    start = Particles([[0.5, 0.0]])
    heavy = StochasticPlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=np.inf,
        x0_distribution=start,
        params_distribution={"m": Particles([[3.0]])},
    )
    nominal = StochasticPlanningProblem(
        plant, cost=quadratic(plant), tf=np.inf, x0_distribution=start
    )
    for backend in ("jax", "numpy"):
        J_heavy = (
            MonteCarloEvaluator(
                heavy, dt=0.05, n_trials=1, episode_length=1.0, backend=backend
            )
            .evaluate(ctl)
            .J
        )
        J_nominal = (
            MonteCarloEvaluator(
                nominal, dt=0.05, n_trials=1, episode_length=1.0, backend=backend
            )
            .evaluate(ctl)
            .J
        )
        assert not np.isclose(J_heavy[0], J_nominal[0])
        if backend == "jax":
            J_heavy_jax = J_heavy[0]
    np.testing.assert_allclose(J_heavy[0], J_heavy_jax, rtol=1e-6)
    assert plant.params["m"] == 1.0  # the draw never touched the nominal plant


def test_deterministic_planner_warns_on_a_stochastic_problem():
    from minilink.planning.trajectory_optimization.planner import (
        TrajectoryOptimizationPlanner,
    )

    plant = pendulum()
    problem = StochasticPlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=2.0,
        x0_distribution=Uniform([-0.1, -0.1], [0.1, 0.1]),
    )
    with pytest.warns(UserWarning, match="deterministic planner"):
        TrajectoryOptimizationPlanner(
            problem, n_steps=5, transcription="direct_collocation"
        )


def test_gymnasium_view_of_a_stochastic_problem():
    pytest.importorskip("gymnasium")
    from minilink.core.distributions import Particles
    from minilink.interfaces.gymnasium import Sys2Gym

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    problem = StochasticPlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=1.0,
        X=plant.state.box,
        x0_distribution=Particles([[0.3, 0.0], [-0.3, 0.0]]),
        infeasible_cost=50.0,
    )
    env = Sys2Gym.from_problem(problem, dt=0.05)
    starts = {
        tuple(np.round(np.asarray(env.reset(seed=s)[0], dtype=float), 3))
        for s in range(6)
    }
    assert starts <= {(0.3, 0.0), (-0.3, 0.0)} and len(starts) == 2
    # a charged exit: terminated, the price on the reward
    env.x = np.array([2 * np.pi - 0.01, 9.9])
    _, r, terminated, truncated, _ = env.step(np.array([4.0]))
    assert terminated and not truncated and r < -49.0
    # the horizon: terminated with h (= 0 here), not truncated
    env.reset(seed=0)
    for _ in range(20):
        _, r, terminated, truncated, _ = env.step(np.array([0.0]))
    assert terminated


# --- the Evaluation record, and open-loop policies replayed by the evaluator ---


def test_evaluation_of_one_trajectory_prints_a_single_cost():
    from minilink.planning.evaluation import Evaluation, MonteCarloEvaluator

    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), tf=1.0)
    t = np.linspace(0.0, 1.0, 6)
    traj = Trajectory(t=t, x=np.vstack([0.1 * t, np.zeros(6)]), u=np.zeros((1, 6)))
    single = Evaluation.of_trajectory(problem, traj)
    assert single.n_trials == 1 and single.trajectories[0] is traj
    assert str(single).startswith("J = ") and "trials" not in str(single)
    # a deterministic problem is one trial for the evaluator as well
    from minilink.control import StateFeedbackController

    law = StateFeedbackController(K=[[1.0, 0.5]], xbar=[0.0, 0.0])
    report = MonteCarloEvaluator(problem, dt=0.1, n_trials=1, backend="numpy").evaluate(
        law
    )
    assert report.n_trials == 1 and np.all(report.x0[0] == problem.x_start)


def test_open_loop_source_scores_like_its_replayed_trajectory_on_every_backend():
    from minilink.blocks import TrajectorySource
    from minilink.planning.evaluation import MonteCarloEvaluator, score_trajectory

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    problem = PlanningProblem(plant, x_start=[0.3, 0.0], cost=quadratic(plant), tf=1.0)
    dt, n_steps = 0.1, 10
    t = dt * np.arange(n_steps + 1)
    u = 2.0 * np.sin(3.0 * t)[None, :]
    source = TrajectorySource(t, u)
    assert int(source.m) == 0  # no input port: an open-loop policy

    # the same held-input rollout by hand, scored under the contract
    evaluator = plant.compile(backend="numpy", verbose=False)
    x = np.zeros((2, n_steps + 1))
    x[:, 0] = problem.x_start
    for k in range(n_steps):
        x[:, k + 1] = evaluator.rk4_step(x[:, k], u[:, k], t[k], dt)
    J_hand, _ = score_trajectory(problem, Trajectory(t=t, x=x, u=u))

    reports = {}
    for backend in ("numpy", "simulator") + (("jax",) if jax_available() else ()):
        reports[backend] = MonteCarloEvaluator(
            problem, dt=dt, n_trials=1, backend=backend
        ).evaluate(source)
    np.testing.assert_allclose(reports["numpy"].J[0], J_hand, rtol=1e-10)
    if "jax" in reports:
        np.testing.assert_allclose(reports["jax"].J[0], J_hand, rtol=1e-8)
    # the simulator integrates the source in series and lands within its step error
    np.testing.assert_allclose(reports["simulator"].J[0], J_hand, rtol=0.05)

    # a held (zero-order) replay keeps each sample until the next
    held = TrajectorySource(t, u, interpolation="previous")
    empty = np.zeros(0)
    np.testing.assert_allclose(held.h(empty, empty, 0.15), u[:, 1])
    np.testing.assert_allclose(source.h(empty, empty, 0.15), 0.5 * (u[:, 1] + u[:, 2]))
    with pytest.raises(ValueError):
        TrajectorySource(t, u, interpolation="cubic")


def jax_available():
    import importlib.util

    return importlib.util.find_spec("jax") is not None
