"""Cost horizon/discount, the problem exit rule, distributions, and the stochastic problem."""

import numpy as np
import pytest

from minilink import Pendulum, WhiteNoise
from minilink.core.costs import CostFunction, QuadraticCost
from minilink.core.distributions import Gaussian, Particles, Uniform
from minilink.core.trajectory import Trajectory
from minilink.planning.dynamic_programming.dp import DynamicProgrammingPlanner
from minilink.planning.problems import PlanningProblem


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
    assert (
        PlanningProblem(plant, cost=cost, tf=5.0, U=plant.input_box).horizon_kind()
        == "finite"
    )
    assert (
        PlanningProblem(plant, cost=cost, tf=np.inf, U=plant.input_box).horizon_kind()
        == "infinite"
    )
    assert (
        PlanningProblem(plant, cost=cost, U=plant.input_box).horizon_kind()
        == "infinite"
    )
    assert not hasattr(CostFunction, "horizon")  # one owner: the problem's tf

    # The discount has one owner too: the problem
    discounted = PlanningProblem(plant, cost=cost, tf=np.inf, discount_rate=0.5)
    np.testing.assert_allclose(discounted.discount_factor(0.1), np.exp(-0.05))
    assert PlanningProblem(plant, cost=cost).discount_factor(0.1) == 1.0
    assert not hasattr(CostFunction, "discount_rate")
    with pytest.raises(ValueError, match="discount_rate"):
        PlanningProblem(plant, cost=cost, discount_rate=-1.0)


def test_problem_exit_rule_and_penalty():
    plant = pendulum()
    problem = PlanningProblem(
        plant, cost=quadratic(plant), tf=np.inf, U=plant.input_box
    )
    assert problem.infeasible_cost is None
    assert problem.X.contains(np.array([1e9, -1e9]))  # unconstrained by default
    assert problem.horizon_kind() == "infinite"

    scalar = PlanningProblem(plant, tf=2.0, infeasible_cost=50, U=plant.input_box)
    assert scalar.infeasible_cost == 50.0
    assert scalar.infeasibility_price() == 50.0
    assert scalar.horizon_kind() == "finite"

    # The switch: None, True (the bound) or a number; a callable price is refused
    assert PlanningProblem(plant, infeasible_cost=True).infeasible_cost is True
    for refused in (lambda x, t: 1.0, False, "big"):
        with pytest.raises(TypeError, match="infeasible_cost"):
            PlanningProblem(plant, infeasible_cost=refused)


def test_the_cost_bound_integrates_the_cost_maximum_over_the_horizon():
    plant = pendulum()  # state box ±2π × ±10, input box the plant's ±5
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), S=np.eye(2))
    g_max = (2 * np.pi) ** 2 + 10.0**2 + 5.0**2
    h_max = (2 * np.pi) ** 2 + 10.0**2

    # Finite horizon: g_max T + h_max
    finite = PlanningProblem(
        plant, cost=cost, tf=2.0, X=plant.state.box, U=plant.input_box
    )
    assert finite.cost_bound() == pytest.approx(2.0 * g_max + h_max)
    assert finite.infeasibility_price() == pytest.approx(2.0 * g_max + h_max)

    # Discounted infinite horizon: g_max / ρ, no h
    discounted = PlanningProblem(
        plant,
        cost=cost,
        tf=np.inf,
        X=plant.state.box,
        U=plant.input_box,
        discount_rate=0.5,
    )
    assert discounted.cost_bound() == pytest.approx(g_max / 0.5)

    # Undiscounted infinite horizon: no bound, 1e6 announced
    undiscounted = PlanningProblem(plant, cost=cost, tf=np.inf, U=plant.input_box)
    with pytest.warns(UserWarning, match="infeasible_cost"):
        assert undiscounted.cost_bound() == 1.0e6


def test_dp_reads_the_problem_infeasible_cost():
    plant = pendulum()
    problem = PlanningProblem(
        plant, cost=quadratic(plant), infeasible_cost=42.0, U=plant.input_box
    )
    planner = DynamicProgrammingPlanner(problem, x_grid=(5, 5), u_grid=(3,), dt=0.05)
    assert planner.options.out_of_bound_cost == 42.0
    explicit = DynamicProgrammingPlanner(
        problem, x_grid=(5, 5), u_grid=(3,), dt=0.05, out_of_bound_cost=7.0
    )
    assert explicit.options.out_of_bound_cost == 7.0


def test_dp_reads_the_problem_discount_rate():
    class Zero(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return 0.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    plant = pendulum()
    problem = PlanningProblem(plant, cost=Zero(), U=plant.input_box, discount_rate=1.0)
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
    ):
        one = dist.sample(key)
        assert one.shape == (2,)
        many = jax.jit(lambda k: dist.sample(k, n=8))(key)
        assert many.shape == (8, 2)
        assert jnp.all(jnp.isfinite(many))


def test_stochastic_problem_is_a_planning_problem_with_a_nominal_bridge():
    plant = pendulum()
    problem = PlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=np.inf,
        x_start=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
        params={"plant": {"m": Uniform([0.8], [1.2])}},
        infeasible_cost=100.0,
        U=plant.input_box,
    )
    assert isinstance(problem, PlanningProblem)
    np.testing.assert_allclose(problem.x_start, [0.0, 0.0])
    assert problem.X0.contains(np.array([1.0, 0.5]))  # the support of the draw
    assert problem.sample_x0(0, n=4).shape == (4, 2)
    assert set(problem.sample_params(0)["plant"]) == {"m"}
    assert (
        problem.is_stochastic
        and problem.params["plant"]["m"] is problem.laws["plant"]["m"]
    )

    nominal = problem.nominal()
    assert type(nominal) is PlanningProblem and not nominal.is_stochastic
    np.testing.assert_allclose(nominal.x_start, [0.0, 0.0])
    assert nominal.infeasible_cost == 100.0

    with pytest.raises(ValueError):
        PlanningProblem(plant, x_start=Gaussian([0.0], 1.0), U=plant.input_box)
    with pytest.raises(ValueError):
        PlanningProblem(
            plant,
            x_start=Gaussian([0.0, 0.0], 1.0),
            params={"plant": {"nope": Uniform([0.0], [1.0])}},
            U=plant.input_box,
        )


def test_a_point_start_is_one_start_and_a_set_start_draws_from_the_set():
    from minilink.core.sets import BoxSet

    plant = pendulum()
    deterministic = PlanningProblem(
        plant,
        x_start=[0.3, -0.1],
        cost=quadratic(plant),
        tf=5.0,
        infeasible_cost=10.0,
        U=plant.input_box,
    )
    assert not deterministic.is_stochastic and deterministic.criterion == "expectation"
    np.testing.assert_allclose(deterministic.sample_x0(0, n=3), [[0.3, -0.1]] * 3)
    assert deterministic.sample_params(0) == {}
    assert deterministic.nominal().x_start.tolist() == [0.3, -0.1]

    # A set as the start is the uniform law on it, the set its X0
    box = BoxSet([0.0, -1.0], [1.0, 1.0])
    from_a_set = deterministic.replace(x_start=box)
    assert from_a_set.is_stochastic and from_a_set.X0 is box
    starts = from_a_set.sample_x0(1, n=50)
    assert np.all([box.contains(x) for x in starts])
    np.testing.assert_allclose(from_a_set.x_start, [0.5, 0.0])


# --- the shared scoring contract ---


def test_evaluate_trajectory_applies_the_discount_rate():
    from minilink.core.trajectory import Trajectory

    class Unit(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return 1.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    t = np.linspace(0.0, 1.0, 101)
    traj = Trajectory(t=t, x=np.zeros((2, t.size)), u=np.zeros((1, t.size)))
    J = traj.with_cost(Unit(), discount_rate=1.0).signals["cost"][0, -1]
    np.testing.assert_allclose(J, 1.0 - np.exp(-1.0), atol=1e-4)  # int_0^1 e^{-t} dt


def test_evaluate_trajectory_integrates_to_tf_and_prices_by_the_switch():
    from minilink.core.trajectory import Trajectory
    from minilink.planning import evaluate_trajectory

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

    box = dict(X=plant.state.box, U=plant.input_box)
    finite = PlanningProblem(plant, cost=Unit(), tf=1.0, infeasible_cost=50.0, **box)
    report = evaluate_trajectory(finite, inside)
    assert not report.failed[0] and np.isclose(report.J[0], 1.0 + 7.0)  # ∫ g + h
    report = evaluate_trajectory(finite, exits)
    assert report.failed[0] and np.isclose(report.J[0], 1.0 + 7.0 + 50.0)  # + price
    assert np.isclose(report.feasibility_margin[0], -1.0)

    # The switch off: the plain cost, the failure reported beside it
    unpriced = PlanningProblem(plant, cost=Unit(), tf=np.inf, **box)
    report = evaluate_trajectory(unpriced, exits)
    assert report.failed[0] and np.isclose(
        report.J[0], 1.0
    )  # no h on an infinite horizon
    free = PlanningProblem(plant, cost=Unit(), tf=np.inf, U=plant.input_box)
    assert not evaluate_trajectory(free, exits).failed[0]  # X unconstrained: no failure


def test_every_backend_scores_the_constraint_set_on_its_parameters_and_time():
    """A parametric, time-varying X is read on its own parameters at each sample time."""
    from minilink.control import StateFeedback
    from minilink.core.backends import array_module
    from minilink.core.sets import Set
    from minilink.planning import evaluate

    class Band(Set):
        """|z_i| <= r - 0.2 t: a band narrowing from its half-width r."""

        def __init__(self, r):
            self.params = {"r": r}

        def margin(self, z, t=0.0, params=None):
            r0 = (self.params if params is None else params)["r"]
            xp = array_module(z)

            r = r0 - 0.2 * t

            return xp.concatenate([z + r, r - z])

    plant = pendulum()
    ctl = StateFeedback(K=[[-20.0, 0.0]])  # u = 20 theta: the loop diverges
    runs = [("numpy", "rk4"), ("numpy", "scipy")]
    runs += [("jax", "rk4")] if jax_available() else []
    for r, leaves in ((0.5, True), (3.0, False)):
        problem = PlanningProblem(
            plant,
            x_start=[0.1, 0.0],
            cost=quadratic(plant),
            tf=1.0,
            X=Band(r),
            infeasible_cost=1.0,
        )
        reports = {
            run: evaluate(problem, ctl, dt=0.01, backend=run[0], solver=run[1])
            for run in runs
        }
        assert all(bool(report.failed[0]) == leaves for report in reports.values())
        reference = reports["numpy", "rk4"].J
        if ("jax", "rk4") in reports:
            np.testing.assert_allclose(reports["jax", "rk4"].J, reference, rtol=1e-6)
        # the adaptive integral of the same loop, within its tolerance
        np.testing.assert_allclose(reports["numpy", "scipy"].J, reference, rtol=1e-3)


@pytest.mark.optional
@pytest.mark.jax
def test_the_backends_share_the_score_on_identical_starts():
    pytest.importorskip("jax")
    from minilink.control import NeuralPolicy
    from minilink.core.distributions import Particles
    from minilink.planning import evaluate

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])

    class Discounted(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return x[0] ** 2 + 0.01 * u[0] ** 2

        def h(self, x, t=0.0, params=None):
            return 3.0

    ctl = NeuralPolicy(plant, hidden=(8,), seed=5)  # bounded by construction
    one_start = Particles([[2.5, 0.0]])
    for kwargs in ({"tf": np.inf}, {"tf": 1.0, "infeasible_cost": 20.0}):
        problem = PlanningProblem(
            plant,
            cost=Discounted(),
            x_start=one_start,
            discount_rate=0.2,
            **kwargs,
        )
        settings = dict(dt=0.05, n_trials=3, episode_length=1.0)
        on_jax = evaluate(problem, ctl, backend="jax", **settings)
        on_numpy = evaluate(problem, ctl, backend="numpy", **settings)
        adaptive = evaluate(problem, ctl, solver="scipy", **settings)
        np.testing.assert_allclose(on_jax.J, on_numpy.J, rtol=1e-6)
        np.testing.assert_allclose(adaptive.J, on_numpy.J, rtol=1e-2)
        assert float(on_jax.worst) == float(np.max(on_jax.J))


@pytest.mark.optional
@pytest.mark.jax
def test_the_law_runs_beyond_the_port_bounds_and_fails_the_declared_U():
    """Port bounds are information: nothing clips a law that exceeds them; a declared U fails it."""
    pytest.importorskip("jax")
    from minilink.control import StateFeedback
    from minilink.core.distributions import Particles
    from minilink.planning import evaluate

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-0.5])
    plant.inputs["u"].upper_bound = np.array([0.5])
    ctl = StateFeedback(K=[[30.0, 8.0]])  # |u| = 15 N m at the start

    class Effort(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return u[0] ** 2

        def h(self, x, t=0.0, params=None):
            return 0.0

    problem = PlanningProblem(
        plant,
        cost=Effort(),
        tf=np.inf,
        x_start=Particles([[0.5, 0.0]]),
        U=plant.input_box,
    )
    settings = dict(dt=0.01, n_trials=1, episode_length=1.0)
    on_jax = evaluate(problem, ctl, backend="jax", **settings)
    on_numpy = evaluate(problem, ctl, backend="numpy", **settings)
    saturated_bound = 0.5**2 * 1.0
    assert on_numpy.J[0] > 10 * saturated_bound and on_numpy.failed[0]
    np.testing.assert_allclose(on_jax.J, on_numpy.J, rtol=1e-6)


def test_an_exiting_run_scores_alike_on_the_step_and_the_adaptive_solver():
    from minilink.control import StateFeedback
    from minilink.planning import evaluate

    plant = pendulum()
    problem = PlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=np.inf,
        X=plant.state.box,
        x_start=Particles([[6.0, 5.0]]),  # leaves θ ≤ 2π within 0.1 s
    )
    ctl = StateFeedback(K=[[0.0, 0.0]])
    settings = dict(dt=0.01, n_trials=1, episode_length=1.0, backend="numpy")
    step = evaluate(problem, ctl, **settings)
    adaptive = evaluate(problem, ctl, solver="scipy", **settings)
    assert step.failed[0] and adaptive.failed[0]
    assert np.isfinite(step.J[0])  # the switch off: the plain cost, failure reported
    np.testing.assert_allclose(adaptive.J, step.J, rtol=1e-3)


def test_a_controller_with_internal_state_scores_like_its_static_twin():
    """The controller's state is stacked in the loop; the score reads the plant's own run."""
    from minilink import PID, SingleMass
    from minilink.control import StateFeedback
    from minilink.core.distributions import Particles
    from minilink.planning import evaluate

    mass = SingleMass()
    problem = PlanningProblem(
        mass,
        cost=quadratic(mass),
        tf=np.inf,
        x_start=Particles([[0.5, 0.0]]),
    )
    settings = dict(dt=0.01, n_trials=1, episode_length=2.0, backend="numpy")
    J_pid = evaluate(problem, PID(Kp=4.0), **settings).J  # with filter states
    J_static = evaluate(problem, StateFeedback(K=[[4.0, 0.0]]), **settings).J
    np.testing.assert_allclose(J_pid, J_static, rtol=0.05)


@pytest.mark.optional
@pytest.mark.jax
def test_randomized_parameters_reach_the_dynamics_on_both_backends():
    pytest.importorskip("jax")
    from minilink.control import NeuralPolicy
    from minilink.core.distributions import Particles
    from minilink.planning import evaluate

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    ctl = NeuralPolicy(plant, hidden=(8,), seed=5)
    start = Particles([[0.5, 0.0]])
    task = dict(cost=quadratic(plant), tf=np.inf, x_start=start)
    heavy = PlanningProblem(plant, params={"plant": {"m": Particles([[3.0]])}}, **task)
    nominal = PlanningProblem(plant, **task)
    settings = dict(dt=0.05, n_trials=1, episode_length=1.0)
    J_heavy = {}
    for backend in ("jax", "numpy"):
        J_heavy[backend] = evaluate(heavy, ctl, backend=backend, **settings).J[0]
        J_nominal = evaluate(nominal, ctl, backend=backend, **settings).J[0]
        assert not np.isclose(J_heavy[backend], J_nominal)
    np.testing.assert_allclose(J_heavy["numpy"], J_heavy["jax"], rtol=1e-6)
    assert plant.params["m"] == 1.0  # the draw never touched the nominal plant


def test_the_default_backend_scores_a_lookup_table_law_on_numpy():
    """A DP law runs on NumPy only: "auto" falls back, an explicit JAX backend refuses."""
    from minilink.control import StateFeedback
    from minilink.planning import ProblemEvaluator

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    problem = PlanningProblem(
        plant,
        x_start=[0.5, 0.0],
        cost=quadratic(plant),
        tf=1.0,
        X=plant.state.box,
        U=plant.input_box,
        infeasible_cost=500.0,
    )
    vi = DynamicProgrammingPlanner(
        problem, x_grid=(11, 11), u_grid=(3,), dt=0.1, alpha=0.95, tol=0.5
    ).solve()

    auto = ProblemEvaluator(problem, vi, 0.1)
    assert auto.backend == "numpy"
    on_numpy = ProblemEvaluator(problem, vi, 0.1, backend="numpy")
    np.testing.assert_array_equal(auto.evaluate().J, on_numpy.evaluate().J)
    if not jax_available():
        return

    with pytest.raises(RuntimeError, match="JAX"):
        ProblemEvaluator(problem, vi, 0.1, backend="jax")

    # a law that traces keeps the JAX backend
    law = StateFeedback(K=[[5.0, 1.0]])
    assert ProblemEvaluator(problem, law, 0.1).backend == "jax"


@pytest.mark.optional
@pytest.mark.jax
def test_the_default_backend_falls_back_on_a_numpy_only_set_or_cost():
    """The JAX run traces X and the cost as well: a NumPy-only one keeps "auto" on NumPy."""
    from minilink.control import StateFeedback
    from minilink.core.backends import array_module
    from minilink.core.sets import Set
    from minilink.planning import ProblemEvaluator

    class NumpyBand(Set):
        """|z_i| <= r, the margin written with NumPy only."""

        params = {"r": 0.8}

        def margin(self, z, t=0.0, params=None):
            r = (self.params if params is None else params)["r"]

            return np.concatenate([np.asarray(z) + r, r - np.asarray(z)])

    class Band(Set):
        """The same band, written for both backends."""

        params = {"r": 0.8}

        def margin(self, z, t=0.0, params=None):
            r = (self.params if params is None else params)["r"]
            xp = array_module(z)

            return xp.concatenate([z + r, r - z])

    class NumpyEnergy(CostFunction):
        """g = x'x + u'u, written with NumPy only."""

        def g(self, x, u, t=0.0, params=None):
            return float(np.dot(x, x) + np.dot(u, u))

        def h(self, x, t=0.0, params=None):
            return 0.0

    plant = pendulum()
    law = StateFeedback(K=[[5.0, 1.0]])
    task = dict(
        tf=1.0,
        x_start=Uniform([-0.5, -0.5], [0.5, 0.5]),
        infeasible_cost=5.0,
    )
    for X, cost in ((NumpyBand(), quadratic(plant)), (plant.state.box, NumpyEnergy())):
        problem = PlanningProblem(plant, cost=cost, X=X, **task)
        auto = ProblemEvaluator(problem, law, 0.1)
        assert auto.backend == "numpy"
        numpy = ProblemEvaluator(problem, law, 0.1, backend="numpy")
        np.testing.assert_array_equal(
            auto.evaluate(n_trials=3).J, numpy.evaluate(n_trials=3).J
        )

    # the same band written with xp traces, and keeps the JAX backend
    problem = PlanningProblem(plant, cost=quadratic(plant), X=Band(), **task)
    assert ProblemEvaluator(problem, law, 0.1).backend == "jax"


def test_deterministic_planner_warns_on_a_stochastic_problem():
    from minilink.planning.trajectory_optimization.planner import (
        TrajectoryOptimizationPlanner,
    )

    plant = pendulum()
    problem = PlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=2.0,
        x_start=Uniform([-0.1, -0.1], [0.1, 0.1]),
        U=plant.input_box,
    )
    with pytest.warns(
        UserWarning, match="deterministic planner: it reads the start set X0"
    ):
        TrajectoryOptimizationPlanner(
            problem, n_steps=5, transcription="direct_collocation"
        )


def test_nominal_keeps_the_mean_start_and_drops_the_support_and_x_start():
    from minilink.core.sets import SingletonSet

    plant = pendulum()
    problem = PlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=2.0,
        x_start=Uniform([-0.1, -0.1], [0.1, 0.1]),
        U=plant.input_box,
    )
    assert problem.X0.contains(np.array([0.08, -0.08]))  # the support
    nominal = problem.nominal()
    np.testing.assert_allclose(nominal.x_start, [0.0, 0.0])  # the mean
    assert isinstance(nominal.X0, SingletonSet)
    np.testing.assert_allclose(nominal.X0.point, [0.0, 0.0])


def test_gymnasium_view_of_a_stochastic_problem():
    pytest.importorskip("gymnasium")
    from minilink.core.distributions import Particles
    from minilink.interfaces.gymnasium import Sys2Gym

    plant = pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    problem = PlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=1.0,
        X=plant.state.box,
        x_start=Particles([[0.3, 0.0], [-0.3, 0.0]]),
        infeasible_cost=50.0,
        U=plant.input_box,
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


# --- the Evaluation record, and open-loop policies in the loop ---


def test_the_evaluation_of_one_trajectory_prints_a_single_cost():
    from minilink.control import StateFeedback
    from minilink.planning import evaluate, evaluate_trajectory

    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), tf=1.0)
    t = np.linspace(0.0, 1.0, 6)
    traj = Trajectory(t=t, x=np.vstack([0.1 * t, np.zeros(6)]), u=np.zeros((1, 6)))
    single = evaluate_trajectory(problem, traj)
    assert single.n_trials == 1 and single.trajectories[0] is traj
    assert str(single).startswith("J = ") and "runs" not in str(single)

    # a deterministic problem is one run for the evaluation as well
    law = StateFeedback(K=[[1.0, 0.5]], xbar=[0.0, 0.0])
    report = evaluate(problem, law, dt=0.1, backend="numpy")
    assert report.n_trials == 1 and np.all(report.x0[0] == problem.x_start)


def test_an_open_loop_source_scores_like_its_first_order_hold_rollout():
    from minilink.blocks import TrajectorySource
    from minilink.planning import evaluate, evaluate_trajectory

    plant = pendulum()
    problem = PlanningProblem(plant, x_start=[0.3, 0.0], cost=quadratic(plant), tf=1.0)
    dt, n_steps = 0.1, 10
    t = dt * np.arange(n_steps + 1)
    u = 2.0 * np.sin(3.0 * t)[None, :]
    source = TrajectorySource(t, u)
    assert int(source.m) == 0  # no input port: an open-loop policy

    # The source interpolates linearly: the plant's first-order-hold rollout
    evaluator = plant.compile(backend="numpy", verbose=False)
    xs = evaluator.rollout(problem.x_start, u.T, 0.0, dt, hold="foh")
    J_hand = evaluate_trajectory(problem, Trajectory(t=t, x=xs.T, u=u)).J

    runs = [("numpy", "rk4"), ("numpy", "scipy")]
    runs += [("jax", "rk4")] if jax_available() else []
    reports = {
        run: evaluate(problem, source, dt=dt, backend=run[0], solver=run[1])
        for run in runs
    }
    np.testing.assert_allclose(reports["numpy", "rk4"].J, J_hand, rtol=1e-10)
    if ("jax", "rk4") in reports:
        np.testing.assert_allclose(reports["jax", "rk4"].J, J_hand, rtol=1e-8)
    np.testing.assert_allclose(reports["numpy", "scipy"].J, J_hand, rtol=1e-3)

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


def test_a_time_varying_law_scores_alike_on_every_backend():
    pytest.importorskip("jax")
    from minilink.control.state import TimeVaryingFeedback
    from minilink.planning import evaluate

    t = np.linspace(0.0, 1.0, 5)
    K = np.stack([np.array([[1.0 + 10.0 * k, 0.5]]) for k in range(5)])
    ctl = TimeVaryingFeedback(t, K)
    problem = PlanningProblem(
        pendulum(),
        cost=QuadraticCost.from_system(pendulum()),
        x_start=Particles([[0.5, 0.0]]),
        tf=1.0,
    )
    settings = dict(dt=0.01, n_trials=1)
    on_jax = evaluate(problem, ctl, backend="jax", **settings)
    on_numpy = evaluate(problem, ctl, backend="numpy", **settings)
    adaptive = evaluate(problem, ctl, solver="scipy", **settings)
    np.testing.assert_allclose(on_jax.J, on_numpy.J, rtol=1e-6)
    np.testing.assert_allclose(adaptive.J, on_numpy.J, rtol=1e-2)


def test_a_disturbance_on_the_command_port_is_refused():
    with pytest.raises(ValueError, match="command port of sys"):
        PlanningProblem(
            pendulum(),
            cost=QuadraticCost.from_system(pendulum()),
            x_start=Particles([[0.5, 0.0]]),
            disturbances={"u": WhiteNoise(1, psd=0.1)},
        )
