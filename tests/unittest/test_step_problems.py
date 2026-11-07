"""The step problem and the compiled step face."""

import numpy as np
import pytest

from minilink import Pendulum, PlanningProblem, QuadraticCost
from minilink.core.compile.evaluators.stepping import step_body
from minilink.core.costs import SampledCost, StepCost
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.core.discrete.step_system import StepSystem
from minilink.planning.step_face import StepFace
from minilink.planning.step_problems import StepPlanningProblem


def swing_problem(**fields):
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant, x_bar=np.array([np.pi, 0.0]))
    fields.setdefault("horizon", 2.0)
    return PlanningProblem(plant, x0=[0.3, 0.0], cost=cost, U=plant.input_box, **fields)


@pytest.mark.parametrize(("solver", "substeps"), [("euler", 1), ("rk4", 1), ("rk4", 3)])
def test_the_augmented_step_is_the_step_model_and_its_cost_g_dt_under_euler(
    solver, substeps
):
    problem = swing_problem(discount_rate=0.4, t0=1.5)
    face = problem.discretize(0.05, solver=solver, substeps=substeps).compile()
    plant = problem.sys
    rng = np.random.default_rng(3)
    for k in range(20):
        x, u = rng.uniform(-3.0, 3.0, 2), rng.uniform(-5.0, 5.0, 1)
        x_next, c = face.transition(x, u, k)
        t_k = 1.5 + k * 0.05

        # The state rows are the step model's, to the bit
        f = lambda x, u, t: plant.f(x, u, t, None)  # noqa: E731
        np.testing.assert_array_equal(
            x_next, step_body(f, x, u, u, t_k, 0.05, solver, substeps)
        )

        # Under one Euler step, c_k = g(x_k, u_k, t_k) dt to the last bit
        if solver == "euler":
            assert c == problem.cost.g(x, u, t_k, None) * 0.05

    assert face.n_steps == 40 and face.gamma == np.exp(-0.4 * 0.05)


@pytest.mark.parametrize("solver", ["euler", "rk4"])
def test_the_discretized_problem_compiles_to_the_same_face(solver):
    problem = swing_problem(discount_rate=0.2)
    face = StepFace(problem, dt=0.05, solver=solver, substeps=1)
    sampled = problem.discretize(0.05, solver=solver)
    assert isinstance(sampled, StepPlanningProblem)
    assert isinstance(sampled.sys, SampledSystem) and isinstance(
        sampled.cost, SampledCost
    )
    other = sampled.compile()
    assert (other.n_steps, other.gamma, other.dt) == (face.n_steps, face.gamma, face.dt)

    rng = np.random.default_rng(5)
    for k in range(10):
        x, u = rng.uniform(-3.0, 3.0, 2), rng.uniform(-5.0, 5.0, 1)
        a, b = face.transition(x, u, k), other.transition(x, u, k)
        np.testing.assert_array_equal(a[0], b[0])
        assert a[1] == b[1]

        # The SampledCost called directly integrates the same period
        assert sampled.cost.c(x, u, k) == a[1]

    # One rollout, one J: Σ γᵏ c_k + γᴺ h on both faces
    law = lambda x, t: np.array([-2.0 * x[0] - x[1]])  # noqa: E731
    assert face.run(law)[3] == other.run(law)[3]


def test_the_stationary_stage_cost_is_the_discounted_integral():
    # c_k = ∫₀^dt e^{−ρτ} g dτ: with g = 1, (1 − e^{−ρ dt}) / ρ
    class Unit(QuadraticCost):
        def g(self, x, u, t=0.0, params=None):
            return 1.0 + 0.0 * x[0]

    problem = swing_problem(discount_rate=0.7)
    problem.cost = Unit.from_system(problem.sys)
    face = problem.discretize(0.1, solver="rk4", substeps=4).compile()
    _, c = face.transition(np.array([0.1, 0.0]), np.zeros(1), 3)
    assert c == pytest.approx((1.0 - np.exp(-0.7 * 0.1)) / 0.7, rel=1e-10)


class Moves(StepCost):
    """``c = 1`` per tick away from the goal cell, ``0`` on it."""

    def __init__(self, goal):
        self.params = {"goal": np.asarray(goal, dtype=float)}

    def c(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        return float(np.any(np.abs(x - params["goal"]) > 0.5))

    def h(self, x, k=0, params=None):
        return 0.0


class Walk(StepSystem):
    """A walker on the cells 0 to 5 of a line: ``x_{k+1} = x_k + u_k``, clipped to the line."""

    def __init__(self):
        super().__init__(state_dim=1, input_dim=1, output_dim=1)
        self.state.lower_bound[:] = [0.0]
        self.state.upper_bound[:] = [5.0]
        self.inputs["u"].lower_bound[:] = [-1.0]
        self.inputs["u"].upper_bound[:] = [1.0]

    def step(self, x, u, k=0, params=None):
        return np.clip(x + u, 0.0, 5.0)


def test_a_step_problem_has_its_own_horizon_discount_and_tick_offset():
    problem = StepPlanningProblem(
        Walk(), n_steps=6, cost=Moves([4.0]), gamma=0.9, x0=[1.0], t0=2
    )
    face = problem.compile()
    xs, us, cs, J = face.run(lambda x, t: np.ones(1))
    np.testing.assert_array_equal(xs[:, 0], [1, 2, 3, 4, 5, 5, 5])
    assert J == pytest.approx(1 + 0.9 + 0.81 + 0.0 + 0.9**4 + 0.9**5)
    assert face.time_of(3) == 5  # the tick offset, an integer
    assert set(problem.params) >= {"plant", "cost", "x0", "t0", "gamma"}

    with pytest.raises(TypeError, match="SampledCost"):
        StepPlanningProblem(
            Walk(), n_steps=3, cost=QuadraticCost.from_system(Pendulum())
        )
    with pytest.raises(ValueError, match="tick offset"):
        StepPlanningProblem(Walk(), n_steps=3, cost=Moves([4.0]), t0=0.5)


def test_the_state_only_step_is_the_transitions_state_on_both_faces():
    problem = swing_problem(t0=2.0)
    face = StepFace(problem, dt=0.05, solver="rk4", substeps=1)
    sampled = problem.discretize(0.05, solver="rk4").compile()
    x, u = np.array([0.4, -0.3]), np.array([1.0])
    for t in (2.0, 2.35):
        np.testing.assert_array_equal(
            face.step_at(x, u, t), face.transition_at(x, u, t)[0]
        )
        np.testing.assert_array_equal(sampled.step_at(x, u, t), face.step_at(x, u, t))

    # A problem without a cost still steps: a grid needs only its successors
    free = PlanningProblem(problem.sys, x0=[0.0, 0.0])
    assert StepFace(free, dt=0.1, solver="rk4", substeps=1).step_at(
        x, u, 0.0
    ).shape == (2,)


def grid_world(**fields):
    """A walker on a 7 × 7 grid of cells, a step to any of its 8 neighbours or none, one unit a move off the goal."""

    class Walker(StepSystem):
        def __init__(self):
            super().__init__(state_dim=2, input_dim=2, output_dim=2)
            self.state.lower_bound[:] = [0.0, 0.0]
            self.state.upper_bound[:] = [6.0, 6.0]
            self.inputs["u"].lower_bound[:] = [-1.0, -1.0]
            self.inputs["u"].upper_bound[:] = [1.0, 1.0]

        def step(self, x, u, k=0, params=None):
            return np.clip(x + u, 0.0, 6.0)

    walker = Walker()
    fields.setdefault("x0", [0.0, 0.0])
    return StepPlanningProblem(
        walker,
        cost=Moves([5.0, 3.0]),
        X=walker.state.box,
        U=walker.input_box,
        penalize_failure=True,
        failure_cost=100.0,
        **fields,
    )


def test_a_grid_world_with_a_law_as_its_start_realizes_from_its_own_tree():
    from minilink import Uniform
    from minilink.planning.problems import stream_of

    start = Uniform([0.0, 0.0], [2.0, 2.0])
    goal = Uniform([4.0, 2.0], [6.0, 4.0])
    problem = grid_world(x0=start, distributions={"cost": {"goal": goal}}, t0=4)
    assert problem.is_stochastic

    # The tree holds the law's numbers: the start's support is the law's own box
    assert problem.params["X0"] is start.params
    roles = {"plant", "cost", "X", "U", "X0", "x0", "t0", "gamma"}
    assert set(problem.params) == roles

    # realize(key, n) draws the start and the goal into the problem's own tree
    tree = problem.realize(7, n=5)
    assert set(tree) == roles
    np.testing.assert_array_equal(tree["x0"], start.sample(stream_of(7, "x0"), 5))
    goals = tree["cost"]["goal"]
    assert goals.shape == (5, 2)
    assert np.all((goals >= [4.0, 2.0]) & (goals <= [6.0, 4.0]))
    assert tree["t0"] == 4 and tree["gamma"] == 1.0
    np.testing.assert_array_equal(problem.realize(7, n=5)["x0"], tree["x0"])

    # The start is drawn from the numbers the tree holds: an edit of its box moves it
    problem.params["X0"]["lower"][:] = [5.0, 5.0]
    problem.params["X0"]["upper"][:] = [6.0, 6.0]
    assert np.all(problem.realize(7, n=5)["x0"] >= 5.0)

    # No key: every law at its mean; one draw of the laws at its path
    nominal = problem.realize(None, n=2)
    np.testing.assert_array_equal(nominal["x0"], [[5.5, 5.5], [5.5, 5.5]])
    np.testing.assert_array_equal(nominal["cost"]["goal"], [[5.0, 3.0], [5.0, 3.0]])
    assert problem.sample_params(3)["cost"]["goal"].shape == (2,)

    # A deterministic grid world has one realization, its own start
    np.testing.assert_array_equal(grid_world().realize()["x0"], [[0.0, 0.0]])
    with pytest.raises(ValueError, match="one realization"):
        grid_world().realize(0, n=2)
    with pytest.raises(ValueError, match="names no leaf"):
        grid_world(distributions={"cost": {"target": goal}})


class Late(StepCost):
    """``c = |x − 4| (1 + k + s) + u²``, ``h = 10 |x − 4| (1 + k + s)``: a cost that reads the tick, shifted by ``s`` ticks."""

    def __init__(self, shift=0):
        self.params = {"goal": 4.0}
        self.shift = shift

    def c(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        return abs(x[0] - params["goal"]) * (1 + k + self.shift) + u[0] ** 2

    def h(self, x, k=0, params=None):
        params = self.params if params is None else params
        return 10.0 * abs(x[0] - params["goal"]) * (1 + k + self.shift)


def late_and_shifted(n_steps):
    """The walk started at ``k₀ = 3`` on :class:`Late`, and from 0 on the cost shifted by 3 ticks."""
    walk = Walk()
    fields = dict(
        n_steps=n_steps, gamma=0.9, x0=[1.0], X=walk.state.box, U=walk.input_box
    )
    late = StepPlanningProblem(walk, cost=Late(), t0=3, **fields)
    shifted = StepPlanningProblem(walk, cost=Late(shift=3), t0=0, **fields)

    return late, shifted


@pytest.mark.parametrize("n_steps", [4, None])
def test_a_step_problem_started_at_k0_scores_as_from_0_with_the_cost_shifted(n_steps):
    late, shifted = late_and_shifted(n_steps)
    assert late.params["t0"] == 3 and late.realize()["t0"] == 3

    # J = Σ γᵏ c(x_k, u_k, k₀ + k) + γᴺ h(x_N, k₀ + N), the discount counted from the start
    law = lambda x, t: np.array([1.0 if x[0] < 3.5 else 0.0])  # noqa: E731
    xs, us, cs, J = late.compile().run(law, n_steps=4)
    xs_0, us_0, cs_0, J_0 = shifted.compile().run(law, n_steps=4)
    np.testing.assert_array_equal(xs, xs_0)
    np.testing.assert_array_equal(cs, cs_0)
    assert J == J_0 == 13.0 + 0.9 * 11.0 + 0.81 * 7.0

    # The same cost bound: the stage and terminal costs read at the tick offset
    assert late.compile().cost_bound == shifted.compile().cost_bound


def test_a_grid_world_solves_by_value_iteration_as_a_step_problem():
    from minilink.planning import DynamicProgrammingPlanner

    vi = DynamicProgrammingPlanner(
        grid_world(), x_grid=(7, 7), u_grid=(3, 3), tol=1e-9, max_iterations=50
    )
    vi.solve()

    # J(x) = the moves to the goal, diagonal moves allowed: the Chebyshev distance
    J = vi.result.J.reshape(7, 7)
    cells = np.arange(7.0)
    distance = np.maximum.outer(np.abs(cells - 5.0), np.abs(cells - 3.0))
    np.testing.assert_array_equal(J, distance)
    assert vi.grid.dt == 1.0 and vi.gamma == 1.0


class TimedTerminal(QuadraticCost):
    """``h(x, t) = 1 + t``: a terminal cost that reads the time it is charged at."""

    def h(self, x, t=0.0, params=None):
        return 1.0 + t + 0.0 * x[0]


def test_the_discretized_face_charges_the_terminal_cost_at_the_horizons_time():
    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=TimedTerminal.from_system(plant),
        x0=[0.3, 0.0],
        horizon=2.0,
        t0=0.5,
    )
    face = StepFace(problem, dt=0.05, solver="rk4", substeps=1)
    sampled = problem.discretize(0.05).compile()
    x = np.array([0.1, 0.2])
    assert face.terminal(x) == sampled.terminal(x) == 1.0 + 2.5
    law = lambda x, t: np.array([-x[0]])  # noqa: E731
    assert face.run(law)[3] == sampled.run(law)[3]


def test_value_iteration_on_a_problem_and_on_its_discretization_agree():
    from minilink.planning import DynamicProgrammingPlanner

    problem = swing_problem(horizon=3.0, discount_rate=0.5)
    kwargs = dict(x_grid=(21, 21), u_grid=(5,), solver="rk4")
    vi = DynamicProgrammingPlanner(problem, dt=0.1, **kwargs)
    vd = DynamicProgrammingPlanner(problem.discretize(0.1, solver="rk4"), **kwargs)
    assert vi.out_of_bound_cost == vd.out_of_bound_cost
    vi.solve()
    vd.solve()
    np.testing.assert_array_equal(vi.result.J, vd.result.J)
    np.testing.assert_array_equal(vi.result.pi, vd.result.pi)


def test_a_step_problems_grid_takes_no_period_and_a_source_does_not_discretize():
    from minilink import WhiteNoise
    from minilink.catalog import NoisyPendulum
    from minilink.planning import DynamicProgrammingPlanner

    with pytest.raises(ValueError, match="own clock"):
        DynamicProgrammingPlanner(
            swing_problem().discretize(0.1), x_grid=(5, 5), u_grid=(3,), dt=0.1
        )
    plant = NoisyPendulum()
    noisy = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x0=[0.3, 0.0],
        horizon=1.0,
        disturbances={"d": WhiteNoise(1, psd=0.5, sample_period=0.01, seed=1)},
    )
    with pytest.raises(ValueError, match="disturbance sources"):
        noisy.discretize(0.1)


def test_the_policy_evaluator_scores_value_iterations_policy_on_the_face():
    from minilink.planning import DynamicProgrammingPlanner
    from minilink.planning.dynamic_programming.policy_eval import PolicyEvaluator

    problem = swing_problem(horizon=np.inf, discount_rate=0.5)
    vi = DynamicProgrammingPlanner(
        problem,
        x_grid=(21, 21),
        u_grid=(5,),
        dt=0.1,
        solver="rk4",
        tol=1e-10,
        clean_infeasible=False,
    )
    vi.solve()
    pe = PolicyEvaluator(
        problem,
        grid=vi.grid,
        policy=vi.controller,
        **vi.params["method"],
        tol=vi.tol,
        max_iterations=vi.max_iterations,
        interpolation=vi.interpolation,
    )
    np.testing.assert_allclose(pe.solve(), vi.result.J, atol=1e-6)


def test_the_design_point_of_a_random_plant_parameter_is_its_mean():
    from minilink import Uniform

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x0=[0.3, 0.0],
        horizon=1.0,
        distributions={"plant": {"m": Uniform(1.5, 2.5)}},
    )
    assert problem.sample_params(None)["plant"]["m"] == 2.0
    face = problem.discretize(0.1).compile()
    assert face.params["plant"]["m"] == 2.0 == problem.nominal().sys.params["m"]


def episode_return(env, law):
    """One episode of ``env`` under ``u = law(x)``: its discounted return ``Σ γᵏ r_k`` and its length."""
    x, t = env.reset(0), env.t0
    total, k = 0.0, 0
    while True:
        x, t, r, terminated, truncated = env.step(x, t, law(x), None)
        total += env.face.gamma**k * float(r)
        k += 1
        if terminated or truncated:
            return total, k


def test_the_rollout_environment_steps_a_native_step_problem_on_its_face():
    from minilink.planning.environment import RolloutEnvironment

    # The face's N, γ and cost bound; a native problem ticks, its time the tick k₀ + k
    late, shifted = late_and_shifted(4)
    env = RolloutEnvironment(late, backend="numpy")
    face = late.compile()
    assert (env.dt, env.t0, env.n_steps_per_episode) == (1.0, 3.0, 4)
    assert env.finite_horizon and env.reward == "stage"
    assert env.failure_cost == face.failure_cost_over(4, 0.9) == face.cost_bound

    # One step is the face's transition, rewarded −c_k
    x_next, t_next, r, terminated, truncated = env.step(
        np.array([1.0]), 3.0, np.array([1.0]), None
    )
    assert (
        x_next[0] == 2.0
        and t_next == 4.0
        and r == -face.transition_at(np.array([1.0]), np.array([1.0]), 3.0)[1]
    )
    assert not terminated and not truncated

    # The return is −J of the face's run, and the same from k₀ as from 0 on the shifted cost
    law = lambda x: np.array([1.0 if x[0] < 3.5 else 0.0])  # noqa: E731
    J = face.run(lambda x, t: law(x))[3]
    assert episode_return(env, law) == (-J, 4)
    shifted_env = RolloutEnvironment(shifted, backend="numpy")
    assert episode_return(shifted_env, law) == (-J, 4)

    # A step problem ticks on its own clock: no control period is given
    with pytest.raises(ValueError, match="own clock"):
        RolloutEnvironment(late, dt=0.1, backend="numpy")


def test_the_rollout_environment_steps_a_discretized_problem_as_its_origin():
    from minilink.planning.environment import RolloutEnvironment

    problem = swing_problem(horizon=1.0, discount_rate=0.3, X=Pendulum().state.box)
    origin = RolloutEnvironment(problem, dt=0.1, reward="stage", backend="numpy")
    sampled = RolloutEnvironment(problem.discretize(0.1), backend="numpy")
    assert sampled.reward == "stage" and sampled.dt == 0.1
    assert sampled.n_steps_per_episode == origin.n_steps_per_episode == 10
    assert sampled.failure_cost == origin.failure_cost

    law = lambda x: np.array([-2.0 * x[0] - x[1]])  # noqa: E731
    assert episode_return(sampled, law) == episode_return(origin, law)


def test_a_problem_compiles_a_controllers_evaluator_only():
    # One verb, one return type: the step face is the discretization's
    problem = swing_problem()

    with pytest.raises(ValueError, match="discretize"):
        problem.compile()
    with pytest.raises(ValueError, match="discretize"):
        problem.compile(dt=0.05)


def test_a_discretized_random_plant_steps_at_its_mean():
    from minilink import Uniform

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x0=[0.3, 0.0],
        horizon=1.0,
        distributions={"plant": {"m": Uniform(1.5, 2.5)}},
    )

    assert problem.discretize(0.1).compile().params["plant"]["m"] == 2.0
