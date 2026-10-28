"""The step problem and the compiled step face (R141, R142, R154)."""

import numpy as np
import pytest

from minilink import Pendulum, PlanningProblem, QuadraticCost
from minilink.core.compile.evaluators.stepping import step_body
from minilink.core.costs import SampledCost, StepCost
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.planning.step_problems import StepPlanningProblem


def swing_problem(**fields):
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant, xbar=np.array([np.pi, 0.0]))
    fields.setdefault("tf", 2.0)
    return PlanningProblem(
        plant, x_start=[0.3, 0.0], cost=cost, U=plant.input_box, **fields
    )


@pytest.mark.parametrize(("solver", "substeps"), [("euler", 1), ("rk4", 1), ("rk4", 3)])
def test_the_augmented_step_is_the_step_model_and_its_cost_g_dt_under_euler(
    solver, substeps
):
    problem = swing_problem(discount_rate=0.4, t_start=1.5)
    face = problem.compile(dt=0.05, solver=solver, substeps=substeps)
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

    assert face.N == 40 and face.gamma == np.exp(-0.4 * 0.05)


@pytest.mark.parametrize("solver", ["euler", "rk4"])
def test_the_discretized_problem_compiles_to_the_same_face(solver):
    problem = swing_problem(discount_rate=0.2)
    face = problem.compile(dt=0.05, solver=solver)
    sampled = problem.discretize(0.05, solver=solver)
    assert isinstance(sampled, StepPlanningProblem)
    assert isinstance(sampled.sys, SampledSystem) and isinstance(
        sampled.cost, SampledCost
    )
    other = sampled.compile()
    assert (other.N, other.gamma, other.dt) == (face.N, face.gamma, face.dt)

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
    face = problem.compile(dt=0.1, solver="rk4", substeps=4)
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


def test_a_step_problem_has_its_own_horizon_discount_and_tick_offset():
    from minilink.core.discrete.step_system import StepSystem

    class Walk(StepSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=1, output_dim=1)
            self.state.lower_bound[:] = [0.0]
            self.state.upper_bound[:] = [5.0]
            self.inputs["u"].lower_bound[:] = [-1.0]
            self.inputs["u"].upper_bound[:] = [1.0]

        def step(self, x, u, k=0, params=None):
            return np.clip(x + u, 0.0, 5.0)

    problem = StepPlanningProblem(
        Walk(), N=6, cost=Moves([4.0]), gamma=0.9, x_start=[1.0], t_start=2
    )
    face = problem.compile()
    xs, us, cs, J = face.run(lambda x, t: np.ones(1))
    np.testing.assert_array_equal(xs[:, 0], [1, 2, 3, 4, 5, 5, 5])
    assert J == pytest.approx(1 + 0.9 + 0.81 + 0.0 + 0.9**4 + 0.9**5)
    assert face.time_of(3) == 5  # the tick offset, an integer
    assert set(problem.params) >= {"plant", "cost", "x_start", "t_start", "gamma"}

    with pytest.raises(TypeError, match="SampledCost"):
        StepPlanningProblem(Walk(), N=3, cost=QuadraticCost.from_system(Pendulum()))
    with pytest.raises(ValueError, match="tick offset"):
        StepPlanningProblem(Walk(), N=3, cost=Moves([4.0]), t_start=0.5)


def test_the_state_only_step_is_the_transitions_state_on_both_faces():
    problem = swing_problem(t_start=2.0)
    face = problem.compile(dt=0.05, solver="rk4")
    sampled = problem.discretize(0.05, solver="rk4").compile()
    x, u = np.array([0.4, -0.3]), np.array([1.0])
    for t in (2.0, 2.35):
        np.testing.assert_array_equal(
            face.step_at(x, u, t), face.transition_at(x, u, t)[0]
        )
        np.testing.assert_array_equal(sampled.step_at(x, u, t), face.step_at(x, u, t))

    # A problem without a cost still steps: a grid needs only its successors
    free = PlanningProblem(problem.sys, x_start=[0.0, 0.0])
    assert free.compile(dt=0.1).step_at(x, u, 0.0).shape == (2,)


def grid_world():
    """A walker on a 7 × 7 grid of cells, a step to any of its 8 neighbours or none, one unit a move off the goal."""
    from minilink.core.discrete.step_system import StepSystem

    class Walker(StepSystem):
        def __init__(self):
            super().__init__(n=2, input_dim=2, output_dim=2)
            self.state.lower_bound[:] = [0.0, 0.0]
            self.state.upper_bound[:] = [6.0, 6.0]
            self.inputs["u"].lower_bound[:] = [-1.0, -1.0]
            self.inputs["u"].upper_bound[:] = [1.0, 1.0]

        def step(self, x, u, k=0, params=None):
            return np.clip(x + u, 0.0, 6.0)

    walker = Walker()
    return StepPlanningProblem(
        walker,
        cost=Moves([5.0, 3.0]),
        x_start=[0.0, 0.0],
        X=walker.state.box,
        U=walker.input_box,
        infeasible_cost=100.0,
    )


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
    assert vi.grid.dt == 1.0 and vi.options.alpha == 1.0


class TimedTerminal(QuadraticCost):
    """``h(x, t) = 1 + t``: a terminal cost that reads the time it is charged at."""

    def h(self, x, t=0.0, params=None):
        return 1.0 + t + 0.0 * x[0]


def test_the_discretized_face_charges_the_terminal_cost_at_the_horizons_time():
    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=TimedTerminal.from_system(plant),
        x_start=[0.3, 0.0],
        tf=2.0,
        t_start=0.5,
    )
    face = problem.compile(dt=0.05)
    sampled = problem.discretize(0.05).compile()
    x = np.array([0.1, 0.2])
    assert face.terminal(x) == sampled.terminal(x) == 1.0 + 2.5
    law = lambda x, t: np.array([-x[0]])  # noqa: E731
    assert face.run(law)[3] == sampled.run(law)[3]


def test_value_iteration_on_a_problem_and_on_its_discretization_agree():
    from minilink.planning import DynamicProgrammingPlanner

    problem = swing_problem(tf=3.0, discount_rate=0.5)
    kwargs = dict(x_grid=(21, 21), u_grid=(5,), solver="rk4")
    vi = DynamicProgrammingPlanner(problem, dt=0.1, **kwargs)
    vd = DynamicProgrammingPlanner(problem.discretize(0.1, solver="rk4"), **kwargs)
    assert vi.options.out_of_bound_cost == vd.options.out_of_bound_cost
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
        x_start=[0.3, 0.0],
        tf=1.0,
        disturbances={"d": WhiteNoise(1, psd=0.5, sample_period=0.01, seed=1)},
    )
    with pytest.raises(ValueError, match="disturbance sources"):
        noisy.discretize(0.1)


def test_the_policy_evaluator_scores_value_iterations_policy_on_the_face():
    from minilink.planning import DynamicProgrammingPlanner
    from minilink.planning.dynamic_programming.policy_eval import PolicyEvaluator

    problem = swing_problem(tf=np.inf, discount_rate=0.5)
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
        problem, grid=vi.grid, policy=vi.get_controller(), options=vi.options
    )
    np.testing.assert_allclose(pe.solve(), vi.result.J, atol=1e-6)


def test_the_design_point_of_a_random_plant_parameter_is_its_mean():
    from minilink import Uniform

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x_start=[0.3, 0.0],
        tf=1.0,
        params={"plant": {"m": Uniform(1.5, 2.5)}},
    )
    assert problem.sample_params(None)["plant"]["m"] == 2.0
    face = problem.compile(dt=0.1)
    assert face.params["plant"]["m"] == 2.0 == problem.nominal().sys.params["m"]
