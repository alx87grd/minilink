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
