"""Value iteration's finite horizon: a law per stage, its cost-to-go in time, and both planners' law past the horizon."""

import numpy as np
import pytest

from minilink import (
    DoubleIntegrator,
    DynamicProgrammingPlanner,
    LQRPlanner,
    PlanningProblem,
    QuadraticCost,
)
from minilink.core.fields import GridField
from minilink.planning.dynamic_programming.dp import STAGE_MEMORY_LIMIT
from minilink.planning.dynamic_programming.lookup_policy import LookupPolicy, stage_of
from tests.unittest.test_planner_params import GRID, reach

# reach(): the bounded pendulum to the upright over 5 s, N = 50 stages at dt = 0.1
N = 50


def finite(verbose=False, **kwargs):
    return DynamicProgrammingPlanner(reach(), **GRID, **kwargs).solve(verbose=verbose)


def double_integrator(horizon=1.0, S=np.eye(2)):
    sys = DoubleIntegrator()
    sys.state.lower = np.array([-2.0, -2.0])
    sys.state.upper = np.array([2.0, 2.0])
    sys.inputs["u"].lower = np.array([-4.0])
    sys.inputs["u"].upper = np.array([4.0])
    cost = QuadraticCost.from_system(sys, Q=np.eye(2), R=np.eye(1), S=S)

    return PlanningProblem(
        sys,
        x0=np.array([0.5, 0.0]),
        cost=cost,
        X=sys.state.box,
        U=sys.input_box,
        horizon=horizon,
    )


def test_the_horizon_picks_the_mode():
    solution = finite()
    record = solution.record
    t, J, pi = record.tables.schedule

    assert record.mode == "finite" and record.n_iterations == N and record.success
    assert t.shape == (N + 1,) and J.shape[0] == N + 1 and pi.shape[0] == N
    np.testing.assert_allclose(t, np.linspace(0.0, 5.0, N + 1), atol=1e-12)
    assert (
        isinstance(solution.controller, LookupPolicy)
        and "t" in solution.controller.params
    )
    assert isinstance(solution.cost_to_go, GridField)
    np.testing.assert_array_equal(solution.cost_to_go.t, t)

    infinite = DynamicProgrammingPlanner(
        reach().replace(horizon=None), max_iterations=30, **GRID
    ).solve()
    assert infinite.record.mode == "infinite"
    assert infinite.record.tables.schedule is None
    assert "t" not in infinite.controller.params


def test_the_last_stage_is_the_terminal_cost_and_the_first_the_full_horizon_tables():
    planner = DynamicProgrammingPlanner(reach(), **GRID)
    solution = planner.solve()
    t, J, pi = solution.record.tables.schedule

    np.testing.assert_array_equal(J[-1], planner.terminal_cost(planner.tf))
    np.testing.assert_array_equal(J[0], solution.record.tables.J)
    np.testing.assert_array_equal(pi[0], solution.record.tables.pi)


def test_each_stage_is_the_receding_law_of_the_steps_left():
    # Stage i of the finite horizon is solve_steps(N − i): its cost-to-go and its law
    planner = DynamicProgrammingPlanner(reach(), **GRID)
    t, J, pi = planner.solve().record.tables.schedule

    for i in (0, 17, N - 1):
        receding = planner.solve_steps(N - i)
        assert receding.record.mode == "receding"
        np.testing.assert_array_equal(J[i], receding.record.tables.J)
        np.testing.assert_array_equal(pi[i], receding.record.tables.pi)


def test_the_law_holds_each_stage_and_stage_zero_outside_the_horizon():
    solution = finite()
    t, _, pi = solution.record.tables.schedule
    grid = solution.record.tables.grid
    law = solution.controller
    stage = {
        i: LookupPolicy(grid, pi[i]) for i in (0, 1, 3, N - 1)
    }  # each stage's own law
    rng = np.random.default_rng(0)
    xs = rng.uniform(grid.x_lb, grid.x_ub, size=(10, 2))

    for x in xs:
        np.testing.assert_array_equal(law.action(x, t=0.0), stage[0].action(x))
        np.testing.assert_array_equal(law.action(x, t=0.15), stage[1].action(x))
        np.testing.assert_array_equal(law.action(x, t=4.95), stage[N - 1].action(x))
        # Past the horizon and before it: stage 0, the full-horizon law
        np.testing.assert_array_equal(law.action(x, t=5.0), stage[0].action(x))
        np.testing.assert_array_equal(law.action(x, t=12.0), stage[0].action(x))
        np.testing.assert_array_equal(law.action(x, t=-1.0), stage[0].action(x))


def test_a_float_tick_reads_its_own_stage():
    # A tick summed from dt drifts from t_i = i dt either way by rounding; a tick a
    # hair short of t_i still reads stage i, not i − 1
    times = np.linspace(0.0, 5.0, N + 1)
    tick = 0.0
    for i in range(N):
        assert int(stage_of(times, tick, np)) == i
        assert int(stage_of(times, np.nextafter(times[i], -np.inf), np)) == i
        tick += 0.1
    assert int(stage_of(times, 5.0, np)) == 0


def test_the_cost_to_go_reads_each_stage_at_its_time():
    solution = finite()
    t, J, _ = solution.record.tables.schedule
    grid = solution.record.tables.grid
    x = grid.states[40]

    for i in (0, 10, N):
        assert solution.cost_to_go(x, t=t[i]) == pytest.approx(J[i][40], rel=1e-12)


def test_the_failure_cost_sums_over_the_horizon():
    planner = DynamicProgrammingPlanner(reach(), **GRID)
    model = planner.grid.model

    assert planner.out_of_bound_cost == model.failure_cost_over(N, 1.0)


def test_a_horizon_too_long_for_memory_is_refused_with_its_numbers():
    nodes = 9 * 9
    stages = STAGE_MEMORY_LIMIT // (8 * nodes) + 1
    problem = reach().replace(horizon=0.1 * stages)

    with pytest.raises(ValueError, match="GiB"):
        DynamicProgrammingPlanner(problem, **GRID).solve()


def test_value_iteration_and_lqr_agree_on_the_finite_double_integrator():
    # A fine grid against the Riccati sweep: the grid's interpolation overestimates
    # J away from the end (about 25 % at t0 here), and both meet at the horizon
    problem = double_integrator()
    vi = DynamicProgrammingPlanner(
        problem, x_grid=(41, 41), u_grid=(41,), dt=0.02, out_of_bound_cost=1e3
    ).solve()
    lqr = LQRPlanner(problem).solve()

    for x in ([0.5, 0.0], [0.0, 0.5], [-0.4, 0.3]):
        x = np.array(x)
        assert vi.cost_to_go(x, t=0.0) == pytest.approx(
            lqr.cost_to_go(x, t=0.0), rel=0.3
        )
        assert vi.cost_to_go(x, t=0.9) == pytest.approx(
            lqr.cost_to_go(x, t=0.9), rel=0.05
        )


def test_past_the_horizon_lqr_applies_the_stationary_gain():
    # S = 0: K(tf) = 0, so holding the last gain would stop regulating
    problem = double_integrator(S=np.zeros((2, 2)))
    finite_law = LQRPlanner(problem).solve().controller
    stationary = LQRPlanner(problem.replace(horizon=None)).solve().record.K
    x = np.array([0.5, -0.2, 0.0, 0.0])  # the state, then the reference x̄

    np.testing.assert_allclose(finite_law.params["K"][-1], 0.0, atol=1e-12)
    np.testing.assert_allclose(finite_law.params["K_after"], stationary, rtol=1e-12)
    np.testing.assert_allclose(
        finite_law.outputs["u"].compute(None, x, 1.5),
        -stationary @ x[:2],
        rtol=1e-12,
    )


def test_past_the_horizon_value_iteration_applies_stage_zero_not_a_tie():
    # h = 0: the last stage's backup is a tie at every node, broken at action 0, the
    # corner of the input box; past the horizon the law applies stage 0 instead
    problem = double_integrator(S=np.zeros((2, 2)))
    solution = DynamicProgrammingPlanner(
        problem, x_grid=(21, 21), u_grid=(9,), dt=0.05, out_of_bound_cost=1e3
    ).solve()
    _, _, pi = solution.record.tables.schedule
    x = np.array([1.0, 0.5])

    assert np.all(pi[-1] == pi[-1][0])  # The tie: one action everywhere
    np.testing.assert_array_equal(
        solution.controller.action(x, t=2.0), solution.controller.action(x, t=0.0)
    )
    assert solution.controller.action(x, t=2.0)[0] < 0.0  # Regulating, toward 0


def test_lqrs_schedule_runs_on_the_problems_clock():
    problem = double_integrator().replace(t0=2.0)
    t, _, _ = LQRPlanner(problem).solve().record.schedule

    assert t[0] == 2.0 and t[-1] == pytest.approx(3.0, abs=1e-12)


@pytest.mark.jax
def test_the_jax_engine_keeps_the_same_stages():
    pytest.importorskip("jax")
    on_numpy = finite().record.tables
    for verbose in (False, True):
        on_jax = finite(backend="jax", verbose=verbose).record.tables

        t_np, J_np, pi_np = on_numpy.schedule
        t_jx, J_jx, pi_jx = on_jax.schedule
        np.testing.assert_array_equal(t_jx, t_np)
        np.testing.assert_allclose(J_jx, J_np, rtol=1e-9, atol=1e-9)
        np.testing.assert_array_equal(pi_jx, pi_np)
        assert on_jax.delta == pytest.approx(on_numpy.delta, rel=1e-9, abs=1e-9)


@pytest.mark.jax
def test_the_staged_law_closes_a_jax_loop_as_on_numpy():
    pytest.importorskip("jax")
    problem = reach()
    law = finite().controller
    plant = problem.sys
    plant.x0 = np.array([0.5, 0.0])
    loop = law @ plant

    on_numpy = loop.compute_trajectory(tf=6.0, solver="rk4")
    on_jax = loop.compute_trajectory(tf=6.0, solver="rk4", backend="jax")

    np.testing.assert_allclose(np.asarray(on_jax.x), on_numpy.x, rtol=1e-9, atol=1e-12)
