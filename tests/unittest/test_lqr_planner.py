"""The linear-quadratic regulator as a planner: the Riccati factories' numbers, as a solution."""

import numpy as np
import pytest
from scipy.linalg import solve_continuous_are

from minilink import LQRPlanner, Pendulum, PlanningProblem, QuadraticCost
from minilink.control.lqr import lqr_gain, lqr_gain_schedule
from minilink.control.state import (
    StateFeedbackController,
    TimeVaryingStateFeedbackController,
)
from minilink.core.costs import TimeCost
from minilink.planning.policy_synthesis.lqr import RiccatiRecord

UPRIGHT = np.array([np.pi, 0.0])
Q = np.diag([10.0, 1.0])
R = np.array([[0.5]])


def pendulum_problem(tf=np.inf, S=None):
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-20.0])
    plant.inputs["u"].upper_bound = np.array([20.0])
    cost = QuadraticCost.from_system(plant, Q=Q, R=R, S=S, xbar=UPRIGHT)
    return PlanningProblem(
        plant, x_start=UPRIGHT + [0.3, 0.0], x_goal=UPRIGHT, cost=cost, tf=tf
    )


def test_infinite_horizon_is_the_algebraic_riccati_solution():
    problem = pendulum_problem()
    planner = LQRPlanner(problem)
    solution = planner.solve()
    A, B = planner.linear_model()
    P = solve_continuous_are(A, B, Q, R)

    assert isinstance(solution.solver, RiccatiRecord) and solution.success
    assert solution.problem is problem and solution.method == "riccati"
    np.testing.assert_allclose(solution.solver.K, lqr_gain(A, B, Q, R))
    np.testing.assert_allclose(solution.solver.P, P)
    assert np.all(np.real(solution.solver.poles) < 0.0)
    assert isinstance(solution.policy, StateFeedbackController)
    np.testing.assert_allclose(solution.policy.inputs["r"].nominal_value, UPRIGHT)
    x = UPRIGHT + [0.4, -0.2]
    assert solution.cost_to_go(x) == pytest.approx((x - UPRIGHT) @ P @ (x - UPRIGHT))
    assert solution.trajectory is None and solution.evaluation is None
    assert "closed-loop poles" in str(solution.solver)


def test_finite_horizon_is_the_riccati_differential_equation():
    tf, S_f = 2.0, np.diag([5.0, 1.0])
    planner = LQRPlanner(pendulum_problem(tf=tf, S=S_f), n_steps=201)
    solution = planner.solve()
    A, B = planner.linear_model()
    t, K, S = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=201)

    assert isinstance(solution.policy, TimeVaryingStateFeedbackController)
    assert solution.success and not solution.open_loop
    np.testing.assert_allclose(solution.policy.params["K"], K)
    np.testing.assert_allclose(solution.solver.K, K[0])
    x = UPRIGHT + [0.1, 0.3]
    for k in (0, 100, 200):
        dx = x - UPRIGHT
        assert solution.cost_to_go(x, t[k]) == pytest.approx(dx @ S[k] @ dx)
    assert solution.cost_to_go(x, tf) == pytest.approx(
        (x - UPRIGHT) @ S_f @ (x - UPRIGHT)
    )


def test_lqr_needs_a_quadratic_cost():
    plant = Pendulum()
    problem = PlanningProblem(plant, cost=TimeCost.from_system(plant), tf=np.inf)
    with pytest.raises(TypeError, match="QuadraticCost"):
        LQRPlanner(problem)


def test_evaluate_rolls_out_and_scores_and_the_loop_balances_the_pendulum():
    problem = pendulum_problem()
    solution = LQRPlanner(problem, dt=0.02).solve(evaluate=True)
    assert solution.trajectory is not None and solution.evaluation.n_trials == 1
    np.testing.assert_allclose(solution.trajectory.x[:, 0], problem.x_start)
    assert solution.evaluation.mean == pytest.approx(
        solution.cost_to_go(problem.x_start), rel=0.5
    )

    # the loop is closed in the script: the policy is a controller block
    plant = problem.sys
    plant.x0 = problem.x_start
    loop = solution.policy @ plant
    traj = loop.compute_trajectory(tf=6.0, verbose=False)
    np.testing.assert_allclose(traj.x[:, -1], UPRIGHT, atol=1e-3)
