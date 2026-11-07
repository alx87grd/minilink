"""The linear-quadratic regulator as a planner: the Riccati factories' numbers, as a solution."""

import numpy as np
import pytest
from scipy.linalg import solve_continuous_are

from minilink import (
    DoubleIntegrator,
    DynamicProgrammingPlanner,
    LQRPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
)
from minilink.control.design import lqr_gain, lqr_gain_schedule
from minilink.control.state import (
    StateFeedback,
    TimeVaryingFeedback,
)
from minilink.core.costs import TimeCost
from minilink.core.sets import BoxSet
from minilink.planning.dynamic_programming.lqr import RiccatiRecord

UPRIGHT = np.array([np.pi, 0.0])
Q = np.diag([10.0, 1.0])
R = np.array([[0.5]])


def pendulum_problem(tf=np.inf, S=None):
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-20.0])
    plant.inputs["u"].upper_bound = np.array([20.0])
    cost = QuadraticCost.from_system(plant, Q=Q, R=R, S=S, x_bar=UPRIGHT)
    return PlanningProblem(
        plant,
        x0=UPRIGHT + [0.3, 0.0],
        cost=cost,
        horizon=tf,
        U=plant.input_box,
    )


def test_infinite_horizon_is_the_algebraic_riccati_solution():
    problem = pendulum_problem()
    planner = LQRPlanner(problem)
    solution = planner.solve()
    A, B = planner.linear_model()
    P = solve_continuous_are(A, B, Q, R)

    assert isinstance(solution.record, RiccatiRecord) and solution.success
    assert solution.problem is problem and solution.method == "riccati"
    np.testing.assert_allclose(solution.record.K, lqr_gain(A, B, Q, R))
    np.testing.assert_allclose(solution.record.P, P)
    assert np.all(np.real(solution.record.poles) < 0.0)
    assert isinstance(solution.controller, StateFeedback)
    np.testing.assert_allclose(solution.controller.inputs["r"].nominal_value, UPRIGHT)
    x = UPRIGHT + [0.4, -0.2]
    assert solution.cost_to_go(x) == pytest.approx((x - UPRIGHT) @ P @ (x - UPRIGHT))
    assert solution.trajectory is None and solution.evaluation is None
    assert "closed-loop poles" in str(solution.record)


@pytest.mark.parametrize("tf", [np.inf, 2.0])
def test_the_record_says_U_ignored_and_nothing_warns(tf):
    """LQR never reads U: its record's printout says so when U bounds the command, silently."""
    import warnings

    bounded = pendulum_problem(tf=tf)
    free = bounded.replace(U=None)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        bounded_record = LQRPlanner(bounded).solve().record
        free_record = LQRPlanner(free).solve().record

    assert bounded_record.U_ignored and "U ignored" in str(bounded_record)
    assert not free_record.U_ignored and "U ignored" not in str(free_record)
    np.testing.assert_array_equal(bounded_record.K, free_record.K)


def test_finite_horizon_is_the_riccati_differential_equation():
    tf, S_f = 2.0, np.diag([5.0, 1.0])
    planner = LQRPlanner(pendulum_problem(tf=tf, S=S_f), n_steps=201)
    solution = planner.solve()
    A, B = planner.linear_model()
    t, K, S = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=201)

    assert isinstance(solution.controller, TimeVaryingFeedback)
    assert solution.success and not solution.open_loop
    np.testing.assert_allclose(solution.controller.params["K"], K)
    np.testing.assert_allclose(solution.record.K, K[0])
    x = UPRIGHT + [0.1, 0.3]
    for k in (0, 100, 200):
        dx = x - UPRIGHT
        assert solution.cost_to_go(x, t=t[k]) == pytest.approx(dx @ S[k] @ dx)
    assert solution.cost_to_go(x, t=tf) == pytest.approx(
        (x - UPRIGHT) @ S_f @ (x - UPRIGHT)
    )


def test_a_plant_with_noise_ports_is_designed_about_its_command():
    from minilink import NoisyCartPole, lqr

    plant = NoisyCartPole()  # inputs u, d, n
    x_up = np.array([0.0, np.pi, 0.0, 0.0])
    Q_x = np.diag([1.0, 10.0, 1.0, 1.0])
    R_all = 0.1 * np.eye(plant.m)  # a cost over every input stacked
    cost = QuadraticCost.from_system(plant, Q=Q_x, R=R_all, x_bar=x_up)

    stationary = LQRPlanner(
        PlanningProblem(plant, cost=cost, U=plant.input_box)
    ).solve()
    K = stationary.controller.params["K"]
    assert K.shape == (1, 4)
    np.testing.assert_allclose(
        K, lqr(plant, Q_x, R_all[:1, :1], x_bar=x_up).params["K"]
    )
    finite = LQRPlanner(
        PlanningProblem(plant, cost=cost, horizon=2.0, U=plant.input_box), n_steps=51
    ).solve()
    assert finite.controller.params["K"].shape == (51, 1, 4)


def test_lqr_needs_a_quadratic_cost():
    plant = Pendulum()
    problem = PlanningProblem(
        plant, cost=TimeCost.from_system(plant), horizon=np.inf, U=plant.input_box
    )
    with pytest.raises(TypeError, match="QuadraticCost"):
        LQRPlanner(problem)


def test_evaluate_scores_the_law_and_the_loop_balances_the_pendulum():
    from minilink.planning.results import drawn_trajectory_of

    problem = pendulum_problem()
    solution = LQRPlanner(problem, dt=0.02).solve(evaluate=True)
    assert solution.trajectory is None and solution.evaluation.n_trials == 1
    np.testing.assert_allclose(drawn_trajectory_of(solution).x[:, 0], problem.x0)
    assert solution.evaluation.mean == pytest.approx(
        solution.cost_to_go(problem.x0), rel=0.5
    )

    # the loop is closed in the script: the policy is a controller block
    plant = problem.sys
    plant.x0 = problem.x0
    loop = solution.controller @ plant
    traj = loop.compute_trajectory(tf=6.0, verbose=False)
    np.testing.assert_allclose(traj.x[:, -1], UPRIGHT, atol=1e-3)


# A discounted problem


def discounted_double_integrator(rho):
    plant = DoubleIntegrator()
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1))
    return PlanningProblem(
        plant,
        cost=cost,
        X=BoxSet([-1.0, -1.0], [1.0, 1.0]),
        U=BoxSet([-2.0], [2.0]),
        discount_rate=rho,
    )


def test_a_discounted_problem_solves_the_discounted_riccati_equation_and_warns():
    rho = 1.0
    with pytest.warns(UserWarning, match="discounted"):
        solution = LQRPlanner(discounted_double_integrator(rho)).solve()
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    P = solution.record.P
    K = solution.record.K

    # Aᵀ P + P A − ρ P − P B R⁻¹ Bᵀ P + Q = 0, the discounted HJB of the quadratic form
    residual = A.T @ P + P @ A - rho * P - P @ B @ B.T @ P + np.eye(2)
    np.testing.assert_allclose(residual, np.zeros((2, 2)), atol=1e-10)
    np.testing.assert_allclose(K, B.T @ P)

    # The poles are the plant's closed loop, eig(A − B K)
    np.testing.assert_allclose(
        np.sort_complex(solution.record.poles),
        np.sort_complex(np.linalg.eigvals(A - B @ K)),
    )


def test_discounted_lqr_agrees_with_value_iteration_where_the_undiscounted_does_not():
    rho = 1.0
    with pytest.warns(UserWarning, match="discounted"):
        lqr = LQRPlanner(discounted_double_integrator(rho)).solve()
    undiscounted = LQRPlanner(discounted_double_integrator(0.0)).solve()
    vi = DynamicProgrammingPlanner(
        discounted_double_integrator(rho),
        x_grid=(41, 41),
        u_grid=(41,),
        dt=0.02,
        max_iterations=4000,
        tol=1e-7,
    ).solve()

    for x in ([0.5, 0.0], [0.0, 0.5], [0.4, -0.4]):
        x = np.array(x)
        J_vi = vi.cost_to_go(x)
        assert lqr.cost_to_go(x) == pytest.approx(J_vi, rel=0.15)
        assert abs(lqr.cost_to_go(x) - J_vi) < abs(undiscounted.cost_to_go(x) - J_vi)
