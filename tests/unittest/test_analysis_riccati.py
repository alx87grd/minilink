"""The Riccati equations: the algebraic solution, and the exact Hamiltonian step in both directions."""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

from minilink.analysis.riccati import care, riccati_step

A = np.array([[0.0, 1.0], [-1.0, -0.5]])
B = np.array([[0.0], [1.0]])
C = np.array([[1.0, 0.0]])
Q = np.diag([1.0, 2.0])
R = np.array([[0.5]])


def test_care_is_the_stabilizing_algebraic_solution():
    P = care(A, B, Q, R)
    np.testing.assert_allclose(P, solve_continuous_are(A, B, Q, R))
    residual = A.T @ P + P @ A - P @ B @ np.linalg.inv(R) @ B.T @ P + Q
    np.testing.assert_allclose(residual, 0.0, atol=1e-10)
    assert np.all(np.linalg.eigvals(A - B @ np.linalg.solve(R, B.T @ P)).real < 0)


def test_backward_step_integrates_the_control_equation():
    R_inv = np.linalg.inv(R)
    S_f = np.diag([3.0, 1.0])
    dt = 0.7

    # -dS/dt = S A + Aᵀ S − S B R⁻¹ Bᵀ S + Q, from S(t_f) back to S(t_f − dt)
    def rhs(tau, s):
        S = s.reshape(2, 2)
        return (S @ A + A.T @ S - S @ B @ R_inv @ B.T @ S + Q).ravel()

    reference = solve_ivp(rhs, (0.0, dt), S_f.ravel(), rtol=1e-11, atol=1e-13).y[:, -1]
    S = riccati_step(A, B, Q, R_inv, S_f, dt)
    np.testing.assert_allclose(S, reference.reshape(2, 2), rtol=1e-7, atol=1e-9)


def test_the_same_step_on_the_dual_pair_integrates_the_filter_equation_forward():
    Q_n = np.diag([0.1, 0.3])
    R_n = np.array([[0.2]])
    R_inv = np.linalg.inv(R_n)
    P_0 = np.eye(2)
    dt = 0.7

    # dP/dt = A P + P Aᵀ − P Cᵀ R⁻¹ C P + Q, from P(0) forward to P(dt): the control
    # equation on (Aᵀ, Cᵀ) with time reversed, so the backward step runs it forward
    def rhs(tau, p):
        P = p.reshape(2, 2)
        return (A @ P + P @ A.T - P @ C.T @ R_inv @ C @ P + Q_n).ravel()

    reference = solve_ivp(rhs, (0.0, dt), P_0.ravel(), rtol=1e-11, atol=1e-13).y[:, -1]
    P = riccati_step(A.T, C.T, Q_n, R_inv, P_0, dt)
    np.testing.assert_allclose(P, reference.reshape(2, 2), rtol=1e-7, atol=1e-9)
    np.testing.assert_allclose(P, P.T)
