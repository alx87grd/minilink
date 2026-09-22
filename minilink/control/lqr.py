"""Linear-quadratic regulator design: arrays in, a state-feedback block out.

The infinite horizon solves the algebraic Riccati equation; the finite horizon and
the design along a trajectory sweep the Riccati differential equation backward,
exactly, through the Hamiltonian system of the co-state.
"""

import numpy as np
from scipy.linalg import expm, solve_continuous_are

from minilink.control.state import (
    StateFeedbackController,
    TimeVaryingStateFeedbackController,
    TrajectoryFeedbackController,
)


def lqr_gain(A, B, Q, R):
    """Return the optimal feedback gain ``K`` of the law ``u = -K x`` minimizing ``∫ xᵀQx + uᵀRu dt``."""
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))

    # AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0
    P = solve_continuous_are(A, B, Q, R)

    # K = R⁻¹ Bᵀ P
    K = np.linalg.solve(R, B.T @ P)

    return K


def lqr(A, B, Q, R, xbar=None, ubar=None):
    """Design an LQR and return a ``StateFeedbackController``.

    The block implements ``u = ubar - K (x - r)`` with ``r`` defaulting to
    ``xbar``; wire the plant state into its ``x`` port.
    """
    K = lqr_gain(A, B, Q, R)
    return StateFeedbackController(K, xbar=xbar, ubar=ubar)


def lqr_at_operating_point(
    sys,
    x_bar,
    Q,
    R,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="auto",
    eps=1e-6,
):
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return an LQR controller.

    Combines equilibrium linearization and :func:`lqr`. The returned block
    regulates about the operating point:

        u = u_bar - K (x - r),   r defaulting to x_bar

    Parameters
    ----------
    sys : System
        Plant to linearize.
    x_bar : array-like, shape (n,)
        Operating-point state.
    Q, R : array-like
        LQR state and input weight matrices.
    u_bar : array-like, shape (m,), optional
        Operating-point input. Defaults to the plant nominal port values.
    t : float, optional
        Time at which Jacobians are evaluated.
    params : dict, optional
        Parameter dict forwarded to ``sys.f``.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.

    Returns
    -------
    StateFeedbackController
        Full-state feedback trimmed about ``(x_bar, u_bar)``.
    """
    from minilink.analysis.derivatives import jacobian, operating_point

    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)
    A = jacobian(sys, "f", "x", x_bar, u_bar, t, params, method=method, eps=eps)
    B = jacobian(sys, "f", "u", x_bar, u_bar, t, params, method=method, eps=eps)
    return lqr(A, B, Q, R, xbar=x_bar, ubar=u_bar)


def lqr_gain_schedule(A, B, Q, R, S_f, tf, *, n_steps=1001):
    """Return ``(t, K, S)``: the finite-horizon gain ``K(t)`` and Riccati matrix ``S(t)``.

    Solves the Riccati differential equation backward from the terminal weight,

        -dS/dt = SA + AᵀS - SBR⁻¹BᵀS + Q,   S(t_f) = S_f,

    exactly on a uniform time grid. With the co-state ``λ = S x``, the pair
    ``z = [x; λ]`` follows the linear Hamiltonian system ``ż = H z``, so one
    step of ``Δt`` backward is the linear-fractional map
    ``S ← (E₂₁ + E₂₂ S)(E₁₁ + E₁₂ S)⁻¹`` with ``E = expm(-H Δt)``: no
    integration error, no stiffness, and ``S`` stays symmetric. Returns
    ``K(t) = R⁻¹BᵀS(t)`` for the law ``u = -K(t) x``. ``t`` holds ``n_steps``
    samples from ``0`` to ``tf``; ``K`` has shape ``(n_steps, m, n)`` and ``S``
    shape ``(n_steps, n, n)``.
    """
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    S_f = np.asarray(S_f, dtype=float)
    n = A.shape[0]
    n_steps = int(n_steps)
    if n_steps < 2 or tf <= 0.0:
        raise ValueError("lqr_gain_schedule needs tf > 0 and n_steps >= 2")
    R_inv = np.linalg.inv(R)
    dt = float(tf) / (n_steps - 1)

    # constant dynamics: one transition serves every interval of the backward sweep
    E = riccati_transition(A, B, Q, R_inv, dt)
    S = np.empty((n_steps, n, n))
    S[-1] = S_f
    for k in range(n_steps - 1, 0, -1):
        S[k - 1] = riccati_map(E, S[k])

    t = np.linspace(0.0, float(tf), n_steps)
    K = R_inv @ B.T @ S
    return t, K, S


def lqr_finite_horizon(
    A, B, Q, R, S_f, tf, *, n_steps=1001, xbar=None, ubar=None, after="hold"
):
    """Design a finite-horizon LQR and return a ``TimeVaryingStateFeedbackController``.

    The block implements ``u = ubar - K(t) (x - r)`` with ``K(t)`` from
    :func:`lqr_gain_schedule`, minimizing
    ``∫₀^tf xᵀQx + uᵀRu dt + x(tf)ᵀ S_f x(tf)`` for the linear model. Past the
    horizon the block keeps ``K(tf)`` (``after="hold"``) or switches to the
    infinite-horizon gain of :func:`lqr_gain` (``after="stationary"``).
    """
    t, K, _ = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=n_steps)
    if after == "stationary":
        K_after = lqr_gain(A, B, Q, R)
    elif after == "hold":
        K_after = None
    else:
        raise ValueError("after must be 'hold' or 'stationary'")
    return TimeVaryingStateFeedbackController(
        t, K, xbar=xbar, ubar=ubar, K_after=K_after
    )


def trajectory_lqr(
    sys, trajectory, Q, R, S_f=None, *, params=None, method="auto", eps=1e-6
):
    """Design an LQR along a reference trajectory and return a ``TrajectoryFeedbackController``.

    ``sys`` is linearized at every sample ``(x_d(t_k), u_d(t_k))`` of
    ``trajectory``, ``A_k = ∂f/∂x`` and ``B_k = ∂f/∂u``, and the Riccati
    differential equation of the error ``x - x_d`` is solved backward along
    the samples with the dynamics frozen on each interval,

        -dS/dt = S A_k + A_kᵀ S - S B_k R⁻¹ B_kᵀ S + Q,   S(t_f) = S_f,

    each interval exactly, as in :func:`lqr_gain_schedule`. The gains
    ``K_k = R⁻¹ B_kᵀ S_k`` give the law ``u = u_d(t) - K(t) (x - x_d(t))``.
    ``S_f`` defaults to the algebraic Riccati solution at the final sample, so
    the gain held past the horizon is the stationary LQR of the end point.

    Parameters
    ----------
    sys : System
        Plant to stabilize.
    trajectory : Trajectory
        Reference ``(t, x_d, u_d)``, ``N`` samples at increasing times.
    Q, R : array-like
        LQR weights on the tracking error and on the input deviation.
    S_f : array-like, optional
        Terminal weight ``S(t_f)``; default the stationary solution at the end.
    params : dict, optional
        Parameter dict forwarded to ``sys.f``.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """
    from minilink.analysis.derivatives import jacobian

    t = np.asarray(trajectory.t, dtype=float).reshape(-1)
    X = np.asarray(trajectory.x, dtype=float).T
    U = np.asarray(trajectory.u, dtype=float).T
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    N, n = X.shape
    m = U.shape[1]
    R_inv = np.linalg.inv(R)

    # the dynamics linearized at every sample of the reference
    A = np.empty((N, n, n))
    B = np.empty((N, n, m))
    for k in range(N):
        A[k] = jacobian(sys, "f", "x", X[k], U[k], t[k], params, method=method, eps=eps)
        B[k] = jacobian(sys, "f", "u", X[k], U[k], t[k], params, method=method, eps=eps)

    # backward sweep from the terminal weight, dynamics frozen on each interval
    S = np.empty((N, n, n))
    S[-1] = solve_continuous_are(A[-1], B[-1], Q, R) if S_f is None else S_f
    for k in range(N - 1, 0, -1):
        S[k - 1] = riccati_step(A[k - 1], B[k - 1], Q, R_inv, S[k], t[k] - t[k - 1])

    K = R_inv @ np.swapaxes(B, 1, 2) @ S
    return TrajectoryFeedbackController(trajectory, K)


# Internal machinery: the Riccati differential equation, one exact step at a time


def hamiltonian_matrix(A, B, Q, R_inv):
    """The Hamiltonian matrix: with the co-state ``λ = S x``, ``z = [x; λ]`` follows ``ż = H z``."""
    # fmt: off
    H = np.block([
        [A, -B @ R_inv @ B.T],
        [-Q, -A.T],
    ])
    # fmt: on

    return H


def riccati_transition(A, B, Q, R_inv, dt):
    """Transition of the Hamiltonian system over one step backward in time."""
    H = hamiltonian_matrix(A, B, Q, R_inv)

    # E = expm(−H dt)
    E = expm(-H * dt)

    return E


def riccati_step(A, B, Q, R_inv, S, dt):
    """``S(t) → S(t - dt)`` with the dynamics frozen over the interval, exactly.

    The interval is cut into substeps with ``‖H‖ dt ≤ 1`` each, so that the
    transition stays well conditioned however stiff the dynamics or long the
    interval; the map is exact on every substep and preserves ``S ⪰ 0``.
    """
    H = hamiltonian_matrix(A, B, Q, R_inv)
    n_sub = max(1, int(np.ceil(np.linalg.norm(H, 2) * dt)))
    E = expm(-H * dt / n_sub)
    for _ in range(n_sub):
        S = riccati_map(E, S)
    return S


def riccati_map(E, S):
    """One exact step ``S(t) → S(t - dt)`` of the Riccati equation, from the transition ``E``.

    ``[X; Y] = E [I; S(t)]`` propagates the pair ``(x, λ)``; the new Riccati
    matrix is ``S(t - dt) = Y X⁻¹``, symmetrized against roundoff.
    """
    n = S.shape[0]
    E11, E12, E21, E22 = E[:n, :n], E[:n, n:], E[n:, :n], E[n:, n:]

    # [X; Y] = E [I; S(t)], then S(t − dt) = Y X⁻¹, symmetrized against roundoff
    X = E11 + E12 @ S
    Y = E21 + E22 @ S
    S_prev = np.linalg.solve(X.T, Y.T).T
    S_prev = (S_prev + S_prev.T) / 2.0

    return S_prev


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole

    plant = CartPole()
    x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin

    controller = lqr_at_operating_point(
        plant,
        x_bar,
        Q=np.diag([1.0, 10.0, 1.0, 1.0]),
        R=np.array([[0.1]]),
    )
    K = controller.params["K"]
    print("LQR gain K =\n", np.round(K, 4))

    diagram = controller @ plant

    plant.x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])

    diagram.compute_trajectory(tf=8.0)
    diagram.plot_trajectory()
    diagram.animate()
