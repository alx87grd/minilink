"""Linear–quadratic regulator design.

Array-in / block-out design factory (the dependency-law pattern for libraries):
``lqr_gain`` solves the continuous-time algebraic Riccati equation for the
optimal gain, and ``lqr`` wraps it as a ready-to-wire
:class:`~minilink.control.state.StateFeedbackController`.

``lqr_at_operating_point`` linearizes a plant about ``(x_bar, u_bar)`` (via
:func:`~minilink.analysis.linearize.linearize_matrices`, lazy-imported) and
returns the trimmed controller in one step.

``lqr_gain_schedule`` integrates the Riccati differential equation backward
over a finite horizon and ``lqr_finite_horizon`` wraps the resulting gain
schedule ``K(t)`` as a
:class:`~minilink.control.state.TimeVaryingStateFeedbackController`.

For matrix-only design, pass Jacobians from any source into ``lqr_gain`` /
``lqr`` directly.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

from minilink.control.state import (
    StateFeedbackController,
    TimeVaryingStateFeedbackController,
)


def lqr_gain(A, B, Q, R):
    """Return the optimal feedback gain ``K`` minimizing ``∫ xᵀQx + uᵀRu dt``.

    Solves the continuous-time ARE ``AᵀP + PA - PBR⁻¹BᵀP + Q = 0`` and returns
    ``K = R⁻¹BᵀP`` for the law ``u = -K x``.
    """
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))

    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


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

    Integrates the Riccati differential equation backward from the terminal
    weight ``S(t_f) = S_f``,

        -dS/dt = SA + AᵀS - SBR⁻¹BᵀS + Q,

    and returns ``K(t) = R⁻¹BᵀS(t)`` for the law ``u = -K(t) x``. ``t`` holds
    ``n_steps`` samples from ``0`` to ``tf``; ``K`` has shape ``(n_steps, m, n)``
    and ``S`` shape ``(n_steps, n, n)``.
    """
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    S_f = np.asarray(S_f, dtype=float)
    n = A.shape[0]
    R_inv = np.linalg.inv(R)

    # Riccati equation in time-to-go tau = tf - t, integrated forward from S_f
    def riccati(tau, s):
        S = s.reshape(n, n)
        dS = S @ A + A.T @ S - S @ B @ R_inv @ B.T @ S + Q
        return dS.ravel()

    tau = np.linspace(0.0, float(tf), int(n_steps))
    solution = solve_ivp(
        riccati, (0.0, float(tf)), S_f.ravel(), t_eval=tau, rtol=1e-8, atol=1e-10
    )

    # back to forward time t = tf - tau, increasing
    t = float(tf) - tau[::-1]
    S = solution.y.T.reshape(-1, n, n)[::-1]
    K = R_inv @ B.T @ S
    return t, K, S


def lqr_finite_horizon(A, B, Q, R, S_f, tf, *, n_steps=1001, xbar=None, ubar=None):
    """Design a finite-horizon LQR and return a ``TimeVaryingStateFeedbackController``.

    The block implements ``u = ubar - K(t) (x - r)`` with ``K(t)`` from
    :func:`lqr_gain_schedule`, minimizing
    ``∫₀^tf xᵀQx + uᵀRu dt + x(tf)ᵀ S_f x(tf)`` for the linear model.
    """
    t, K, _ = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=n_steps)
    return TimeVaryingStateFeedbackController(t, K, xbar=xbar, ubar=ubar)


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
