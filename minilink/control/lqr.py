"""Linear–quadratic regulator design.

Array-in / block-out design factory (the dependency-law pattern for libraries):
``lqr_gain`` solves the continuous-time algebraic Riccati equation for the
optimal gain, and ``lqr`` wraps it as a ready-to-wire
:class:`~minilink.control.state.StateFeedbackController`.

``lqr_at_operating_point`` linearizes a plant about ``(x_bar, u_bar)`` (via
:func:`~minilink.analysis.linearize.linearize_matrices`, lazy-imported) and
returns the trimmed controller in one step.

For matrix-only design, pass Jacobians from any source into ``lqr_gain`` /
``lqr`` directly.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from minilink.control.state import StateFeedbackController


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
    *,
    t=0.0,
    params=None,
    epsilon=1e-6,
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
        Parameter dict forwarded to ``sys.f`` / ``sys.h``.
    epsilon : float, optional
        Central-difference step for linearization.

    Returns
    -------
    StateFeedbackController
        Full-state feedback trimmed about ``(x_bar, u_bar)``.
    """
    from minilink.analysis.linearize import linearize_matrices

    x_bar = np.asarray(x_bar, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = sys.get_u_from_input_ports()
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)

    A, B, _, _ = linearize_matrices(
        sys, x_bar, u_bar, t=t, params=params, epsilon=epsilon
    )
    return lqr(A, B, Q, R, xbar=x_bar, ubar=u_bar)


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
