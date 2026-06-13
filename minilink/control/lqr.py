"""Linear–quadratic regulator design.

Array-in / block-out design factory (the dependency-law pattern for libraries):
``lqr_gain`` solves the continuous-time algebraic Riccati equation for the
optimal gain, and ``lqr`` wraps it as a ready-to-wire
:class:`~minilink.control.linear.LinearFeedbackController`.

For a nonlinear plant, linearize first with
:func:`minilink.analysis.linearize.linearize` (a tool), then pass the resulting
matrices here — keeping this library module free of tool imports.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from minilink.control.linear import LinearFeedbackController


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
    """Design an LQR and return it as a ``LinearFeedbackController``.

    The block implements ``u = ubar - K (x - r)`` with ``r`` defaulting to
    ``xbar``; wire the plant state into its ``x`` port.
    """
    K = lqr_gain(A, B, Q, R)
    return LinearFeedbackController(K, xbar=xbar, ubar=ubar)


if __name__ == "__main__":
    # Linearize an inverted pendulum at upright, design LQR, check stability.
    from minilink.analysis.linearize import linearize
    from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

    plant = InvertedPendulum()
    lti = linearize(plant, x_bar=[0.0, 0.0])
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))

    K = lqr_gain(lti.A(), lti.B(), Q=np.diag([10.0, 1.0]), R=np.array([[1.0]]))
    closed = lti.A() - lti.B() @ K
    print("gain K:", np.round(K, 4))
    print("closed-loop poles:", np.round(np.linalg.eigvals(closed), 4))
