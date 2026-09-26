"""Pole placement: arrays in, a state-feedback block out.

One input: Ackermann's formula, the unique gain, repeated poles included. Several
inputs: many gains place the same poles, and ``scipy.signal.place_poles`` picks the
robust one. Either way the closed-loop eigenvalues are checked against the request.
"""

import numpy as np
from scipy.signal import place_poles

from minilink.control.state import StateFeedbackController


def place_gain(A, B, poles):
    """Return the gain ``K`` of the law ``u = -K x`` that puts the eigenvalues of ``A − B K`` at ``poles``."""
    from minilink.analysis.structural import controllability

    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n, m = B.shape
    poles = requested_poles(poles, n)

    # 𝒞 = [B, AB, …, Aⁿ⁻¹B]: every pole can be moved only when rank 𝒞 = n
    ctrb = controllability(A, B)
    require_controllable(ctrb)

    if m == 1:
        # φ(s) = ∏ (s − pᵢ) = sⁿ + a₁ sⁿ⁻¹ + … + aₙ
        a = np.real(np.poly(poles))

        # φ(A) = Aⁿ + a₁ Aⁿ⁻¹ + … + aₙ I   (Horner)
        phi_A = np.zeros((n, n))
        for a_k in a:
            phi_A = phi_A @ A + a_k * np.eye(n)

        # Ackermann: K = [0 … 0 1] 𝒞⁻¹ φ(A)
        e_n = np.eye(n)[-1]
        K = np.linalg.solve(ctrb.matrix.T, e_n)[np.newaxis, :] @ phi_A
    else:
        # Several inputs: many K place these poles; take the robust one (Kautz–Nichols–Van Dooren)
        K = robust_gain(A, B, poles)

    require_placed(A, B, K, poles)

    return K


def place(A, B, poles, xbar=None, ubar=None):
    """Place the closed-loop poles and return a ``StateFeedbackController``.

    The block implements ``u = ubar - K (x - r)`` with ``r`` defaulting to
    ``xbar``; wire the plant state into its ``x`` port.
    """
    K = place_gain(A, B, poles)
    return StateFeedbackController(K, xbar=xbar, ubar=ubar)


def place_at_operating_point(
    sys,
    x_bar,
    poles,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="auto",
    eps=1e-6,
):
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return a pole-placement controller.

    Combines equilibrium linearization and :func:`place`. The returned block
    regulates about the operating point:

        u = u_bar - K (x - r),   r defaulting to x_bar

    Parameters
    ----------
    sys : System
        Plant to linearize.
    x_bar : array-like, shape (n,)
        Operating-point state.
    poles : array-like, shape (n,)
        Closed-loop poles; complex ones in conjugate pairs.
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
    return place(A, B, poles, xbar=x_bar, ubar=u_bar)


# =============================================================================
# Internal machinery: the request checked, the result checked
# =============================================================================


def requested_poles(poles, n):
    """The requested poles as a complex array: exactly ``n``, complex ones in conjugate pairs."""
    poles = np.atleast_1d(np.asarray(poles, dtype=complex))
    if poles.shape != (n,):
        raise ValueError(f"place: {n} poles needed for {n} states, got {poles.size}.")
    if not np.allclose(np.sort_complex(poles), np.sort_complex(poles.conj())):
        raise ValueError(
            "place: complex poles must come in conjugate pairs (a real gain K "
            f"gives a real characteristic polynomial), got {poles}."
        )
    return poles


def require_controllable(ctrb):
    """Refuse an uncontrollable pair, naming the rank of the controllability matrix."""
    if ctrb.rank < ctrb.n:
        raise ValueError(
            f"place: (A, B) is not controllable (rank 𝒞 = {ctrb.rank} < n = "
            f"{ctrb.n}): only the controllable modes can be moved."
        )


def robust_gain(A, B, poles):
    """``scipy.signal.place_poles``, its multiplicity limit said in plain words."""
    try:
        return place_poles(A, B, poles).gain_matrix
    except ValueError as error:
        raise ValueError(
            f"place: with {B.shape[1]} inputs a pole can be repeated at most "
            f"{np.linalg.matrix_rank(B)} times; spread the repeated poles ({error})."
        ) from error


def require_placed(A, B, K, poles):
    """Check ``det(sI − (A − B K)) = ∏ (s − pᵢ)``; a badly conditioned design fails loudly.

    The characteristic polynomials are compared, not the eigenvalues: a repeated
    pole is computed only to about the cube root of machine precision, while its
    polynomial stays exact.
    """
    placed = np.real(np.poly(A - B @ K))
    requested = np.real(np.poly(poles))
    if not np.allclose(
        placed, requested, rtol=1e-6, atol=1e-9 * np.max(np.abs(requested))
    ):
        raise ValueError(
            f"place: the closed loop landed at {np.linalg.eigvals(A - B @ K)}, not at "
            f"the requested {poles}; the design is badly conditioned."
        )
