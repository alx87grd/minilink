"""The Riccati equations of linear-quadratic design: the algebraic solution and one exact step of the differential one.

The algebraic equation gives the stationary gain (:func:`care`); the differential
equation gives a schedule. The control weight ``S(t)`` is swept backward from the
terminal weight, and the same step on the dual pair ``(Aᵀ, Cᵀ)`` sweeps a filter
covariance ``P(t)`` forward from its initial value: the two equations differ by the
sign of time and the transpose. With the co-state ``λ = S x``, the pair ``z = [x; λ]``
follows the linear Hamiltonian system ``ż = H z``, so a step of ``dt`` is a matrix
exponential and a linear-fractional map, with no integration error and no stiffness.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_are


def care(A, B, Q, R):
    """Return ``P``, the stabilizing solution of the continuous algebraic Riccati equation."""
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))

    # AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0
    P = solve_continuous_are(A, B, Q, R)

    return P


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

    ``[X; Y] = E [I; S(t)]`` propagates the pair ``(x, λ)``; the new Riccati matrix
    is ``S(t - dt) = Y X⁻¹``, symmetrized against roundoff.
    """
    n = S.shape[0]
    E11, E12, E21, E22 = E[:n, :n], E[:n, n:], E[n:, :n], E[n:, n:]

    # [X; Y] = E [I; S(t)], then S(t − dt) = Y X⁻¹, symmetrized against roundoff
    X = E11 + E12 @ S
    Y = E21 + E22 @ S
    S_new = np.linalg.solve(X.T, Y.T).T
    S_new = (S_new + S_new.T) / 2.0

    return S_new
