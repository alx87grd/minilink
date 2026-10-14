"""The Riccati equations of linear-quadratic design: the algebraic solution and the exact step of the differential one."""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_are

# Public API


def care(A, B, Q, R):
    """Return ``P``, the stabilizing solution of the continuous algebraic Riccati equation."""
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))

    # Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0
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
    """The transition ``E`` of the Hamiltonian system over one step of ``dt`` backward in time."""
    H = hamiltonian_matrix(A, B, Q, R_inv)

    E = expm(-H * dt)

    return E


def riccati_step(A, B, Q, R_inv, S, dt):
    """``S(t) → S(t − dt)`` with the dynamics frozen over the interval, exactly.

    The same step on the dual pair ``(Aᵀ, Cᵀ)`` carries a filter covariance from
    ``P(t)`` forward to ``P(t + dt)``: the filter equation is the control equation
    with time reversed and the matrices transposed.
    """
    H = hamiltonian_matrix(A, B, Q, R_inv)
    n_sub = substeps(H, dt)

    # −dS/dt = S A + Aᵀ S − S B R⁻¹ Bᵀ S + Q: z = [x; λ] follows ż = H z, so
    # E = expm(−H dt) carries it back exactly, one substep at a time
    E = expm(-H * dt / n_sub)
    for _ in range(n_sub):
        S = riccati_map(E, S)

    return S


def riccati_map(E, S):
    """One exact step ``S(t) → S(t − dt)`` of the Riccati equation, from the transition ``E``."""
    n = S.shape[0]
    E11, E12, E21, E22 = E[:n, :n], E[:n, n:], E[n:, :n], E[n:, n:]

    # [X; Y] = E [I; S(t)], then S(t − dt) = Y X⁻¹, symmetrized against roundoff
    X = E11 + E12 @ S
    Y = E21 + E22 @ S
    S_new = np.linalg.solve(X.T, Y.T).T
    S_new = (S_new + S_new.T) / 2.0

    return S_new


# Internal machinery


def substeps(H, dt):
    """The substeps that cut ``dt`` to ``‖H‖ dt ≤ 1`` each, so the transition stays well conditioned however stiff the pair."""
    return max(1, int(np.ceil(np.linalg.norm(H, 2) * dt)))
