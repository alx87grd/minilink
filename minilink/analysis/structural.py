"""Structural analysis: the controllability and observability of a system's linear model."""

from dataclasses import dataclass

import numpy as np

from minilink.analysis.linearization import design_model
from minilink.core.system import System

# Relative singular-value threshold of every rank decision: the Kalman matrices raise
# A to the power n − 1, so their rounding sits far above machine precision
RANK_TOL = 1e-9


###############################################################################
# Public API
###############################################################################


def controllability_matrix(A, B):
    """Return the Kalman controllability matrix ``𝒞 = [B, AB, …, Aⁿ⁻¹B]`` of the pair ``(A, B)``.

    The arrays twin of :func:`controllability`, as ``lqr_gain`` is ``lqr``'s.
    """
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n = A.shape[0]

    # 𝒞 = [B, AB, …, Aⁿ⁻¹B]
    blocks = [B]
    for _ in range(1, n):
        blocks.append(A @ blocks[-1])
    ctrb = np.hstack(blocks)

    return ctrb


def observability_matrix(A, C):
    """Return the Kalman observability matrix ``𝒪 = [C; CA; …; CAⁿ⁻¹]`` of the pair ``(A, C)``.

    The arrays twin of :func:`observability`.
    """
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    n = A.shape[0]

    # 𝒪 = [C; CA; …; CAⁿ⁻¹]
    blocks = [C]
    for _ in range(1, n):
        blocks.append(blocks[-1] @ A)
    obsv = np.vstack(blocks)

    return obsv


def controllability(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    tol=RANK_TOL,
    method="auto",
    eps=1e-6,
):
    """Return the controllability test of ``sys``'s linear model.

    The pair ``(A, B)`` is controllable when its Kalman controllability matrix
    has full row rank ``n``. ``sys`` is an ``LTISystem``, read as is, or any
    plant, linearized at ``(x_bar, u_bar)`` about its command port alone (the
    nominal point by default), as ``lqr`` and ``place`` read it. ``tol`` is
    the relative singular-value threshold below which a direction counts as
    missing, the same as :func:`~minilink.analysis.linear.minreal`'s. On
    arrays, :func:`controllability_matrix` gives ``𝒞``.
    """
    require_system(sys, "controllability", "controllability_matrix(A, B)")

    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A = model.A()
    B = model.B()
    n = A.shape[0]

    # 𝒞 = [B, AB, …, Aⁿ⁻¹B]
    ctrb = controllability_matrix(A, B)

    # Controllable ⇔ rank 𝒞 = n, counting the singular values σᵢ > tol σ₁
    sigma = np.linalg.svd(ctrb, compute_uv=False)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))

    return StructuralResult(matrix=ctrb, rank=r, n=n)


def observability(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    tol=RANK_TOL,
    method="auto",
    eps=1e-6,
):
    """Return the observability test of ``sys``'s linear model.

    The pair ``(A, C)`` is observable when its Kalman observability matrix has
    full column rank ``n``. ``sys`` is an ``LTISystem``, read as is, or any
    plant, linearized at ``(x_bar, u_bar)`` with ``C`` over its measured
    output ``y`` (a plant without ``y`` is refused), as ``luenberger`` and
    ``kalman`` read it; ``tol`` as in :func:`controllability`. On arrays,
    :func:`observability_matrix` gives ``𝒪``.
    """
    require_system(sys, "observability", "observability_matrix(A, C)")

    model = design_model(
        sys, x_bar, u_bar, t, params, method=method, eps=eps, measured=True
    )
    A = model.A()
    C = model.C()
    n = A.shape[0]

    # 𝒪 = [C; CA; …; CAⁿ⁻¹]
    obsv = observability_matrix(A, C)

    # Observable ⇔ rank 𝒪 = n, counting the singular values σᵢ > tol σ₁
    sigma = np.linalg.svd(obsv, compute_uv=False)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))

    return StructuralResult(matrix=obsv, rank=r, n=n)


@dataclass(frozen=True)
class StructuralResult:
    """Result of a controllability or observability test."""

    matrix: np.ndarray  # The stacked Kalman matrix
    rank: int
    n: int  # Number of states

    @property
    def is_full_rank(self) -> bool:
        """The test passes: the Kalman matrix has rank ``n``."""
        return self.rank == self.n

    def __str__(self):
        verdict = "full rank" if self.is_full_rank else "rank deficient"

        return f"StructuralResult: rank {self.rank} of n = {self.n}, {verdict}"


###############################################################################
# Internal machinery
###############################################################################


def require_system(sys, verb, twin):
    """Refuse a non-system argument, naming the arrays twin."""
    if not isinstance(sys, System):
        raise TypeError(
            f"{verb} takes a system, a linear model LTISystem(A, B, C) or a plant, "
            f"not {type(sys).__name__}; on arrays {twin} gives the Kalman matrix"
        )


def rank(matrix, tol=RANK_TOL):
    """The rank of ``matrix``, counting the singular values ``σᵢ > tol σ₁``; the designs' precondition."""
    sigma = np.linalg.svd(matrix, compute_uv=False)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))

    return r


if __name__ == "__main__":
    # Double integrator: controllable from force, observable from position.
    from minilink.dynamics.abstraction.state_space import LTISystem

    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    model = LTISystem(A, B, C)
    print(controllability(model))
    print(observability(model))
    print("𝒞 =\n", controllability_matrix(A, B))
