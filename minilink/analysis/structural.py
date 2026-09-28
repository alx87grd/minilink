"""Structural analysis of linear systems: controllability and observability."""

from dataclasses import dataclass

import numpy as np

# Relative singular-value threshold of every rank decision: the Kalman matrices raise
# A to the power n − 1, so their rounding sits far above machine precision
RANK_TOL = 1e-9


@dataclass(frozen=True)
class StructuralResult:
    """Result of a controllability or observability test."""

    matrix: np.ndarray  # the stacked Kalman matrix
    rank: int
    n: int  # number of states

    @property
    def is_full_rank(self) -> bool:
        return self.rank == self.n


def controllability(A, B=None, *, tol=RANK_TOL):
    """Return the controllability test for the pair ``(A, B)``.

    The pair is controllable when its Kalman controllability matrix has full
    row rank ``n``. Pass the two matrices or one ``LTISystem``
    (``controllability(plant.linearize(x_bar))``). ``tol`` is the relative
    singular-value threshold below which a direction counts as missing, the
    same as :func:`~minilink.analysis.linear.minreal`'s.
    """
    A, B = matrix_pair(A, B, "B")
    n = A.shape[0]

    # 𝒞 = [B, AB, …, Aⁿ⁻¹B]
    blocks = [B]
    for _ in range(1, n):
        blocks.append(A @ blocks[-1])
    ctrb = np.hstack(blocks)

    # Controllable ⇔ rank 𝒞 = n, counting the singular values σᵢ > tol σ₁
    sigma = np.linalg.svd(ctrb, compute_uv=False)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))

    return StructuralResult(matrix=ctrb, rank=r, n=n)


def observability(A, C=None, *, tol=RANK_TOL):
    """Return the observability test for the pair ``(A, C)``.

    The pair is observable when its Kalman observability matrix has full
    column rank ``n``. Pass the two matrices or one ``LTISystem``; ``tol`` as
    in :func:`controllability`.
    """
    A, C = matrix_pair(A, C, "C")
    n = A.shape[0]

    # 𝒪 = [C; CA; …; CAⁿ⁻¹]
    blocks = [C]
    for _ in range(1, n):
        blocks.append(blocks[-1] @ A)
    obsv = np.vstack(blocks)

    # Observable ⇔ rank 𝒪 = n, counting the singular values σᵢ > tol σ₁
    sigma = np.linalg.svd(obsv, compute_uv=False)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))

    return StructuralResult(matrix=obsv, rank=r, n=n)


# =============================================================================
# Internal machinery
# =============================================================================


def matrix_pair(A, M, second):
    """``(A, B)`` or ``(A, C)`` as float arrays, from the two matrices or from one ``LTISystem``."""
    if M is None:
        if not all(callable(getattr(A, name, None)) for name in ("A", second)):
            raise TypeError(
                f"pass the two matrices (A, {second}) or one LTISystem; got {type(A).__name__}"
            )
        A, M = A.A(), getattr(A, second)()
    return np.asarray(A, dtype=float), np.atleast_2d(np.asarray(M, dtype=float))


if __name__ == "__main__":
    # Double integrator: controllable from force, observable from position.
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    ctrb = controllability(A, B)
    obsv = observability(A, C)
    print("controllable:", ctrb.is_full_rank, "rank", ctrb.rank)
    print("observable:  ", obsv.is_full_rank, "rank", obsv.rank)
