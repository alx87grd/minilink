"""Structural analysis of linear systems: controllability and observability.

These verbs take the constant matrices of a linear model (an
:class:`~minilink.dynamics.abstraction.state_space.LTISystem` or raw arrays)
and return the Kalman controllability/observability matrices, their rank, and
the boolean verdict. Use :func:`minilink.analysis.linearize.linearize` first to
get an ``LTISystem`` from a nonlinear plant.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StructuralResult:
    """Result of a controllability or observability test."""

    matrix: np.ndarray  # the stacked Kalman matrix
    rank: int
    n: int  # number of states

    @property
    def is_full_rank(self) -> bool:
        return self.rank == self.n


def controllability(A, B=None):
    """Return the controllability test for ``dx = A x + B u``.

    The Kalman matrix is ``[B, AB, A²B, …, Aⁿ⁻¹B]``; the pair is controllable
    when it has full row rank ``n``. Pass the two matrices or one
    ``LTISystem`` (``controllability(plant.linearize(x_bar))``).
    """
    if B is None:
        A, B = _lti_matrices(A, "B")
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n = A.shape[0]

    # 𝒞 = [B, AB, …, A^{n-1} B]
    blocks = [B]
    for _ in range(1, n):
        blocks.append(A @ blocks[-1])
    ctrb = np.hstack(blocks)

    return StructuralResult(matrix=ctrb, rank=int(np.linalg.matrix_rank(ctrb)), n=n)


def observability(A, C=None):
    """Return the observability test for ``dx = A x``, ``y = C x``.

    The Kalman matrix is ``[C; CA; CA²; …; CAⁿ⁻¹]``; the pair is observable
    when it has full column rank ``n``. Pass the two matrices or one
    ``LTISystem``.
    """
    if C is None:
        A, C = _lti_matrices(A, "C")
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    n = A.shape[0]

    # 𝒪 = [C; CA; …; CA^{n-1}]
    blocks = [C]
    for _ in range(1, n):
        blocks.append(blocks[-1] @ A)
    obsv = np.vstack(blocks)

    return StructuralResult(matrix=obsv, rank=int(np.linalg.matrix_rank(obsv)), n=n)


def _lti_matrices(lti, second):
    """``(A, B)`` or ``(A, C)`` of an ``LTISystem`` passed as the only argument."""
    if not all(callable(getattr(lti, name, None)) for name in ("A", second)):
        raise TypeError(
            f"pass the two matrices (A, {second}) or one LTISystem; got {type(lti).__name__}"
        )
    return lti.A(), getattr(lti, second)()


if __name__ == "__main__":
    # Double integrator: controllable from force, observable from position.
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    ctrb = controllability(A, B)
    obsv = observability(A, C)
    print("controllable:", ctrb.is_full_rank, "rank", ctrb.rank)
    print("observable:  ", obsv.is_full_rank, "rank", obsv.rank)
