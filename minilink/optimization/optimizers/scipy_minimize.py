"""
SciPy :func:`scipy.optimize.minimize` optimizer skeleton.

The first optimization pass keeps this as a reviewable contract. Full
SciPy wiring is intentionally deferred until a concrete transcription
needs it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer import Optimizer


@dataclass(frozen=True)
class ScipyMinimizeOptimizer(Optimizer):
    """
    Skeleton for a future :func:`scipy.optimize.minimize` optimizer.
    """

    method: str = "SLSQP"
    options: dict[str, Any] = field(default_factory=dict)

    def solve(self, program: MathematicalProgram) -> OptimizationResult:
        """
        Solve the mathematical program.

        TODO: User Architectural Review - implement SciPy minimize wiring
        after the deterministic planning architecture is reviewed.
        """
        raise NotImplementedError(
            "ScipyMinimizeOptimizer.solve is deferred until architecture review"
        )
