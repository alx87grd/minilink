"""
Abstract optimizer for finite-dimensional mathematical programs.

Optimizers consume :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
instances. They do not know about planning systems, trajectories, or
transcription internals.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)


class Optimizer(ABC):
    """
    Base class for finite-dimensional optimizers (mathematical-program solvers).
    """

    @abstractmethod
    def solve(self, program: MathematicalProgram) -> OptimizationResult:
        """Solve a finite-dimensional mathematical program."""
        ...
