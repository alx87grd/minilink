"""
Backend protocol for finite-dimensional mathematical programs.

Backends consume generic mathematical programs. They do not know about
planning systems, trajectories, or transcription internals.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from minilink.optimization.problems import MathematicalProgram, OptimizationResult


class OptimizationBackend(ABC):
    """
    Base class for finite-dimensional optimization backends.
    """

    @abstractmethod
    def solve(self, program: MathematicalProgram) -> OptimizationResult:
        """Solve a finite-dimensional mathematical program."""
        ...
