"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory-optimization result
from the optimizer output.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from minilink.optimization.problems import MathematicalProgram, OptimizationResult
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.results import (
    TrajectoryOptimizationResult,
)


class Transcription(ABC):
    """
    Base class for finite-dimensional trajectory transcriptions.
    """

    @abstractmethod
    def transcribe(self, problem: PlanningProblem) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional mathematical program."""
        ...

    @abstractmethod
    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        metadata: dict[str, Any] | None = None,
    ) -> TrajectoryOptimizationResult:
        """Convert an optimizer result back into a planning result."""
        ...
