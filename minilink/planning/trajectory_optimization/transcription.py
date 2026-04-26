"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory from the optimizer
output.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem


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
    ) -> Trajectory:
        """Convert an optimizer result back into a trajectory."""
        ...
