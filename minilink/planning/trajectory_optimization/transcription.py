"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory from the optimizer
output.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem


@dataclass(frozen=True)
class TranscriptionContext:
    """Execution context shared by a trajopt planner and transcription."""

    compile_backend: str | None = "numpy"

    def __post_init__(self) -> None:
        if self.compile_backend is not None:
            object.__setattr__(self, "compile_backend", str(self.compile_backend))


class Transcription(ABC):
    """
    Base class for finite-dimensional trajectory transcriptions.
    """

    @abstractmethod
    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: Any | None = None,
        context: TranscriptionContext | None = None,
    ) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional mathematical program."""
        ...

    @abstractmethod
    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        metadata: dict[str, Any] | None = None,
        context: TranscriptionContext | None = None,
    ) -> Trajectory:
        """Convert an optimizer result back into a trajectory."""
        ...

    @abstractmethod
    def initial_guess_time_grid(self, problem: PlanningProblem) -> Any:
        """Return the time grid used by generic trajectory guesses."""
        ...
