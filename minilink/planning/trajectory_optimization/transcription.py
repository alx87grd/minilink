"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory from the optimizer
output.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem


@dataclass
class FixedGridOptions:
    """Uniform fixed-time-grid options."""

    tf: float
    n_steps: int

    def __post_init__(self) -> None:
        tf = float(self.tf)
        if not np.isfinite(tf) or tf <= 0.0:
            raise ValueError("tf must be positive and finite")
        if self.n_steps < 2:
            raise ValueError("n_steps must be at least 2")
        self.tf = tf
        self.n_steps = int(self.n_steps)

    @property
    def t(self) -> np.ndarray:
        return np.linspace(0.0, self.tf, self.n_steps)

    @property
    def dt(self) -> float:
        return self.tf / (self.n_steps - 1)


def dynamics_function(
    problem: PlanningProblem,
    compile_backend: str | None,
):
    """Return ``(x, u, t) -> f(x, u, t)`` for a transcription."""
    params = problem.params.system
    if params is not None or compile_backend is None or compile_backend == "direct":
        return lambda x, u, t: problem.sys.f(x, u, t, params)

    evaluator = problem.sys.compile(backend=compile_backend, verbose=False)
    return lambda x, u, t: evaluator.f(x, u, t)


class Transcription(ABC):
    """
    Base class for finite-dimensional trajectory transcriptions.
    """

    @abstractmethod
    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess=None,
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional mathematical program."""
        ...

    @abstractmethod
    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str | None = "numpy",
    ) -> Trajectory:
        """Convert an optimizer result back into a trajectory."""
        ...

    @abstractmethod
    def initial_guess_time_grid(self, problem: PlanningProblem):
        """Return the time grid used by generic trajectory guesses."""
        ...
