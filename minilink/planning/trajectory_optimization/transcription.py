"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory from the optimizer
output.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.compile.backend_policy import BACKEND_DIRECT, BACKEND_JAX, BACKEND_NUMPY
from minilink.compile.jax_utils import array_module
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem

ConstraintFunction = Callable[[np.ndarray], np.ndarray]


def transcription_backend_key(compile_backend: str | None) -> str | None:
    """Normalize a transcription compile-backend string.

    ``None`` is preserved because transcriptions use it as an explicit
    "evaluate system.f directly" escape hatch.
    """
    if compile_backend is None:
        return None
    return str(compile_backend).strip().lower()


def program_backend_for_compile(compile_backend: str | None) -> str:
    """Return the mathematical-program evaluator backend for a transcription."""
    return (
        BACKEND_JAX
        if transcription_backend_key(compile_backend) == BACKEND_JAX
        else BACKEND_NUMPY
    )


def uses_direct_dynamics(
    problem: PlanningProblem,
    compile_backend: str | None,
) -> bool:
    """Return true when a transcription should call ``system.f`` directly."""
    key = transcription_backend_key(compile_backend)
    return problem.params.system is not None or key is None or key == BACKEND_DIRECT


def native_concatenate(values, like):
    """Concatenate vector pieces with the array module used by ``like``."""
    xp = array_module(like)
    pieces = []
    for value in values:
        pieces.append(value.reshape(-1))

    if not pieces:
        return xp.array([])

    return xp.concatenate(pieces)


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
    key = transcription_backend_key(compile_backend)
    if uses_direct_dynamics(problem, compile_backend):
        return lambda x, u, t: problem.sys.f(x, u, t, params)

    evaluator = problem.sys.compile(backend=key, verbose=False)
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
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional mathematical program."""
        ...

    @abstractmethod
    def pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        """Pack a trajectory or array guess into the transcription decision vector."""
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
    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the time grid used by generic trajectory guesses."""
        ...


def stack_constraints(
    constraints: list[ConstraintFunction],
) -> ConstraintFunction | None:
    """Return one native-array constraint vector from a list of vector functions."""
    if not constraints:
        return None

    def stacked(z: np.ndarray) -> np.ndarray:
        xp = array_module(z)
        values = []
        for constraint in constraints:
            values.append(constraint(z).reshape(-1))
        return xp.concatenate(values)

    return stacked
