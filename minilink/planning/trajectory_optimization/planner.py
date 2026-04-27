"""Generic trajectory-optimization planner orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer import Optimizer
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
    TranscriptionContext,
)


@dataclass(frozen=True)
class TrajectoryOptimizationIteration:
    """Planning-aware optimizer callback payload."""

    iteration: int
    z: np.ndarray
    trajectory: Trajectory
    cost: float
    max_eq: float
    min_ineq: float | None


@dataclass(frozen=True)
class TrajectoryOptimizationOptions:
    """Generic trajectory-optimization workflow options."""

    compile_backend: str | None = "numpy"
    initial_guess: np.ndarray | Trajectory | None = None
    warm_start: bool = False
    record_history: bool = False
    callback: Callable[[TrajectoryOptimizationIteration], None] | None = None

    def __post_init__(self) -> None:
        if self.compile_backend is not None:
            object.__setattr__(self, "compile_backend", str(self.compile_backend))
        object.__setattr__(self, "warm_start", bool(self.warm_start))
        object.__setattr__(self, "record_history", bool(self.record_history))


class TrajectoryOptimizationPlanner(Planner):
    """
    Generic trajectory-optimization planner.

    A transcription owns the decision-vector layout and method-specific
    constraints. This planner owns the domain workflow:
    ``problem -> transcription -> mathematical program -> optimizer -> trajectory``.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        transcription: Transcription,
        optimizer: Optimizer | None = None,
        options: TrajectoryOptimizationOptions | None = None,
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.transcription = transcription
        self.optimizer = ScipyMinimizeOptimizer() if optimizer is None else optimizer
        self.options = TrajectoryOptimizationOptions() if options is None else options
        self.last_program: MathematicalProgram | None = None
        self.last_optimization_result: OptimizationResult | None = None
        self.iteration_history: list[TrajectoryOptimizationIteration] = []

    @classmethod
    def from_system(
        cls,
        sys: Any,
        *,
        x_start: np.ndarray,
        x_goal: np.ndarray,
        cost: CostFunction,
        transcription: Transcription,
        optimizer: Optimizer | None = None,
        options: TrajectoryOptimizationOptions | None = None,
    ) -> TrajectoryOptimizationPlanner:
        """Convenience constructor building a :class:`PlanningProblem`."""
        problem = PlanningProblem(
            sys=sys,
            x_start=x_start,
            x_goal=x_goal,
            cost=cost,
        )
        return cls(
            problem,
            transcription=transcription,
            optimizer=optimizer,
            options=options,
        )

    def compute_solution(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
    ) -> Trajectory:
        """Compute and store a trajectory-optimization solution."""
        context = TranscriptionContext(compile_backend=self.options.compile_backend)
        guess = self._resolve_initial_guess(initial_guess, warm_start)
        program = self.transcription.transcribe(
            self.problem,
            initial_guess=guess,
            context=context,
        )

        self.iteration_history = []
        optimization_result = self.optimizer.solve(
            program,
            callback=self._make_callback(program, context),
        )
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            metadata=program.metadata,
            context=context,
        )

        self.last_program = program
        self.last_optimization_result = optimization_result
        return self._store_result(trajectory)

    def _resolve_initial_guess(
        self,
        initial_guess: np.ndarray | Trajectory | None,
        warm_start: bool | None,
    ) -> np.ndarray | Trajectory:
        if initial_guess is not None:
            return initial_guess

        use_warm_start = self.options.warm_start if warm_start is None else warm_start
        if use_warm_start and isinstance(self.last_result, Trajectory):
            return self.last_result

        if self.options.initial_guess is not None:
            return self.options.initial_guess

        t = self.transcription.initial_guess_time_grid(self.problem)
        return default_initial_trajectory(self.problem, t)

    def _make_callback(
        self,
        program: MathematicalProgram,
        context: TranscriptionContext,
    ) -> Callable[[Any], None] | None:
        if not self.options.record_history and self.options.callback is None:
            return None

        iteration_index = 0

        def callback(z) -> None:
            nonlocal iteration_index
            z_arr = np.asarray(getattr(z, "x", z), dtype=float).reshape(-1)
            iteration = self._iteration_from_z(
                program,
                z_arr,
                context,
                iteration_index,
            )
            if self.options.record_history:
                self.iteration_history.append(iteration)
            if self.options.callback is not None:
                self.options.callback(iteration)
            iteration_index += 1

        return callback

    def _iteration_from_z(
        self,
        program: MathematicalProgram,
        z: np.ndarray,
        context: TranscriptionContext,
        iteration_index: int,
    ) -> TrajectoryOptimizationIteration:
        """Build one planning-aware optimizer iteration payload."""
        cost = program.objective(z)
        trajectory = self.transcription.reconstruct_result(
            OptimizationResult(z=z, success=False, cost=cost),
            problem=self.problem,
            metadata=program.metadata,
            context=context,
        )
        max_eq, min_ineq = self._constraint_metrics(program, z)
        return TrajectoryOptimizationIteration(
            iteration=iteration_index,
            z=z.copy(),
            trajectory=trajectory,
            cost=cost,
            max_eq=max_eq,
            min_ineq=min_ineq,
        )

    @staticmethod
    def _constraint_metrics(
        program: MathematicalProgram,
        z: np.ndarray,
    ) -> tuple[float, float | None]:
        if program.equalities:
            max_eq = max(
                float(np.max(np.abs(equality.residual(z))))
                for equality in program.equalities
            )
        else:
            max_eq = 0.0

        min_ineq = None
        if program.inequalities:
            min_ineq = min(
                float(np.min(inequality.margin(z)))
                for inequality in program.inequalities
            )
        return max_eq, min_ineq
