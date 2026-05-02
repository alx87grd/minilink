"""Generic trajectory-optimization planner orchestration."""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.compile.backend_policy import BACKEND_NUMPY
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import (
    OptimizationProgressCallback,
    Optimizer,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
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


@dataclass
class TrajectoryOptimizationOptions:
    """Generic trajectory-optimization workflow options.

    ``solve_disp`` maps to :meth:`~minilink.optimization.optimizer.Optimizer.solve`
    ``disp=…`` (Minilink text report, not SciPy's ``options['disp']``).
    """

    compile_backend: str | None = BACKEND_NUMPY
    initial_guess: np.ndarray | Trajectory | None = None
    warm_start: bool = False
    record_history: bool = False
    callback: Callable[[TrajectoryOptimizationIteration], None] | None = None
    record_solve_time: bool = False
    solve_disp: bool = False


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
        self.optimizer = Optimizer() if optimizer is None else optimizer
        self.options = TrajectoryOptimizationOptions() if options is None else options
        self.last_program: MathematicalProgram | None = None
        self.last_optimization_result: OptimizationResult | None = None
        self.iteration_history: list[TrajectoryOptimizationIteration] = []

    def compute_solution(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
    ) -> Trajectory:
        """Compute and store a trajectory-optimization solution."""
        compile_backend = self.options.compile_backend
        guess = self._resolve_initial_guess(initial_guess, warm_start)
        program = self.transcription.transcribe(
            self.problem,
            initial_guess=guess,
            compile_backend=compile_backend,
        )

        self.iteration_history = []
        optimization_result = self.optimizer.solve(
            program,
            callback=self._make_callback(program, compile_backend),
            record_solve_time=self.options.record_solve_time,
            disp=self.options.solve_disp,
        )
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            compile_backend=compile_backend,
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
        compile_backend: str | None,
    ) -> OptimizationProgressCallback | None:
        if not self.options.record_history and self.options.callback is None:
            return None

        iteration_index = 0

        def planner_progress(z: np.ndarray, J: float, _t: float) -> None:
            nonlocal iteration_index
            z_arr = np.asarray(z, dtype=float).reshape(-1)
            iteration = self._iteration_from_z(
                program,
                z_arr,
                compile_backend,
                iteration_index,
                cost=J,
            )
            if self.options.record_history:
                self.iteration_history.append(iteration)
            if self.options.callback is not None:
                self.options.callback(iteration)
            iteration_index += 1

        return planner_progress

    def _iteration_from_z(
        self,
        program: MathematicalProgram,
        z: np.ndarray,
        compile_backend: str | None,
        iteration_index: int,
        *,
        cost: float | None = None,
    ) -> TrajectoryOptimizationIteration:
        """Build one planning-aware optimizer iteration payload."""
        if cost is None:
            cost = program.objective(z)
        trajectory = self.transcription.reconstruct_result(
            OptimizationResult(z=z, success=False, cost=cost),
            problem=self.problem,
            compile_backend=compile_backend,
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
