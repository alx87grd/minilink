"""Generic trajectory-optimization planner orchestration."""

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from minilink.compile.backend_policy import BACKEND_NUMPY
from minilink.core.trajectory import Trajectory
from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
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
    optimizer_method: str = "scipy_slsqp"
    optimizer_options: dict[str, object] = field(default_factory=dict)
    use_hessian: bool = False
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
        options: TrajectoryOptimizationOptions | None = None,
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.transcription = transcription
        self.options = TrajectoryOptimizationOptions() if options is None else options
        self.last_program: MathematicalProgram | None = None
        self.last_optimizer: Optimizer | None = None
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
            compile_backend=compile_backend,
        )
        z0 = self.transcription.pack_initial_guess(self.problem, guess)
        optimizer = self._make_optimizer(program, z0)

        self.iteration_history = []
        optimization_result = optimizer.solve(
            callback=self._make_callback(optimizer, compile_backend),
            record_solve_time=self.options.record_solve_time,
            disp=self.options.solve_disp,
        )
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            compile_backend=compile_backend,
        )

        self.last_program = program
        self.last_optimizer = optimizer
        self.last_optimization_result = optimization_result
        return self._store_result(trajectory)

    def _make_optimizer(self, program: MathematicalProgram, z0: np.ndarray) -> Optimizer:
        program_backend = str(program.metadata.get("program_backend", BACKEND_NUMPY))
        return Optimizer(
            program,
            z0=z0,
            method=self.options.optimizer_method,
            compile_backend=program_backend,
            use_hessian=self.options.use_hessian,
            options=dict(self.options.optimizer_options),
        )

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
        optimizer: Optimizer,
        compile_backend: str | None,
    ) -> OptimizationProgressCallback | None:
        if not self.options.record_history and self.options.callback is None:
            return None

        iteration_index = 0

        def planner_progress(z: np.ndarray, J: float, _t: float) -> None:
            nonlocal iteration_index
            z_arr = np.asarray(z, dtype=float).reshape(-1)
            iteration = self._iteration_from_z(
                optimizer,
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
        optimizer: Optimizer,
        z: np.ndarray,
        compile_backend: str | None,
        iteration_index: int,
        *,
        cost: float | None = None,
    ) -> TrajectoryOptimizationIteration:
        """Build one planning-aware optimizer iteration payload."""
        if cost is None:
            cost = optimizer.program_evaluator.objective(z)
        trajectory = self.transcription.reconstruct_result(
            OptimizationResult(z=z, success=False, cost=cost),
            problem=self.problem,
            compile_backend=compile_backend,
        )
        max_eq, min_ineq = self._constraint_metrics(optimizer.program_evaluator, z)
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
        program_evaluator: MathematicalProgramEvaluator,
        z: np.ndarray,
    ) -> tuple[float, float | None]:
        max_eq = 0.0
        if program_evaluator.n_h:
            max_eq = float(np.max(np.abs(program_evaluator.equality_residual(z))))

        min_ineq = None
        if program_evaluator.n_g:
            min_ineq = float(np.min(program_evaluator.inequality_margin(z)))
        return max_eq, min_ineq
