"""Generic trajectory-optimization planner orchestration."""

import time
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

# Human-readable ``solve_disp=True`` panel.
_DISP_LINE_WIDTH = 60
_DISP_RULE_MAIN = "=" * _DISP_LINE_WIDTH
_DISP_RULE_DIV = "-" * _DISP_LINE_WIDTH


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

    ``solve_disp`` prints a Minilink trajectory-optimization preamble and
    report. It is separate from SciPy's ``options['disp']``.
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
        workflow_t0 = time.perf_counter()
        compile_backend = self.options.compile_backend
        guess = self._resolve_initial_guess(initial_guess, warm_start)

        transcribe_t0 = time.perf_counter()
        program = self.transcription.transcribe(
            self.problem,
            compile_backend=compile_backend,
        )
        transcribe_s = time.perf_counter() - transcribe_t0
        z0 = self.transcription.pack_initial_guess(self.problem, guess)

        compile_t0 = time.perf_counter()
        optimizer = self._make_optimizer(program, z0)
        compile_s = time.perf_counter() - compile_t0

        if self.options.solve_disp:
            self._print_solve_preamble(
                program=program,
                optimizer=optimizer,
                z0=z0,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
            )

        self.iteration_history = []
        optimization_result = optimizer.solve(
            callback=self._make_callback(optimizer, compile_backend),
            record_solve_time=self.options.record_solve_time
            or self.options.solve_disp,
            disp=False,
        )
        reconstruct_t0 = time.perf_counter()
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            compile_backend=compile_backend,
        )
        reconstruct_s = time.perf_counter() - reconstruct_t0
        total_s = time.perf_counter() - workflow_t0

        self.last_program = program
        self.last_optimizer = optimizer
        self.last_optimization_result = optimization_result
        stored = self._store_result(trajectory)

        if self.options.solve_disp:
            self._print_solve_report(
                optimizer=optimizer,
                result=optimization_result,
                trajectory=stored,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
                reconstruct_s=reconstruct_s,
                total_s=total_s,
            )

        return stored

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

    def _print_solve_preamble(
        self,
        *,
        program: MathematicalProgram,
        optimizer: Optimizer,
        z0: np.ndarray,
        transcribe_s: float,
        compile_s: float,
    ) -> None:
        problem = self.problem
        sys = problem.sys
        print()
        print(_DISP_RULE_MAIN)
        print("===          Trajectory Optimization Program             ===")
        print(_DISP_RULE_MAIN)
        print("system:", getattr(sys, "name", type(sys).__name__))
        print(f"dimensions: n={int(sys.n)}, m={int(sys.m)}, p={int(sys.p)}")
        print("x_start:", self._preview_vector(problem.x_start))
        print("x_goal:", self._preview_vector(problem.x_goal))
        print("X:", self._class_name(problem.X))
        print("U:", self._class_name(problem.U))
        print("X0:", self._class_name(problem.X0))
        print("Xf:", self._class_name(problem.Xf))
        print("cost:", self._class_name(problem.cost))
        print(_DISP_RULE_DIV)
        print("transcription:", type(self.transcription).__name__)
        print("transcription_options:", self._transcription_options())
        print(f"compile_backend={self.options.compile_backend!r}")
        print(f"program_backend={optimizer.program_evaluator.backend!r}")
        print("n_z:", int(program.n_z))
        print(
            f"constraints: n_h={optimizer.program_evaluator.n_h}, "
            f"n_g={optimizer.program_evaluator.n_g}"
        )
        print("z0:", self._preview_vector(z0))
        print(_DISP_RULE_DIV)
        print(f"method={self.options.optimizer_method!r}")
        print("options:", getattr(optimizer.backend, "options", {}))
        print(f"setup: transcribe={transcribe_s:.6g}s compile={compile_s:.6g}s")
        print(_DISP_RULE_DIV)
        print("Running trajectory optimization...")

    def _print_solve_report(
        self,
        *,
        optimizer: Optimizer,
        result: OptimizationResult,
        trajectory: Trajectory,
        transcribe_s: float,
        compile_s: float,
        reconstruct_s: float,
        total_s: float,
    ) -> None:
        max_eq, min_ineq = self._constraint_metrics(
            optimizer.program_evaluator,
            result.z,
        )
        max_bound = self._bound_violation(optimizer.program, result.z)

        print("Completed in", result.solve_time_s, "seconds")
        print(_DISP_RULE_DIV)
        print("success:", result.success)
        print("message:", result.message)
        print("J*:", result.cost)
        print("stats:", result.stats)
        print("max_eq:", max_eq)
        print("min_ineq:", min_ineq)
        print("max_bound:", max_bound)
        print("x(0):", self._preview_vector(trajectory.x[:, 0]))
        print("x(tf):", self._preview_vector(trajectory.x[:, -1]))
        if self.problem.x_goal is not None:
            terminal_error = trajectory.x[:, -1] - self.problem.x_goal
            print("terminal_error_inf:", float(np.max(np.abs(terminal_error))))
        print(
            "timing:",
            {
                "transcribe_s": transcribe_s,
                "compile_s": compile_s,
                "solve_s": result.solve_time_s,
                "reconstruct_s": reconstruct_s,
                "total_s": total_s,
            },
        )
        print(_DISP_RULE_MAIN)

    def _transcription_options(self) -> dict[str, object]:
        options = getattr(self.transcription, "options", None)
        if options is None:
            return {}
        return dict(vars(options))

    @staticmethod
    def _class_name(value: object | None) -> str | None:
        if value is None:
            return None
        return type(value).__name__

    @staticmethod
    def _preview_vector(value) -> str | None:
        if value is None:
            return None
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.size <= 8:
            return np.array2string(arr, precision=6, max_line_width=96)
        head = np.array2string(arr[:4], precision=6, max_line_width=96)
        return f"{head} ... ({arr.size} values)"

    @staticmethod
    def _bound_violation(program: MathematicalProgram, z: np.ndarray) -> float:
        max_bound = 0.0
        if program.lower is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(program.lower - z, 0.0))),
            )
        if program.upper is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(z - program.upper, 0.0))),
            )
        return max_bound
