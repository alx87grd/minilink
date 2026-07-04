"""Beta compile-once MPC planner for receding-horizon direct collocation."""

from __future__ import annotations

import time
from dataclasses import replace

import numpy as np

from minilink.core.backends import BACKEND_JAX, normalize_backend
from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.mpc.options import MPCOptions
from minilink.planning.mpc.parametric_evaluator import JaxParametricProgramEvaluator
from minilink.planning.mpc.transcription import MPCDirectCollocationTranscription
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem


class MPCPlanner(Planner):
    """
    Beta receding-horizon planner with one-time JAX compilation.

    Parameters
    ----------
    problem : PlanningProblem
        Template problem frozen at construction (cost, sets, reference ``x_start``).
        Only ``x_start`` varies at runtime via :meth:`step`.
    transcription : MPCDirectCollocationTranscription
        Parametric direct-collocation transcription.
    options : MPCOptions, optional
        Solver and reporting options.
    """

    _USER_OPTIMIZER_METHODS = {
        "scipy_slsqp": (
            "scipy_minimize",
            {"scipy_method": "SLSQP", "options": {"disp": False}},
        ),
    }

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        transcription: MPCDirectCollocationTranscription,
        options: MPCOptions | None = None,
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.transcription = transcription
        self.options = MPCOptions() if options is None else options
        self.program = None
        self.program_evaluator: JaxParametricProgramEvaluator | None = None
        self.optimizer_backend: ScipyMinimizeOptimizer | None = None
        self._dynamics = None
        self.compile_time_s: float | None = None
        self.last_solve_time_s: float | None = None
        self.last_step_time_s: float | None = None
        self.last_optimization_result: OptimizationResult | None = None
        self.prepare()

    def prepare(self) -> None:
        """Transcribe and JIT-compile the parametric NLP once."""
        compile_backend = normalize_backend(self.options.compile_backend)
        if compile_backend != BACKEND_JAX:
            raise ValueError(
                "Beta MPCPlanner currently supports compile_backend='jax' only."
            )

        t0 = time.perf_counter()
        self.program = self.transcription.transcribe_parametric(
            self.problem,
            compile_backend=compile_backend,
        )

        guess = default_initial_trajectory(
            self.problem,
            self.transcription.initial_guess_time_grid(self.problem),
        )
        z0 = self.transcription.pack_initial_guess(self.problem, guess)

        self.program_evaluator = JaxParametricProgramEvaluator(
            self.program,
            sample_z=z0,
            sample_x0=self.problem.x_start,
        )
        self.program_evaluator.bind(self.problem.x_start)

        params = self.problem.params.system
        evaluator = self.problem.sys.compile(backend=compile_backend, verbose=False)
        if params is None:
            self._dynamics = evaluator.f
        else:
            self._dynamics = lambda x, u, t: evaluator.f_p(x, u, t, params)

        self.optimizer_backend = self._make_optimizer_backend()
        self.compile_time_s = time.perf_counter() - t0

    def step(
        self,
        x_start,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> Trajectory:
        """Solve one MPC tick for the measured initial state ``x_start``."""
        if self.program_evaluator is None or self.optimizer_backend is None:
            raise RuntimeError("MPCPlanner.prepare() must run before step().")

        step_t0 = time.perf_counter()
        x_arr = np.asarray(x_start, dtype=float).reshape(-1)
        n = int(self.problem.sys.n)
        if x_arr.shape != (n,):
            raise ValueError(f"x_start must have shape ({n},)")

        problem_k = replace(
            self.problem,
            x_start=x_arr,
            X0=SingletonSet(x_arr),
        )
        self.program_evaluator.bind(x_arr)

        if initial_guess is None:
            initial_guess = default_initial_trajectory(
                problem_k,
                self.transcription.initial_guess_time_grid(problem_k),
            )
        z0 = self.transcription.pack_initial_guess(problem_k, initial_guess)

        record_solve_time = self.options.record_solve_time or self.options.step_disp
        if record_solve_time:
            solve_t0 = time.perf_counter()

        result = self.optimizer_backend.solve(
            self.program_evaluator,
            z0,
        )

        if record_solve_time:
            self.last_solve_time_s = time.perf_counter() - solve_t0
            result = OptimizationResult(
                z=result.z,
                success=result.success,
                cost=result.cost,
                message=result.message,
                stats=result.stats,
                solve_time_s=self.last_solve_time_s,
            )
        else:
            self.last_solve_time_s = None

        trajectory = self.transcription.reconstruct_result(
            result,
            problem=problem_k,
            dynamics=self._dynamics,
        )
        self.last_step_time_s = time.perf_counter() - step_t0
        self.last_optimization_result = result
        stored = self._store_result(trajectory)

        if self.options.step_disp:
            print(
                f"MPC step: success={result.success} "
                f"solve={self.last_solve_time_s:.6g}s "
                f"step={self.last_step_time_s:.6g}s"
            )

        return stored

    def compute_solution(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> Trajectory:
        """Single-shot solve at the template problem's ``x_start``."""
        return self.step(self.problem.x_start, initial_guess=initial_guess)

    def _make_optimizer_backend(self) -> ScipyMinimizeOptimizer:
        method = self.options.optimizer_method
        if method not in self._USER_OPTIMIZER_METHODS:
            valid = ", ".join(sorted(self._USER_OPTIMIZER_METHODS))
            raise ValueError(
                f"Unknown optimizer method {method!r}. Expected one of: {valid}."
            )

        _, preset = self._USER_OPTIMIZER_METHODS[method]
        kwargs = {}
        for key, value in preset.items():
            kwargs[key] = dict(value) if isinstance(value, dict) else value
        existing = kwargs.get("options", {})
        kwargs["options"] = {**existing, **dict(self.options.optimizer_options)}
        return ScipyMinimizeOptimizer(**kwargs)
