"""Generic trajectory-optimization planner orchestration."""

from __future__ import annotations

import time
import warnings
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, normalize_backend
from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import (
    OptimizationProgressCallback,
    Optimizer,
)
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import SolveMetadata, TrajectoryPlan
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
)

_UNSET = object()

_TRAJOPT_OPTION_KEYS = (
    "compile_backend",
    "initial_guess",
    "warm_start",
    "optimizer_method",
    "optimizer_options",
    "use_hessian",
    "record_history",
    "callback",
    "record_solve_time",
    "solve_disp",
    "step_disp",
)

_TRANSCRIPTION_PRESETS = frozenset({"direct_collocation", "multiple_shooting"})


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

    ``step_disp`` prints per-tick timing on the parametric / from-solve path.

    ``record_history=True`` reconstructs a full :class:`Trajectory` per
    optimizer iterate — convenient for live plots and teaching, but it adds
    one rollout/unpack per iteration; leave it off for production solves.

    Parametric compile (:meth:`TrajectoryOptimizationPlanner.compile_parametric_program`)
    currently requires ``compile_backend='jax'``.
    """

    compile_backend: str = BACKEND_NUMPY
    initial_guess: np.ndarray | Trajectory | None = None
    warm_start: bool = False
    optimizer_method: str = "scipy_slsqp"
    optimizer_options: dict[str, object] = field(default_factory=dict)
    use_hessian: bool = False
    record_history: bool = False
    callback: Callable[[TrajectoryOptimizationIteration], None] | None = None
    record_solve_time: bool = False
    solve_disp: bool = False
    step_disp: bool = False


class TrajectoryOptimizationPlanner(Planner):
    """
    Generic trajectory-optimization planner.

    A transcription owns the decision-vector layout and method-specific
    constraints. This planner owns the domain workflow:
    ``problem -> transcription -> mathematical program -> optimizer -> trajectory``.

    Offline :meth:`solve_trajectory` always rebuilds a fresh
    :class:`~minilink.optimization.mathematical_program.MathematicalProgram`.
    Online :meth:`solve_trajectory_from` rebuilds with the measured ``x0``
    unless :meth:`compile_parametric_program` has been called, in which case
    each from-solve is bind + solve only.
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
        transcription: Transcription | str | None = None,
        n_steps: int | None = None,
        options: TrajectoryOptimizationOptions | None = None,
        compile_backend=_UNSET,
        initial_guess=_UNSET,
        warm_start=_UNSET,
        optimizer_method=_UNSET,
        optimizer_options=_UNSET,
        use_hessian=_UNSET,
        record_history=_UNSET,
        callback=_UNSET,
        record_solve_time=_UNSET,
        solve_disp=_UNSET,
        step_disp=_UNSET,
    ) -> None:
        """
        Parameters
        ----------
        problem : PlanningProblem
            Continuous-time planning task (requires a cost).
        transcription : Transcription or str, optional
            Discretization method. ``None`` defaults to direct collocation.
            String presets: ``\"direct_collocation\"``, ``\"multiple_shooting\"``.
            Teach demos should pass the string explicitly.
        n_steps : int, optional
            Knot count when ``transcription`` is omitted or a string preset.
            Required in those cases; ignored when a ``Transcription`` instance
            already carries its grid (must not conflict if also passed).
        options : TrajectoryOptimizationOptions, optional
            Tier-2 workflow bag. Flat kwargs below overlay matching fields.
        compile_backend, initial_guess, warm_start, optimizer_method,
        optimizer_options, use_hessian, record_history, callback,
        record_solve_time, solve_disp, step_disp
            Tier-1 flat mirrors of :class:`TrajectoryOptimizationOptions`.
        """
        super().__init__(problem)
        self.require_cost()
        self.transcription = _resolve_transcription(transcription, n_steps)
        self.options = _merge_trajopt_options(
            options,
            compile_backend=compile_backend,
            initial_guess=initial_guess,
            warm_start=warm_start,
            optimizer_method=optimizer_method,
            optimizer_options=optimizer_options,
            use_hessian=use_hessian,
            record_history=record_history,
            callback=callback,
            record_solve_time=record_solve_time,
            solve_disp=solve_disp,
            step_disp=step_disp,
        )
        self.last_program: MathematicalProgram | None = None
        self.last_optimizer: Optimizer | None = None
        self.last_optimization_result: OptimizationResult | None = None
        self.iteration_history: list[TrajectoryOptimizationIteration] = []
        self.program = None
        self.program_evaluator = None
        self.optimizer_backend: ScipyMinimizeOptimizer | None = None
        self._dynamics = None
        self.compile_time_s: float | None = None
        self.last_solve_time_s: float | None = None
        self.last_step_time_s: float | None = None
        self._warned_uncompiled_from = False

    @property
    def has_parametric_program(self) -> bool:
        """True after a successful :meth:`compile_parametric_program`."""
        return self.program_evaluator is not None and self.optimizer_backend is not None

    def solve(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
    ) -> TrajectoryPlan:
        """Offline trajopt entry (``solve_trajectory``)."""
        return self.solve_trajectory(initial_guess=initial_guess, warm_start=warm_start)

    def solve_trajectory(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
    ) -> TrajectoryPlan:
        """Compute and store a trajectory-optimization solution (always rebuild)."""
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
            record_solve_time=self.options.record_solve_time or self.options.solve_disp,
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
        plan = self._store_trajectory_plan(
            TrajectoryPlan(
                trajectory=trajectory,
                metadata=SolveMetadata(
                    success=bool(optimization_result.success),
                    message=str(optimization_result.message),
                    cost=optimization_result.cost,
                    solve_time_s=optimization_result.solve_time_s,
                    stats=dict(optimization_result.stats),
                ),
                warm_state=optimization_result.z,
            )
        )

        if self.options.solve_disp:
            self._print_solve_report(
                optimizer=optimizer,
                result=optimization_result,
                trajectory=plan.trajectory,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
                reconstruct_s=reconstruct_s,
                total_s=total_s,
            )

        return plan

    def compile_parametric_program(self) -> None:
        """Build and JIT-compile a parametric NLP once (idempotent).

        Requires ``options.compile_backend='jax'`` and a transcription that
        implements :meth:`~Transcription.transcribe_parametric` (direct
        collocation).
        """
        if self.has_parametric_program:
            return

        compile_backend = normalize_backend(
            self.options.compile_backend, allow_direct=True
        )
        if compile_backend != BACKEND_JAX:
            raise ValueError(
                "compile_parametric_program requires compile_backend='jax' "
                f"(got {self.options.compile_backend!r})."
            )
        if not hasattr(self.transcription, "transcribe_parametric"):
            raise TypeError(
                f"{type(self.transcription).__name__} does not support "
                "transcribe_parametric; use DirectCollocationTranscription."
            )

        from minilink.planning.trajectory_optimization.parametric_evaluator import (
            JaxParametricProgramEvaluator,
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

        self.optimizer_backend = self._make_parametric_optimizer_backend()
        self.compile_time_s = time.perf_counter() - t0

    def solve_trajectory_from(
        self,
        x0,
        *,
        params=None,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> TrajectoryPlan:
        """
        Online traj-family solve from measured ``x0``.

        Parameters
        ----------
        x0 : array_like
            Measured initial state.
        params : mapping, optional
            Scene / bind façade. ``None`` or ``{}`` binds ``x0`` only.
            Non-empty keys are rejected until pipeline B scene bind.
        initial_guess : ndarray or Trajectory, optional
            NLP seed (schedule and/or packed ``z``). Warm-start *policy*
            (reuse last latch) is owned by ``ModelPredictiveController``, not
            this method.

        Notes
        -----
        Fast bind path when :attr:`has_parametric_program`; otherwise rebuilds
        the NLP with ``x_start`` replaced (one-time warning suggesting
        :meth:`compile_parametric_program`).
        """
        reject_unknown_online_params(params)
        if self.has_parametric_program:
            return self._solve_trajectory_from_parametric(
                x0, initial_guess=initial_guess
            )
        return self._solve_trajectory_from_rebuild(x0, initial_guess=initial_guess)

    def step(
        self,
        x0,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> Trajectory:
        """Solve one from-tick and return the bare schedule (hand loops)."""
        return self.solve_trajectory_from(x0, initial_guess=initial_guess).trajectory

    def _solve_trajectory_from_parametric(
        self,
        x0,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> TrajectoryPlan:
        if self.program_evaluator is None or self.optimizer_backend is None:
            raise RuntimeError(
                "compile_parametric_program() must run before parametric from-solve."
            )

        step_t0 = time.perf_counter()
        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        n = int(self.problem.sys.n)
        if x_arr.shape != (n,):
            raise ValueError(f"x0 must have shape ({n},)")

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
        plan = self._store_trajectory_plan(
            TrajectoryPlan(
                trajectory=trajectory,
                metadata=SolveMetadata(
                    success=bool(result.success),
                    message=str(result.message),
                    cost=result.cost,
                    solve_time_s=result.solve_time_s,
                    stats=dict(result.stats),
                ),
                warm_state=result.z,
            )
        )

        if self.options.step_disp:
            print(
                f"TOP step: success={result.success} "
                f"solve={self.last_solve_time_s:.6g}s "
                f"step={self.last_step_time_s:.6g}s"
            )

        return plan

    def _solve_trajectory_from_rebuild(
        self,
        x0,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> TrajectoryPlan:
        if not self._warned_uncompiled_from:
            backend = normalize_backend(self.options.compile_backend, allow_direct=True)
            if backend == BACKEND_JAX:
                warnings.warn(
                    "TrajectoryOptimizationPlanner.solve_trajectory_from is rebuilding "
                    "the NLP each call. Call compile_parametric_program() once for "
                    "fast bind-only from-solves (requires compile_backend='jax').",
                    UserWarning,
                    stacklevel=3,
                )
            self._warned_uncompiled_from = True

        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        n = int(self.problem.sys.n)
        if x_arr.shape != (n,):
            raise ValueError(f"x0 must have shape ({n},)")

        saved_problem = self.problem
        self.problem = replace(
            saved_problem,
            x_start=x_arr,
            X0=SingletonSet(x_arr),
        )
        try:
            return self.solve_trajectory(initial_guess=initial_guess, warm_start=False)
        finally:
            self.problem = saved_problem

    def _make_optimizer(
        self, program: MathematicalProgram, z0: np.ndarray
    ) -> Optimizer:
        return Optimizer(
            program,
            z0=z0,
            method=self.options.optimizer_method,
            use_hessian=self.options.use_hessian,
            options=dict(self.options.optimizer_options),
        )

    def _make_parametric_optimizer_backend(self) -> ScipyMinimizeOptimizer:
        method = self.options.optimizer_method
        if method not in self._USER_OPTIMIZER_METHODS:
            valid = ", ".join(sorted(self._USER_OPTIMIZER_METHODS))
            raise ValueError(
                f"Unknown optimizer method {method!r} for parametric compile. "
                f"Expected one of: {valid}."
            )

        _, preset = self._USER_OPTIMIZER_METHODS[method]
        kwargs = {}
        for key, value in preset.items():
            kwargs[key] = dict(value) if isinstance(value, dict) else value
        existing = kwargs.get("options", {})
        kwargs["options"] = {**existing, **dict(self.options.optimizer_options)}
        return ScipyMinimizeOptimizer(**kwargs)

    def _resolve_initial_guess(
        self,
        initial_guess: np.ndarray | Trajectory | None,
        warm_start: bool | None,
    ) -> np.ndarray | Trajectory:
        if initial_guess is not None:
            return initial_guess

        use_warm_start = self.options.warm_start if warm_start is None else warm_start
        if use_warm_start and self.last_trajectory_plan is not None:
            return self.last_trajectory_plan.trajectory

        if self.options.initial_guess is not None:
            return self.options.initial_guess

        t = self.transcription.initial_guess_time_grid(self.problem)
        return default_initial_trajectory(self.problem, t)

    def _make_callback(
        self,
        optimizer: Optimizer,
        compile_backend: str,
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
        compile_backend: str,
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
        max_eq, min_ineq, _ = optimizer.program_evaluator.constraint_violations(z)
        return TrajectoryOptimizationIteration(
            iteration=iteration_index,
            z=z.copy(),
            trajectory=trajectory,
            cost=cost,
            max_eq=max_eq,
            min_ineq=min_ineq,
        )

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
        print(DISP_RULE_MAIN)
        print("===          Trajectory Optimization Program             ===")
        print(DISP_RULE_MAIN)
        print("system:", getattr(sys, "name", type(sys).__name__))
        print(f"dimensions: n={int(sys.n)}, m={int(sys.m)}, p={int(sys.p)}")
        print("x_start:", preview_vector(problem.x_start))
        print("x_goal:", preview_vector(problem.x_goal))
        print("X:", self._class_name(problem.X))
        print("U:", self._class_name(problem.U))
        print("X0:", self._class_name(problem.X0))
        print("Xf:", self._class_name(problem.Xf))
        print("cost:", self._class_name(problem.cost))
        print(DISP_RULE_DIV)
        print("transcription:", type(self.transcription).__name__)
        print("transcription_options:", self._transcription_options())
        print(f"compile_backend={self.options.compile_backend!r}")
        print(f"program_backend={optimizer.program_evaluator.backend!r}")
        print("n_z:", int(program.n_z))
        print(
            f"constraints: n_h={optimizer.program_evaluator.n_h}, "
            f"n_g={optimizer.program_evaluator.n_g}"
        )
        print("z0:", preview_vector(z0))
        print(DISP_RULE_DIV)
        print(f"method={self.options.optimizer_method!r}")
        print("options:", getattr(optimizer.backend, "options", {}))
        print(f"setup: transcribe={transcribe_s:.6g}s compile={compile_s:.6g}s")
        print(DISP_RULE_DIV)
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
        max_eq, min_ineq, max_bound = optimizer.program_evaluator.constraint_violations(
            result.z
        )

        print("Completed in", result.solve_time_s, "seconds")
        print(DISP_RULE_DIV)
        print("success:", result.success)
        print("message:", result.message)
        print("J*:", result.cost)
        print("stats:", result.stats)
        print("max_eq:", max_eq)
        print("min_ineq:", min_ineq)
        print("max_bound:", max_bound)
        print("x(0):", preview_vector(trajectory.x[:, 0]))
        print("x(tf):", preview_vector(trajectory.x[:, -1]))
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
        print(DISP_RULE_MAIN)

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


# Reserved online keys for pipeline B (ObstacleBank / J(z, p)); not bound yet.
_DEFERRED_ONLINE_PARAM_KEYS = frozenset({"scene"})


def _resolve_transcription(
    transcription: Transcription | str | None,
    n_steps: int | None,
) -> Transcription:
    """Resolve string / default / instance transcription for TOP."""
    if isinstance(transcription, Transcription):
        if n_steps is not None:
            existing = getattr(getattr(transcription, "options", None), "n_steps", None)
            if existing is not None and int(existing) != int(n_steps):
                raise ValueError(
                    f"n_steps={n_steps} conflicts with transcription.options.n_steps="
                    f"{existing}."
                )
        return transcription

    if n_steps is None:
        raise ValueError(
            "n_steps is required when transcription is omitted or a string preset."
        )
    n_steps = int(n_steps)
    if n_steps < 2:
        raise ValueError(f"n_steps must be >= 2; got {n_steps}.")

    name = "direct_collocation" if transcription is None else str(transcription)
    if name == "direct_collocation":
        from minilink.planning.trajectory_optimization.direct_collocation import (
            DirectCollocationOptions,
            DirectCollocationTranscription,
        )

        return DirectCollocationTranscription(DirectCollocationOptions(n_steps=n_steps))
    if name == "multiple_shooting":
        from minilink.planning.trajectory_optimization.multiple_shooting import (
            MultipleShootingOptions,
            MultipleShootingTranscription,
        )

        return MultipleShootingTranscription(MultipleShootingOptions(n_steps=n_steps))
    raise ValueError(
        f"Unknown transcription {name!r}. Use one of {_TRANSCRIPTION_PRESETS}, "
        "None (default direct_collocation), or a Transcription instance."
    )


def _merge_trajopt_options(
    options: TrajectoryOptimizationOptions | None,
    **flat,
) -> TrajectoryOptimizationOptions:
    """Build options from optional bag + flat kwargs (flats win)."""
    base = TrajectoryOptimizationOptions() if options is None else options
    updates = {key: value for key, value in flat.items() if value is not _UNSET}
    unknown = sorted(k for k in updates if k not in _TRAJOPT_OPTION_KEYS)
    if unknown:
        raise ValueError(f"Unknown TrajectoryOptimizationPlanner kwargs: {unknown}.")
    if not updates:
        return base
    return replace(base, **updates)


def reject_unknown_online_params(params) -> None:
    """
    Validate online ``params`` for ``solve_trajectory_from`` / MPC.

    ``None`` or ``{}`` → bind ``x0`` only. Known deferred keys (``scene``) raise
    :class:`NotImplementedError` until pipeline B. Any other key raises
    :class:`ValueError`.
    """
    if params is None:
        return
    if not hasattr(params, "keys"):
        raise TypeError(
            f"params must be a mapping or None; got {type(params).__name__}."
        )
    keys = list(params.keys())
    if not keys:
        return
    unknown = sorted(k for k in keys if k not in _DEFERRED_ONLINE_PARAM_KEYS)
    deferred = sorted(k for k in keys if k in _DEFERRED_ONLINE_PARAM_KEYS)
    if unknown:
        raise ValueError(
            "solve_trajectory_from got unknown params keys "
            f"{unknown!r}. Pass params=None or {{}} for x0-only bind, "
            "or reserved key 'scene' (NotImplemented until pipeline B)."
        )
    if deferred:
        raise NotImplementedError(
            f"Online params key(s) {deferred} are reserved for pipeline B "
            "(ObstacleBank / ParametricMathematicalProgram J(z, p) bind). "
            "Pass params=None or {} for x0-only bind. See "
            "docs/plans/planning-pipeline-architecture.md."
        )
