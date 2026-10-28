"""Trajectory optimization: a planning problem transcribed into a nonlinear program and solved."""

from __future__ import annotations

import time
import warnings
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass, field, replace

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, normalize_backend
from minilink.core.params import merge_known
from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import (
    OptimizationProgressCallback,
    Optimizer,
    make_optimizer_backend,
    method_for,
)
from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
    dynamics_function,
    require_jax_compile_backend,
)

_UNSET = object()  # Marks a keyword the caller did not pass

_TRAJOPT_OPTION_KEYS = (
    "backend",
    "initial_guess",
    "warm_start",
    "method",
    "optimizer_options",
    "use_hessian",
    "record_history",
    "callback",
    "record_solve_time",
    "verbose",
    "feasibility_tol",
    "live_plot",
    "iterations",
)

_TRANSCRIPTION_PRESETS = frozenset({"direct_collocation", "multiple_shooting"})


###############################################################################
# Public API
###############################################################################


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

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        transcription: Transcription | str | None = None,
        n_steps: int | None = None,
        options: TrajectoryOptimizationOptions | None = None,
        backend=_UNSET,
        initial_guess=_UNSET,
        warm_start=_UNSET,
        method=_UNSET,
        optimizer_options=_UNSET,
        use_hessian=_UNSET,
        record_history=_UNSET,
        callback=_UNSET,
        record_solve_time=_UNSET,
        verbose=_UNSET,
        feasibility_tol=_UNSET,
        live_plot=_UNSET,
        iterations=_UNSET,
    ) -> None:
        """
        Parameters
        ----------
        problem : PlanningProblem
            Continuous-time planning task (requires a cost).
        transcription : Transcription or str, optional
            The transcription. ``None`` defaults to direct collocation.
            String presets: ``\"direct_collocation\"``, ``\"multiple_shooting\"``.
            Teach demos should pass the string explicitly.
        n_steps : int, optional
            Knot count when ``transcription`` is omitted or a string preset.
            Required in those cases; ignored when a ``Transcription`` instance
            already carries its grid (must not conflict if also passed).
        options : TrajectoryOptimizationOptions, optional
            Tier-2 workflow bag. Flat kwargs below overlay matching fields.
        backend, initial_guess, warm_start, method,
        optimizer_options, use_hessian, record_history, callback,
        record_solve_time, verbose, feasibility_tol, live_plot, iterations
            Tier-1 flat mirrors of :class:`TrajectoryOptimizationOptions`.
        """
        super().__init__(problem)
        self.require_cost()

        # Discretization: a transcription instance, or a preset on n_steps knots
        self.transcription = resolve_transcription(transcription, n_steps)

        # Workflow options: the flat keywords over the bag
        self.options = trajopt_options_of(
            options,
            backend=backend,
            initial_guess=initial_guess,
            warm_start=warm_start,
            method=method,
            optimizer_options=optimizer_options,
            use_hessian=use_hessian,
            record_history=record_history,
            callback=callback,
            record_solve_time=record_solve_time,
            verbose=verbose,
            feasibility_tol=feasibility_tol,
            live_plot=live_plot,
            iterations=iterations,
        )

        # Memory of the latest solve
        self.last_program: MathematicalProgram | None = None
        self.last_optimizer: Optimizer | None = None
        self.last_optimization_result: OptimizationResult | None = None
        self.iteration_history: list[TrajectoryOptimizationIteration] = []

        # The compiled parametric program, its optimizer and its dynamics
        self.program = None
        self.program_evaluator = None
        self.optimizer_backend = None
        self._dynamics = None

        # Wall-clock times of the latest compile, solve and step
        self.compile_time_s: float | None = None
        self.last_solve_time_s: float | None = None
        self.last_step_time_s: float | None = None

        # The rebuild path warns once, on JAX, that a compiled program is faster
        self._warned_uncompiled_from = False

    @property
    def params(self) -> dict:
        """The tree the plan depends on (R148): ``x_start``, ``t_start``, the plant's, the cost's, the discount and the sets the program reads."""
        return self.transcription.program_params(self.problem)

    def compile(self, backend=None):
        """
        The planner's pure face, ``pv`` (R148): its solve as functions of ``(params, z)``.

        ``backend`` defaults to the planner's. See
        :class:`~minilink.planning.trajectory_optimization.compiled.CompiledTrajectoryPlanner`.
        """
        # Lazy import: the compiled face loads on first compile, not with the planner
        from minilink.planning.trajectory_optimization.compiled import (
            CompiledTrajectoryPlanner,
        )

        backend = self.options.backend if backend is None else backend

        return CompiledTrajectoryPlanner(self, backend)

    @property
    def has_parametric_program(self) -> bool:
        """True after a successful :meth:`compile_parametric_program`."""
        return self.program_evaluator is not None and self.optimizer_backend is not None

    def solve(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """
        Offline trajopt entry (``solve_trajectory``).

        The solution's policy is the optimized input as a source block,
        ``policy >> plant``; ``evaluate=True`` also replays it over the
        problem's draws (one trial when nothing is random).
        """
        return self.solve_trajectory(
            initial_guess=initial_guess,
            warm_start=warm_start,
            evaluate=evaluate,
            n_trials=n_trials,
        )

    def solve_trajectory(
        self,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        warm_start: bool | None = None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """Compute and store a trajectory-optimization solution (always rebuild)."""
        workflow_t0 = time.perf_counter()
        backend = self.options.backend
        guess = self.resolve_initial_guess(initial_guess, warm_start)

        # The problem transcribed into a program, and the seed packed into its z0
        transcribe_t0 = time.perf_counter()
        program = self.transcription.transcribe(
            self.problem,
            backend=backend,
        )
        transcribe_s = time.perf_counter() - transcribe_t0
        z0 = self.transcription.pack_initial_guess(self.problem, guess)

        # The program compiled into an optimizer
        compile_t0 = time.perf_counter()
        optimizer = self.build_optimizer(program, z0)
        compile_s = time.perf_counter() - compile_t0

        if self.options.verbose:
            self.print_solve_preamble(
                program=program,
                optimizer=optimizer,
                z0=z0,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
            )

        # z* = argmin J(z) subject to the program's constraints, then the plan read
        # back from z*
        self.iteration_history = []
        optimization_result = optimizer.solve(
            callback=self.build_callback(optimizer, backend),
            record_solve_time=self.options.record_solve_time or self.options.verbose,
            verbose=False,
        )
        reconstruct_t0 = time.perf_counter()
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            backend=backend,
        )
        reconstruct_s = time.perf_counter() - reconstruct_t0
        total_s = time.perf_counter() - workflow_t0

        # Memory of this solve
        self.last_program = program
        self.last_optimizer = optimizer
        self.last_optimization_result = optimization_result
        self.last_solve_time_s = optimization_result.solve_time_s
        self.last_step_time_s = total_s

        # Success is the plan's feasibility, not the solver's flag: an iteration-limit
        # stop on a feasible plan is not a failure, and a solver that reports
        # convergence on an infeasible plan is. Feasible to tol:
        # max |h(z*)| ≤ tol, min g(z*) ≥ −tol, and z* within its bounds to tol
        max_eq, min_ineq, max_bound = optimizer.program_evaluator.constraint_violations(
            optimization_result.z
        )
        tol = float(self.options.feasibility_tol)
        feasible = bool(
            max_eq <= tol
            and (min_ineq is None or min_ineq >= -tol)
            and max_bound <= tol
        )

        # The solver's account of the solve, beside the feasibility check
        record = TrajectoryOptimizationRecord(
            cost=optimization_result.cost,
            solver_success=bool(optimization_result.success),
            message=str(optimization_result.message),
            iterations=dict(optimization_result.stats).get("nit"),
            solve_time_s=optimization_result.solve_time_s,
            feasibility_tol=float(self.options.feasibility_tol),
            max_equality_violation=float(max_eq),
            min_inequality_margin=(None if min_ineq is None else float(min_ineq)),
            max_bound_violation=float(max_bound),
            feasible=feasible,
            stats=dict(optimization_result.stats),
        )
        solution = self.store_solution(
            self.trajectory_solution(trajectory, record, evaluate, n_trials)
        )

        if self.options.verbose:
            self.print_solve_report(
                record=record,
                result=optimization_result,
                trajectory=trajectory,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
                reconstruct_s=reconstruct_s,
                total_s=total_s,
            )

        return solution

    def compile_parametric_program(self) -> None:
        """Build and JIT-compile a parametric NLP once (idempotent).

        Requires ``options.backend='jax'`` and a transcription that
        implements :meth:`~Transcription.transcribe_parametric` (direct
        collocation).
        """
        if self.has_parametric_program:
            return

        backend = normalize_backend(self.options.backend, allow_direct=True)
        require_jax_compile_backend(self.options.backend)
        require_parametric_transcription(self.transcription)

        # The program with the start x_start as its parameter
        t0 = time.perf_counter()
        self.program = self.transcription.transcribe_parametric(
            self.problem,
            backend=backend,
        )

        # Its evaluator, traced on the default seed and bound to the problem's start
        guess = default_initial_trajectory(
            self.problem,
            self.transcription.initial_guess_time_grid(self.problem),
        )
        z0 = self.transcription.pack_initial_guess(self.problem, guess)
        self.program_evaluator = compile_program_evaluator(
            self.program, BACKEND_JAX, sample_z=z0
        )
        self.program_evaluator.bind({"x_start": self.problem.x_start})

        # The compiled plant that reads each tick's plan back, on the tick's numbers
        self._dynamics = self.problem.sys.compile(backend=backend, verbose=False).f

        # The optimizer every online tick reuses
        self.optimizer_backend = self.build_parametric_optimizer_backend()
        self.compile_time_s = time.perf_counter() - t0

    def solve_trajectory_from(
        self,
        x0,
        *,
        params=None,
        initial_guess: np.ndarray | Trajectory | None = None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """
        Online traj-family solve from measured ``x0``.

        Parameters
        ----------
        x0 : array_like
            Measured initial state.
        params : mapping, optional
            The values of this tick, a partial tree of the problem's leaves
            (``{"plant": {"mu": 0.6}, "cost": {"xbar": …}, "t_start": t}``),
            merged once onto the program's defaults with ``x0`` as its
            ``x_start`` (R148): an unknown path raises naming the known ones.
            ``None`` or ``{}`` binds ``x0`` only; ``scene`` is reserved for the
            scene bind (A3) and raises.
        initial_guess : ndarray or Trajectory, optional
            NLP seed (schedule and/or packed ``z``). Warm-start *policy*
            (reuse last latch) is owned by ``ModelPredictiveController``, not
            this method.

        Notes
        -----
        Fast bind path when :attr:`has_parametric_program`; otherwise rebuilds
        the NLP on a copy of the problem holding the values (one-time warning
        suggesting :meth:`compile_parametric_program`): one API at two speeds.
        """
        require_no_scene(params)

        # The fast bind path on a compiled program, else a rebuild from x0
        if self.has_parametric_program:
            return self.solve_trajectory_from_parametric(
                x0,
                params=params,
                initial_guess=initial_guess,
                evaluate=evaluate,
                n_trials=n_trials,
            )

        return self.solve_trajectory_from_rebuild(
            x0,
            params=params,
            initial_guess=initial_guess,
            evaluate=evaluate,
            n_trials=n_trials,
        )

    def step(
        self,
        x0,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> Trajectory:
        """Solve one from-tick and return the bare schedule (hand loops)."""
        return self.solve_trajectory_from(x0, initial_guess=initial_guess).trajectory

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_initial_guess(
        self,
        initial_guess: np.ndarray | Trajectory | None,
        warm_start: bool | None,
    ) -> np.ndarray | Trajectory:
        """The seed: the given guess, the last plan on a warm start, the options' guess, else the default."""
        if initial_guess is not None:
            return initial_guess

        use_warm_start = self.options.warm_start if warm_start is None else warm_start
        if use_warm_start and self.last_solution is not None:
            return self.last_solution.trajectory

        if self.options.initial_guess is not None:
            return self.options.initial_guess

        t = self.transcription.initial_guess_time_grid(self.problem)
        return default_initial_trajectory(self.problem, t)

    def build_optimizer(
        self, program: MathematicalProgram, z0: np.ndarray
    ) -> Optimizer:
        """The offline optimizer of ``program``, from the method preset and its options."""
        return Optimizer(
            program,
            z0=z0,
            method=self.options.method,
            use_hessian=self.options.use_hessian,
            options=dict(self.options.optimizer_options),
            iterations=self.options.iterations,
        )

    def print_solve_preamble(
        self,
        *,
        program: MathematicalProgram,
        optimizer: Optimizer,
        z0: np.ndarray,
        transcribe_s: float,
        compile_s: float,
    ) -> None:
        """Open the solve panel: the problem, the transcription and the method."""
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
        print("X:", type_name(problem.X))
        print("U:", type_name(problem.U))
        print("X0:", type_name(problem.X0))
        print("Xf:", type_name(problem.Xf))
        print("cost:", type_name(problem.cost))
        print(DISP_RULE_DIV)
        print("transcription:", type(self.transcription).__name__)
        print("transcription_options:", self.transcription_options())
        print(f"backend={self.options.backend!r}")
        print(f"program_backend={optimizer.program_evaluator.backend!r}")
        print("n_z:", int(program.n_z))
        print(
            f"constraints: n_h={optimizer.program_evaluator.n_h}, "
            f"n_g={optimizer.program_evaluator.n_g}"
        )
        print("z0:", preview_vector(z0))
        print(DISP_RULE_DIV)
        print(f"method={self.options.method!r}")
        print("options:", getattr(optimizer.adapter, "options", {}))
        print(f"setup: transcribe={transcribe_s:.6g}s compile={compile_s:.6g}s")
        print(DISP_RULE_DIV)
        print("Running trajectory optimization...")

    def transcription_options(self) -> dict[str, object]:
        """The transcription's options as a dictionary (empty when it has none)."""
        options = getattr(self.transcription, "options", None)
        if options is None:
            return {}

        return dict(vars(options))

    def build_callback(
        self,
        optimizer: Optimizer,
        backend: str,
    ) -> OptimizationProgressCallback | None:
        """The optimizer's progress hook (history, live plot, user callback), or ``None``."""
        callback = self.options.callback

        # The live figure, beside the user's callback when there is one
        if self.options.live_plot:
            # Lazy import: the graphics load on first use, not with the core
            from minilink.planning.trajectory_optimization.live_plot import (
                LiveTrajectoryPlotCallback,
            )

            live_callback = LiveTrajectoryPlotCallback(self.problem.sys)
            if callback is None:
                callback = live_callback
            else:
                user_callback = callback

                def callback(iteration):  # Both the live figure and the user's hook
                    live_callback(iteration)
                    user_callback(iteration)

        if not self.options.record_history and callback is None:
            return None

        # One dynamics callable for every iterate (reconstruct_result would
        # otherwise compile the plant again on each optimizer step).
        dynamics = dynamics_function(self.problem, backend)
        iteration_index = 0

        def planner_progress(z: np.ndarray, J: float, _t: float) -> None:
            nonlocal iteration_index
            z_arr = np.asarray(z, dtype=float).reshape(-1)
            iteration = self.iteration_from_z(
                optimizer,
                z_arr,
                backend,
                iteration_index,
                cost=J,
                dynamics=dynamics,
            )
            if self.options.record_history:
                self.iteration_history.append(iteration)
            if callback is not None:
                callback(iteration)
            iteration_index += 1

        return planner_progress

    def iteration_from_z(
        self,
        optimizer: Optimizer,
        z: np.ndarray,
        backend: str,
        iteration_index: int,
        *,
        cost: float | None = None,
        dynamics=None,
    ) -> TrajectoryOptimizationIteration:
        """One iterate ``z`` as the planner reports it: its plan, its cost and its residuals."""
        if cost is None:
            cost = optimizer.program_evaluator.objective(z)

        trajectory = self.transcription.reconstruct_result(
            OptimizationResult(z=z, success=False, cost=cost),
            problem=self.problem,
            backend=backend,
            dynamics=dynamics,
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

    def trajectory_solution(
        self, trajectory, record, evaluate, n_trials, problem=None
    ) -> PlanningSolution:
        """The schedule as a solution of the planner's problem (or ``problem``): its input replayed by a source block, scored over the draws when asked."""
        problem = self.problem if problem is None else problem
        policy = self.open_loop_policy(trajectory)

        evaluation = None
        if evaluate:
            dt = float(trajectory.t[1] - trajectory.t[0])
            evaluation = self.evaluate(
                policy,
                dt=dt,
                n_trials=n_trials,
                tf=float(trajectory.tf - trajectory.t[0]),
                problem=problem,
            )

        return PlanningSolution(problem, policy, record, trajectory, evaluation)

    def print_solve_report(
        self,
        *,
        record: TrajectoryOptimizationRecord,
        result: OptimizationResult,
        trajectory: Trajectory,
        transcribe_s: float,
        compile_s: float,
        reconstruct_s: float,
        total_s: float,
    ) -> None:
        """Close the solve panel: the record, the plan's ends and the timings."""
        print("Completed in", result.solve_time_s, "seconds")
        print(DISP_RULE_DIV)
        print(
            "success:",
            record.success,
            f"(plan feasible to tol={self.options.feasibility_tol:g})",
        )
        print("solver success:", result.success)
        print("message:", result.message)
        print("J*:", result.cost)
        print("stats:", result.stats)
        print("max_eq:", record.max_equality_violation)
        print("min_ineq:", record.min_inequality_margin)
        print("max_bound:", record.max_bound_violation)
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

    def build_parametric_optimizer_backend(self):
        """The solver of the online ticks: the method's preset from the one table (OPW-1), the planner's options over it."""
        method = method_for(self.program, self.options.method)
        options = dict(self.options.optimizer_options)

        return make_optimizer_backend(method, options=options)

    def solve_trajectory_from_parametric(
        self,
        x0,
        *,
        params=None,
        initial_guess: np.ndarray | Trajectory | None = None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """One online tick on the compiled program: bind ``x0`` and the tick's values, seed, solve, read the plan back."""
        require_compiled_program(self.program_evaluator, self.optimizer_backend)

        # The tick's tree: the given values, the measured state as x_start, bound once
        step_t0 = time.perf_counter()
        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        require_x0_shape(x_arr, self.problem.sys)
        self.program_evaluator.bind({**(params or {}), "x_start": x_arr})
        bound = self.program_evaluator.params

        # The problem restarted at the measured state and the tick's start time, a
        # copy holding the tick's other values when there are any
        t_start = float(bound["t_start"])
        problem_k = problem_with(self.problem, host_tree(params)).replace(
            x_start=x_arr, X0=SingletonSet(x_arr), t_start=t_start
        )

        # The seed: the given guess, else the default schedule from the measured state
        if initial_guess is None:
            initial_guess = default_initial_trajectory(
                problem_k,
                self.transcription.initial_guess_time_grid(problem_k),
            )
        z0 = self.transcription.pack_initial_guess(problem_k, initial_guess)

        # z* = argmin J(z; x0), timed when asked
        record_solve_time = self.options.record_solve_time or self.options.verbose
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

        # The plan read back from z* with the compiled dynamics on the tick's plant
        plant = bound["plant"]
        trajectory = self.transcription.reconstruct_result(
            result,
            problem=problem_k,
            dynamics=lambda x, u, t: self._dynamics(x, u, t, plant),
            params=bound,
        )
        self.last_step_time_s = time.perf_counter() - step_t0
        self.last_optimization_result = result

        # An online tick runs no residual check: its success is the solver's flag
        record = TrajectoryOptimizationRecord(
            cost=result.cost,
            solver_success=bool(result.success),
            message=str(result.message),
            iterations=dict(result.stats).get("nit"),
            solve_time_s=result.solve_time_s,
            feasibility_tol=float(self.options.feasibility_tol),
            max_equality_violation=None,
            min_inequality_margin=None,
            max_bound_violation=None,
            feasible=None,
            stats=dict(result.stats),
        )
        solution = self.store_solution(
            self.trajectory_solution(
                trajectory, record, evaluate, n_trials, problem=problem_k
            )
        )

        if self.options.verbose:
            self.print_step_report(result)

        return solution

    def print_step_report(self, result: OptimizationResult) -> None:
        """One line per online tick: the solver's flag, the cost and the timings."""
        j_txt = "n/a" if result.cost is None else f"{float(result.cost):.6g}"
        print(
            f"TOP step: success={result.success} "
            f"J={j_txt} "
            f"solve={self.last_solve_time_s:.6g}s "
            f"step={self.last_step_time_s:.6g}s"
        )

    def solve_trajectory_from_rebuild(
        self,
        x0,
        *,
        params=None,
        initial_guess: np.ndarray | Trajectory | None = None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """One online tick by an offline solve from ``x0`` and the tick's values (warns once, on JAX, to compile instead)."""
        # Once: on JAX, a compiled parametric program would skip the rebuild
        if not self._warned_uncompiled_from:
            backend = normalize_backend(self.options.backend, allow_direct=True)
            if backend == BACKEND_JAX:
                warnings.warn(
                    "TrajectoryOptimizationPlanner.solve_trajectory_from is rebuilding "
                    "the NLP each call. Call compile_parametric_program() once for "
                    "fast bind-only from-solves (requires backend='jax').",
                    UserWarning,
                    stacklevel=3,
                )
            self._warned_uncompiled_from = True

        # One offline solve on the problem restarted at the measured state, then the
        # planner's own problem back
        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        require_x0_shape(x_arr, self.problem.sys)
        saved_problem = self.problem
        self.problem = problem_with(saved_problem, params).replace(
            x_start=x_arr, X0=SingletonSet(x_arr)
        )
        try:
            return self.solve_trajectory(
                initial_guess=initial_guess,
                warm_start=False,
                evaluate=evaluate,
                n_trials=n_trials,
            )
        finally:
            self.problem = saved_problem


@dataclass
class TrajectoryOptimizationOptions:
    """Generic trajectory-optimization workflow options.

    ``verbose`` prints the Minilink trajectory-optimization preamble and report
    on :meth:`TrajectoryOptimizationPlanner.solve`, and one timing line per tick
    on the online :meth:`TrajectoryOptimizationPlanner.solve_trajectory_from`
    path. It is separate from SciPy's per-iteration ``options['disp']``.

    ``record_history=True`` reconstructs a full :class:`Trajectory` per
    optimizer iterate — convenient for live plots and teaching, but it adds
    one rollout/unpack per iteration; leave it off for production solves.

    Parametric compile (:meth:`TrajectoryOptimizationPlanner.compile_parametric_program`)
    requires ``backend='jax'``.
    """

    backend: str = BACKEND_NUMPY
    initial_guess: np.ndarray | Trajectory | None = None
    warm_start: bool = False
    method: str = "scipy_slsqp"
    optimizer_options: dict[str, object] = field(default_factory=dict)
    use_hessian: bool = False
    record_history: bool = False
    callback: Callable[[TrajectoryOptimizationIteration], None] | None = None
    record_solve_time: bool = False
    verbose: bool = False
    #: ``success`` is granted when the returned plan satisfies the program
    #: constraints (equality residuals, inequality margins, bounds) to this
    #: tolerance, even if the solver stopped on an iteration limit.
    feasibility_tol: float = 1e-6
    #: Redraw the iterate trajectory during the solve (matplotlib); a
    #: custom ``callback`` takes precedence.
    live_plot: bool = False
    #: A native method's fixed step count, forwarded to its ``Optimizer``
    #: (R149): exactly this many steps; ``None`` runs to the method's tolerance.
    iterations: int | None = None


@dataclass(frozen=True)
class TrajectoryOptimizationRecord:
    """
    The optimizer's account of one solve, and the plan's constraint check.

    ``success`` is *the returned plan satisfies the program constraints to
    ``feasibility_tol``*, nothing else: an iteration-limit stop on a feasible
    plan is not a failure, a solver that reports convergence on an infeasible
    plan is. The solver's own flag and message stay beside it. An online tick
    (``solve_trajectory_from`` on a compiled program) runs no residual check,
    ``feasible`` is ``None`` and ``success`` is the solver's flag.
    """

    cost: float | None
    solver_success: bool
    message: str
    iterations: int | None
    solve_time_s: float | None
    feasibility_tol: float
    max_equality_violation: float | None
    min_inequality_margin: float | None
    max_bound_violation: float | None
    feasible: bool | None
    stats: dict

    @property
    def success(self) -> bool:
        """The plan is feasible to ``feasibility_tol``; on an online tick, the solver's flag."""
        if self.feasible is None:
            return bool(self.solver_success)

        return bool(self.feasible)

    # Display

    def __str__(self) -> str:
        cost = "n/a" if self.cost is None else f"{float(self.cost):.6g}"

        if self.feasible is None:
            check = "online tick, no residual check"
        elif self.feasible:
            check = f"feasible to {self.feasibility_tol:g}"
        else:
            check = f"infeasible at {self.feasibility_tol:g}"
        if self.max_equality_violation is not None:
            check += f" (max defect {self.max_equality_violation:.2g})"

        solver = f"solver {'ok' if self.solver_success else 'stopped'}: {self.message}"
        if self.iterations is not None:
            solver += f", {self.iterations} iterations"
        if self.solve_time_s is not None:
            solver += f", {self.solve_time_s:.3g} s"

        return f"J* = {cost}, {check}; {solver}"


@dataclass(frozen=True)
class TrajectoryOptimizationIteration:
    """One optimizer iterate as the planner reports it: ``z``, its plan, its cost and its residuals."""

    iteration: int
    z: np.ndarray
    trajectory: Trajectory
    cost: float
    max_eq: float
    min_ineq: float | None


def require_no_scene(params) -> None:
    """
    Refuse online ``params`` that are not a tree, or that hold the reserved ``scene``.

    The tree's own check is the program's (an unknown path raises naming the
    known ones); ``scene`` waits for the scene bind (A3) and raises
    :class:`NotImplementedError`.
    """
    if params is None:
        return

    if not hasattr(params, "keys"):
        raise TypeError(
            f"params must be a mapping or None; got {type(params).__name__}."
        )
    if "scene" in params:
        raise NotImplementedError(
            "the online params key 'scene' waits for the scene bind (A3); "
            "bind the problem's own leaves (plant, cost, t_start, …)"
        )


def host_tree(tree):
    """A tree with every leaf a NumPy value (``None`` kept), or ``None`` for none."""
    if not tree:
        return None
    if isinstance(tree, dict):
        return {
            key: host_tree(value) if isinstance(value, dict) else host_leaf(value)
            for key, value in tree.items()
        }

    return tree


def host_leaf(value):
    """One leaf on the host: a NumPy value, ``None`` kept."""
    return None if value is None else np.asarray(value)


def problem_with(problem, params):
    """The problem itself for no values, else a copy holding them: its parts copied, so the caller's stay as they are."""
    if not params:
        return problem

    merged = merge_known(problem.parts_view(), params)
    copy = deepcopy(problem)
    copy.params = merged

    return copy


###############################################################################
# Internal machinery
###############################################################################


def resolve_transcription(
    transcription: Transcription | str | None,
    n_steps: int | None,
) -> Transcription:
    """The transcription passed in, or a preset (direct collocation by default) on ``n_steps`` knots."""
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
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.direct_collocation import (
            DirectCollocationOptions,
            DirectCollocationTranscription,
        )

        return DirectCollocationTranscription(DirectCollocationOptions(n_steps=n_steps))

    if name == "multiple_shooting":
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.multiple_shooting import (
            MultipleShootingOptions,
            MultipleShootingTranscription,
        )

        return MultipleShootingTranscription(MultipleShootingOptions(n_steps=n_steps))

    raise ValueError(
        f"Unknown transcription {name!r}. Use one of {_TRANSCRIPTION_PRESETS}, "
        "None (default direct_collocation), or a Transcription instance."
    )


def trajopt_options_of(
    options: TrajectoryOptimizationOptions | None,
    **flat,
) -> TrajectoryOptimizationOptions:
    """The options bag with the passed flat keywords laid over it (flats win)."""
    base = TrajectoryOptimizationOptions() if options is None else options
    updates = {key: value for key, value in flat.items() if value is not _UNSET}
    unknown = sorted(k for k in updates if k not in _TRAJOPT_OPTION_KEYS)
    if unknown:
        raise ValueError(f"Unknown TrajectoryOptimizationPlanner kwargs: {unknown}.")

    if not updates:
        return base

    return replace(base, **updates)


def type_name(value: object | None) -> str | None:
    """The type name of ``value``, ``None`` for ``None``."""
    if value is None:
        return None

    return type(value).__name__


def require_parametric_transcription(transcription):
    """Refuse a transcription that cannot build a parametric program."""
    if not getattr(transcription, "supports_parametric", False):
        raise TypeError(
            f"{type(transcription).__name__} does not support a "
            "parametric program; use transcription='direct_collocation'."
        )


def require_compiled_program(program_evaluator, optimizer_backend):
    """Refuse a parametric tick before :meth:`compile_parametric_program` has run."""
    if program_evaluator is None or optimizer_backend is None:
        raise RuntimeError(
            "compile_parametric_program() must run before parametric from-solve."
        )


def require_x0_shape(x_arr, sys):
    """Refuse a measured state without the system's ``n`` entries."""
    n = int(sys.n)
    if x_arr.shape != (n,):
        raise ValueError(f"x0 must have shape ({n},)")
