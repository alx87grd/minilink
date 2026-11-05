"""Trajectory optimization: a planning problem transcribed into a nonlinear program and solved."""

from __future__ import annotations

import time
import warnings
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, normalize_backend
from minilink.core.params import checked, tree_digest
from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import (
    NATIVE_METHODS,
    OptimizationProgressCallback,
    Optimizer,
    make_optimizer_backend,
    method_for,
    method_params_of,
)
from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner, host_tree, planner_evidence, problem_with
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
    dynamics_function,
    require_jax_compile_backend,
)

_UNSET = object()  # Marks a keyword the caller did not pass

_TRANSCRIPTION_PRESETS = frozenset(
    {"direct_collocation", "multiple_shooting", "single_shooting"}
)


###############################################################################
# Public API
###############################################################################


class TrajectoryOptimizationPlanner(Planner):
    """
    Generic trajectory-optimization planner.

    A transcription owns the decision-vector layout and method-specific
    constraints. This planner owns the domain workflow:
    ``problem -> transcription -> mathematical program -> optimizer -> trajectory``.

    Offline :meth:`solve` always rebuilds a fresh
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
        method: str = "scipy_slsqp",
        backend: str = BACKEND_NUMPY,
        n_iterations: int | None = None,
        use_hessian: bool = False,
        feasibility_tol: float = 1e-6,
        guess=None,
        options: dict | None = None,
        verbose: bool = False,
        callback: Callable | None = None,
        live_plot: bool = False,
        record_history: bool = False,
        record_solve_time: bool = False,
        **method_numbers,
    ) -> None:
        """
        Parameters
        ----------
        problem : PlanningProblem
            Continuous-time planning task (requires a cost).
        transcription : Transcription or str, optional
            The transcription. ``None`` defaults to direct collocation.
            String presets: ``\"direct_collocation\"``, ``\"multiple_shooting\"``, ``\"single_shooting\"`` (box-bounded: the native methods solve it).
            Teach demos should pass the string explicitly.
        n_steps : int, optional
            Knot count when ``transcription`` is omitted or a string preset.
            Required in those cases; ignored when a ``Transcription`` instance
            already carries its grid (must not conflict if also passed).
        method, backend, n_iterations, use_hessian, feasibility_tol, guess
            Structure: the optimizer's method, the array backend, a native
            method's exact step count (``None`` runs to its tolerance), the
            dense Hessian, the plan's constraint tolerance (``success`` is the
            returned plan satisfying the program to it, even after an iteration
            limit), and the default initial guess.
        options : dict, optional
            An external solver's own knobs (SciPy's ``maxiter``, ``ftol``;
            Ipopt's), passed through untouched.
        verbose, callback, live_plot, record_history, record_solve_time
            Workflow: the preamble and report, a per-iterate callback (it takes
            precedence over ``live_plot``), the live redraw, a full
            :class:`Trajectory` per iterate (one rollout each: leave it off for
            production solves), and the solve's wall time.
        **method_numbers
            A native method's own numbers (``learning_rate``, ``b1``, ``b2``,
            ``eps`` for ``"adam"``): held in :attr:`params` ``["method"]``.
        """
        super().__init__(problem)
        self.require_cost()

        # Discretization: a transcription instance, or a preset on n_steps knots
        self.transcription = resolve_transcription(transcription, n_steps)

        # Structure
        self.method = method
        self.backend = backend
        self.n_iterations = n_iterations
        self.use_hessian = bool(use_hessian)
        self.feasibility_tol = float(feasibility_tol)
        self.guess = guess

        # The numbers: a native law's, as keywords; options= the external solver's knobs
        self.options = dict(options or {})
        self.method_numbers = method_numbers_of(method, self.options, method_numbers)

        # Workflow
        self.verbose = bool(verbose)
        self.callback = callback
        self.live_plot = bool(live_plot)
        self.record_history = bool(record_history)
        self.record_solve_time = bool(record_solve_time)

        # Diagnostics: the machinery of the latest offline solve, never read by a tool
        # (its answer, timings and history are in solution.record)
        self.last_program: MathematicalProgram | None = None
        self.last_optimizer: Optimizer | None = None

        # The offline program and its optimizer, kept with the key they were built on and
        # the problem's parts the key names (held, so no identity in it is reused)
        self.offline = None

        # The compiled parametric program, its optimizer and its dynamics
        self.program = None
        self.program_evaluator = None
        self.optimizer_backend = None
        self._dynamics = None

        # Wall-clock time of the latest compile
        self.compile_time_s: float | None = None

        # The rebuild path warns once, on JAX, that a compiled program is faster
        self._warned_uncompiled_from = False

    @property
    def params(self) -> dict:
        """
        The planner's tree: the problem's leaves the program reads, ``"method"`` and ``"seed"``.

        ``x0``, ``t0``, the plant's, the cost's, the discount and the
        sets the program reads; ``"method"`` the method's numbers (a native
        law's learning rate and moments, ``{}`` for a host method, whose name
        and tolerances are structure); ``"seed"`` ``None`` (a gradient method
        draws nothing).
        """
        method = method_params_of(self.method, self.method_numbers)

        return {
            **self.transcription.program_params(self.problem),
            "method": method,
            "seed": None,
        }

    def compile(self, backend=None):
        """
        The planner's pure face, ``pv``: its solve as functions of ``(params, z)``.

        ``backend`` defaults to the planner's. See
        :class:`~minilink.planning.trajectory_optimization.compiled.CompiledTrajectoryPlanner`.
        """
        # Lazy import: the compiled face loads on first compile, not with the planner
        from minilink.planning.trajectory_optimization.compiled import (
            CompiledTrajectoryPlanner,
        )

        backend = self.backend if backend is None else backend

        return CompiledTrajectoryPlanner(self, backend)

    @property
    def has_parametric_program(self) -> bool:
        """True after a successful :meth:`compile_parametric_program`."""
        return self.program_evaluator is not None and self.optimizer_backend is not None

    def solve(
        self,
        *,
        params=None,
        guess=None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """
        Offline trajopt entry.

        ``params`` is the complete tree of :attr:`params` (``x0``, ``t0``, the
        plant's, the cost's, ..., ``method``, ``seed``), a copy edited, solved on
        a copy of the problem holding it; ``None`` solves the problem as it is.
        ``guess`` seeds the solve: a :class:`~minilink.core.trajectory.Trajectory`,
        a packed ``z`` (a compiled face's, its origin dropped), or a
        :class:`~minilink.planning.results.PlanningSolution`
        (a warm start: ``planner.solve(guess=previous)``); ``None`` takes the
        options' guess, else the default. The solution's policy is the optimized
        input as a source block, ``policy >> plant``; ``evaluate=True`` also
        replays it over the problem's draws (one trial when nothing is random).
        """
        problem_params, method = planner_parts(
            None if params is None else checked(params, self.params)
        )
        problem = problem_with(self.problem, host_tree(problem_params))

        return self.solve_on(problem, guess, evaluate, n_trials, method)

    def solve_on(
        self, problem, guess, evaluate, n_trials, method=None
    ) -> PlanningSolution:
        """One offline solve of ``problem`` from ``guess``: transcribe, seed, solve, read back, check, store."""
        workflow_t0 = time.perf_counter()
        backend = self.backend
        seed = self.resolve_guess(problem, guess)

        # The problem transcribed into a program and compiled into an optimizer, or
        # the ones of the last solve while the problem is unchanged; the seed packed
        # into its z0
        key = self.offline_key(problem)
        transcribe_t0 = time.perf_counter()
        if key is None or self.offline is None or self.offline[0][0] != key[0]:
            program = self.transcription.transcribe(
                problem,
                backend=backend,
            )
            transcribe_s = time.perf_counter() - transcribe_t0
            seed = decision_vector_of(seed, program)
            z0 = self.transcription.pack_initial_guess(problem, seed)
            compile_t0 = time.perf_counter()
            optimizer = self.build_optimizer(program, z0)
            warm_bound_form(optimizer)
            compile_s = time.perf_counter() - compile_t0
            self.offline = (key, program, optimizer)
        else:
            _, program, optimizer = self.offline
            seed = decision_vector_of(seed, program)
            z0 = self.transcription.pack_initial_guess(problem, seed)
            transcribe_s = compile_s = 0.0

        if self.verbose:
            self.print_solve_preamble(
                program=program,
                optimizer=optimizer,
                z0=z0,
                transcribe_s=transcribe_s,
                compile_s=compile_s,
            )

        # z* = argmin J(z) subject to the program's constraints, then the plan read
        # back from z*
        history = []
        optimization_result = optimizer.solve(
            params=None if method is None else {**optimizer.params, "method": method},
            z0=z0,
            callback=self.build_callback(optimizer, backend, history),
            record_solve_time=self.record_solve_time or self.verbose,
            verbose=False,
        )
        reconstruct_t0 = time.perf_counter()
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=problem,
            backend=backend,
        )
        reconstruct_s = time.perf_counter() - reconstruct_t0
        total_s = time.perf_counter() - workflow_t0

        # Diagnostics of this solve
        self.last_program = program
        self.last_optimizer = optimizer

        # Success is the plan's feasibility, not the solver's flag: an iteration-limit
        # stop on a feasible plan is not a failure, and a solver that reports
        # convergence on an infeasible plan is. Feasible to tol:
        # max |h(z*)| ≤ tol, min g(z*) ≥ −tol, and z* within its bounds to tol
        max_eq, min_ineq, max_bound = optimizer.program_evaluator.constraint_violations(
            optimization_result.z
        )
        tol = float(self.feasibility_tol)
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
            feasibility_tol=float(self.feasibility_tol),
            max_equality_violation=float(max_eq),
            min_inequality_margin=(None if min_ineq is None else float(min_ineq)),
            max_bound_violation=float(max_bound),
            feasible=feasible,
            stats=dict(optimization_result.stats),
            step_time_s=total_s,
            optimization_result=optimization_result,
            history=history,
        )
        solution = self.store_solution(
            self.trajectory_solution(
                trajectory, record, evaluate, n_trials, problem=problem
            )
        )

        if self.verbose:
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

        backend = normalize_backend(self.backend, allow_direct=True)
        require_jax_compile_backend(self.backend)
        require_parametric_transcription(self.transcription)

        # The program with the start x0 as its parameter
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
        self.program_evaluator.bind({"x0": self.problem.x0})
        self.program_evaluator.warm_parametric()

        # The compiled plant that reads each tick's plan back, on the tick's numbers
        self._dynamics = self.problem.sys.compile(backend=backend).f

        # The optimizer every online tick reuses
        self.optimizer_backend = self.build_parametric_optimizer_backend()
        self.compile_time_s = time.perf_counter() - t0

    def solve_trajectory_from(
        self,
        x0,
        *,
        params=None,
        guess=None,
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
            The values of this tick: ``None``, or the planner's complete tree
            (:attr:`params`, a copy edited: ``p["plant"]["mu"] = 0.6``,
            ``p["t0"] = t``), checked against it, with ``x0`` as its ``x0``:
            an unknown or a missing path raises naming the known ones.
            ``None`` binds ``x0`` only; ``scene`` is reserved for the
            scene bind (A3) and raises.
        guess : ndarray, Trajectory or PlanningSolution, optional
            NLP seed (a schedule, a packed ``z``, or a solution's trajectory);
            ``None`` the default schedule from ``x0``.

        Notes
        -----
        Fast bind path when :attr:`has_parametric_program`; otherwise rebuilds
        the NLP on a copy of the problem holding the values (one-time warning
        suggesting :meth:`compile_parametric_program`): one API at two speeds.
        """
        require_no_scene(params)
        if params is not None:
            params, _ = planner_parts(checked(params, self.params))

        # The fast bind path on a compiled program, else a rebuild from x0
        if self.has_parametric_program:
            return self.solve_trajectory_from_parametric(
                x0,
                params=params,
                guess=guess,
                evaluate=evaluate,
                n_trials=n_trials,
            )

        return self.solve_trajectory_from_rebuild(
            x0,
            params=params,
            guess=guess,
            evaluate=evaluate,
            n_trials=n_trials,
        )

    def step(self, x0, *, guess=None) -> Trajectory:
        """Solve one from-tick and return the bare schedule (hand loops)."""
        return self.solve_trajectory_from(x0, guess=guess).trajectory

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_guess(self, problem, guess) -> np.ndarray | Trajectory:
        """The seed: the given guess (a solution's trajectory), the options' guess, else the default on ``problem``."""
        if guess is None:
            guess = self.guess
        if guess is not None:
            return getattr(guess, "trajectory", guess)

        t = self.transcription.initial_guess_time_grid(problem)
        return default_initial_trajectory(problem, t)

    def offline_key(self, problem):
        """
        What an offline program is built from, ``(key, parts)``: the problem's parts by identity, its horizon and its numbers (``None``: build every time).

        The planner is built once and run many times: a part replaced on the
        problem (its plant, cost or a set), a new horizon or new numbers build a
        new program; an edit inside a part (a plant's method, a transcription's
        options) needs a new planner. ``parts`` are held with the program, so no
        identity in the key is reused while it is kept.
        """
        params_of = getattr(self.transcription, "program_params", None)
        if params_of is None:
            return None

        parts = (
            problem.sys,
            problem.cost,
            problem.X,
            problem.U,
            problem.X0,
            problem.Xf,
        )
        key = (
            tuple(id(part) for part in parts),
            problem.horizon,
            tree_digest(params_of(problem)),
            tree_digest(sets_params(problem)),
        )

        return key, parts

    def build_optimizer(
        self, program: MathematicalProgram, z0: np.ndarray
    ) -> Optimizer:
        """The offline optimizer of ``program``, from the method preset and its options."""
        return Optimizer(
            program,
            z0=z0,
            method=self.method,
            use_hessian=self.use_hessian,
            options=dict(self.options),
            n_iterations=self.n_iterations,
            **self.method_numbers,
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
        print("system:", sys.name)
        print(f"dimensions: n={int(sys.n)}, m={int(sys.m)}, p={int(sys.p)}")
        print("x0:", preview_vector(problem.x0))
        print("goal point:", preview_vector(problem.goal_point()))
        print("X:", type_name(problem.X))
        print("U:", type_name(problem.U))
        print("X0:", type_name(problem.X0))
        print("Xf:", type_name(problem.Xf))
        print("cost:", type_name(problem.cost))
        print(DISP_RULE_DIV)
        print("transcription:", type(self.transcription).__name__)
        print("transcription_options:", self.transcription_options())
        print(f"backend={self.backend!r}")
        print(f"program_backend={optimizer.program_evaluator.backend!r}")
        print("n_z:", int(program.n_z))
        print(
            f"constraints: n_h={optimizer.program_evaluator.n_h}, "
            f"n_g={optimizer.program_evaluator.n_g}"
        )
        print("z0:", preview_vector(z0))
        print(DISP_RULE_DIV)
        print(f"method={self.method!r}")
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
        history: list,
    ) -> OptimizationProgressCallback | None:
        """The optimizer's progress hook (``history`` appended to, live plot, user callback), or ``None``."""
        callback = self.callback

        # The live figure, beside the user's callback when there is one
        if self.live_plot:
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

        if not self.record_history and callback is None:
            return None

        # One dynamics callable for every iterate (reconstruct_result would
        # otherwise compile the plant again on each optimizer step).
        dynamics = dynamics_function(
            self.problem,
            backend,
            self.transcription.plant_evaluator(self.problem, backend),
        )
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
            if self.record_history:
                history.append(iteration)
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
            evaluation = planner_evidence(
                self,
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
            f"(plan feasible to tol={self.feasibility_tol:g})",
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
        if self.problem.Xf is not None:
            terminal_error = trajectory.x[:, -1] - self.problem.goal_point()
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
        """The solver of the online ticks: the method's preset from the one table, the planner's options over it."""
        method = method_for(self.program, self.method)
        options = {**self.options, **self.method_numbers}

        return make_optimizer_backend(method, options=options)

    def solve_trajectory_from_parametric(
        self,
        x0,
        *,
        params=None,
        guess=None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """One online tick on the compiled program: bind ``x0`` and the tick's values, seed, solve, read the plan back."""
        require_compiled_program(self.program_evaluator, self.optimizer_backend)

        # The tick's tree: the given values, the measured state as x0, bound once
        step_t0 = time.perf_counter()
        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        require_x0_shape(x_arr, self.problem.sys)
        self.program_evaluator.bind({**(params or {}), "x0": x_arr})
        bound = self.program_evaluator.params

        # The problem restarted at the measured state and the tick's start time, a
        # copy holding the tick's other values when there are any
        t0 = float(bound["t0"])
        problem_k = problem_with(self.problem, host_tree(params)).replace(
            x0=x_arr, X0=SingletonSet(x_arr), t0=t0
        )

        # The seed: the given guess, else the default schedule from the measured state
        seed = self.resolve_guess(problem_k, guess)
        z0 = self.transcription.pack_initial_guess(problem_k, seed)

        # z* = argmin J(z; x0), timed when asked
        record_solve_time = self.record_solve_time or self.verbose
        if record_solve_time:
            solve_t0 = time.perf_counter()

        result = self.optimizer_backend.solve(
            self.program_evaluator,
            z0,
        )

        if record_solve_time:
            result = OptimizationResult(
                z=result.z,
                success=result.success,
                cost=result.cost,
                message=result.message,
                stats=result.stats,
                solve_time_s=time.perf_counter() - solve_t0,
            )

        # The plan read back from z* with the compiled dynamics on the tick's plant
        plant = bound["plant"]
        trajectory = self.transcription.reconstruct_result(
            result,
            problem=problem_k,
            dynamics=lambda x, u, t: self._dynamics(x, u, t, plant),
            params=bound,
        )
        step_time_s = time.perf_counter() - step_t0

        # An online tick runs no residual check: its success is the solver's flag
        record = TrajectoryOptimizationRecord(
            cost=result.cost,
            solver_success=bool(result.success),
            message=str(result.message),
            iterations=dict(result.stats).get("nit"),
            solve_time_s=result.solve_time_s,
            feasibility_tol=float(self.feasibility_tol),
            max_equality_violation=None,
            min_inequality_margin=None,
            max_bound_violation=None,
            feasible=None,
            stats=dict(result.stats),
            step_time_s=step_time_s,
            optimization_result=result,
        )
        solution = self.store_solution(
            self.trajectory_solution(
                trajectory, record, evaluate, n_trials, problem=problem_k
            )
        )

        if self.verbose:
            self.print_step_report(record)

        return solution

    def print_step_report(self, record: TrajectoryOptimizationRecord) -> None:
        """One line per online tick: the solver's flag, the cost and the timings."""
        j_txt = "n/a" if record.cost is None else f"{float(record.cost):.6g}"
        print(
            f"TOP step: success={record.solver_success} "
            f"J={j_txt} "
            f"solve={record.solve_time_s:.6g}s "
            f"step={record.step_time_s:.6g}s"
        )

    def solve_trajectory_from_rebuild(
        self,
        x0,
        *,
        params=None,
        guess=None,
        evaluate: bool = False,
        n_trials: int = 50,
    ) -> PlanningSolution:
        """One online tick by an offline solve from ``x0`` and the tick's values (warns once, on JAX, to compile instead)."""
        # Once: on JAX, a compiled parametric program would skip the rebuild
        if not self._warned_uncompiled_from:
            backend = normalize_backend(self.backend, allow_direct=True)
            if backend == BACKEND_JAX:
                warnings.warn(
                    "TrajectoryOptimizationPlanner.solve_trajectory_from is rebuilding "
                    "the NLP each call. Call compile_parametric_program() once for "
                    "fast bind-only from-solves (requires backend='jax').",
                    UserWarning,
                    stacklevel=3,
                )
            self._warned_uncompiled_from = True

        # One offline solve on a copy of the problem restarted at the measured state
        x_arr = np.asarray(x0, dtype=float).reshape(-1)
        require_x0_shape(x_arr, self.problem.sys)
        problem_k = problem_with(self.problem, params).replace(
            x0=x_arr, X0=SingletonSet(x_arr)
        )

        return self.solve_on(problem_k, guess, evaluate, n_trials)


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
    step_time_s: float | None = None
    optimization_result: OptimizationResult | None = field(
        default=None, repr=False, compare=False
    )
    history: list = field(default_factory=list, repr=False, compare=False)

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


def sets_params(problem) -> dict:
    """Every set's numbers by role: a box lowered to the decision bounds is no leaf of the program, yet its numbers build it."""
    return {
        role: getattr(set_, "params", None)
        for role in ("X", "U", "X0", "Xf")
        if (set_ := getattr(problem, role)) is not None
    }


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
            "bind the problem's own leaves (plant, cost, t0, …)"
        )


def warm_bound_form(optimizer) -> None:
    """Compile a JAX program's bound form before a timed solve (a NumPy program compiles nothing)."""
    warm = getattr(optimizer.program_evaluator, "warm", None)
    if warm is not None:
        warm()


def planner_parts(params):
    """``(problem, method)``: a planner's tree split into the problem's leaves and the method's numbers (the seed dropped: a gradient method draws nothing)."""
    if not params:
        return params, None
    problem = {
        key: value for key, value in params.items() if key not in ("method", "seed")
    }

    return problem, params.get("method")


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

    if name == "single_shooting":
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.shooting import (
            ShootingOptions,
            ShootingTranscription,
        )

        return ShootingTranscription(ShootingOptions(n_steps=n_steps))

    raise ValueError(
        f"Unknown transcription {name!r}. Use one of {_TRANSCRIPTION_PRESETS}, "
        "None (default direct_collocation), or a Transcription instance."
    )


def method_numbers_of(method, options, numbers):
    """Refuse a native law's number in ``options`` or a keyword the method does not take; return the numbers."""
    own = method_params_of(method) if method in NATIVE_METHODS else {}
    hidden = sorted(key for key in options if key in own)
    if hidden:
        raise ValueError(
            f"{hidden} are the {method!r} method's own numbers: pass them as keywords "
            f"(TrajectoryOptimizationPlanner(..., {hidden[0]}=...)); options= holds an "
            "external solver's knobs"
        )
    unknown = sorted(key for key in numbers if key not in own)
    if unknown:
        raise TypeError(
            f"TrajectoryOptimizationPlanner got unexpected keyword(s) {unknown}; "
            f"method {method!r} takes the numbers {sorted(own) or 'none'}"
        )

    return dict(numbers)


def type_name(value: object | None) -> str | None:
    """The type name of ``value``, ``None`` for ``None``."""
    if value is None:
        return None

    return type(value).__name__


def require_parametric_transcription(transcription):
    """Refuse a transcription that cannot build a parametric program."""
    if not transcription.supports_parametric:
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


def decision_vector_of(guess, program):
    """A guess as the transcription reads it: a compiled face's ``z`` (its origin appended) without its origin."""
    if isinstance(guess, np.ndarray) and guess.ndim == 1:
        if guess.size == int(program.n_z) + 1:
            return guess[:-1]

    return guess
