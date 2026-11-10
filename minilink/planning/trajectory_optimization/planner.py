"""Trajectory optimization: a planning problem transcribed into a nonlinear program and solved."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.params import checked
from minilink.core.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import (
    NATIVE_METHODS,
    OptimizationProgressCallback,
    Optimizer,
    method_params_of,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import (
    Planner,
    host_tree,
    open_loop_controller_of,
    planner_evidence,
    problem_with,
)
from minilink.planning.problems import PlanningProblem, require_cost
from minilink.planning.results import PlanningSolution
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
    dynamics_function,
)

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

    Two routes to a plan from new numbers (a new start among them): offline,
    :meth:`solve` rebuilds a fresh
    :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
    each call (``planner.solve(params=p, guess=previous)``); in real time,
    :meth:`compile` is the planner's pure face, ``pv.solve(params, z0)``, which
    compiles once and never for new numbers.
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
            The intervals ``N`` of the grid on the horizon (``N + 1`` knots)
            when ``transcription`` is omitted or a string preset.
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
        **method_numbers
            A native method's own numbers (``learning_rate``, ``b1``, ``b2``,
            ``eps`` for ``"adam"``): held in :attr:`params` ``["method"]``.
        """
        super().__init__(problem)
        require_cost(self.problem)

        # Discretization: a transcription instance, or a preset on n_steps intervals
        self.transcription = resolve_transcription(transcription, n_steps)

        # Structure
        self.method = method
        self.backend = backend
        self.n_iterations = n_iterations
        self.use_hessian = bool(use_hessian)
        self.feasibility_tol = float(feasibility_tol)
        self.guess = guess

        # The numbers: a native law's, complete over its preset, the one home of
        # params["method"]; options= the external solver's knobs
        self.options = dict(options or {})
        self.method_numbers = method_params_of(
            method, method_numbers_of(method, self.options, method_numbers)
        )

        # Diagnostics: the machinery of the latest offline solve, never read by a tool
        # (its answer, timings and history are in solution.record)
        self.last_program: MathematicalProgram | None = None
        self.last_optimizer: Optimizer | None = None

        # The rebuild path warns once, on JAX, that a compiled program is faster

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
        method = self.method_numbers

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

    def solve(
        self,
        *,
        params=None,
        guess=None,
        evaluate: bool = False,
        n_trials: int = 50,
        verbose: bool = False,
        callback: Callable | None = None,
        live_plot: bool = False,
        record_history: bool = False,
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
        planner's own guess, else the default. The solution's controller is the optimized
        input as a source block, ``controller >> plant``; ``evaluate=True`` also
        replays it over the problem's draws (one trial when nothing is random).
        The workflow: ``verbose`` prints the preamble and the report,
        ``callback(iteration)`` sees every iterate, ``live_plot`` redraws the
        plan beside it, and ``record_history`` keeps a full :class:`Trajectory`
        per iterate in the record (one rollout each: leave it off for
        production solves).
        """
        problem_params, method = planner_parts(
            None if params is None else checked(params, self.params)
        )
        problem = problem_with(self.problem, host_tree(problem_params))

        return self.solve_on(
            problem,
            guess,
            evaluate,
            n_trials,
            method,
            verbose=verbose,
            callback=callback,
            live_plot=live_plot,
            record_history=record_history,
        )

    def solve_on(
        self,
        problem,
        guess,
        evaluate,
        n_trials,
        method=None,
        *,
        verbose=False,
        callback=None,
        live_plot=False,
        record_history=False,
    ) -> PlanningSolution:
        """One offline solve of ``problem`` from ``guess``: transcribe, seed, solve, read back, check, store."""
        workflow_t0 = time.perf_counter()
        backend = self.backend
        seed = self.resolve_guess(problem, guess)

        # The problem transcribed into a program and compiled into an optimizer, every
        # solve (a convenience verb builds; a held compiled planner is planner.compile());
        # the plant compiled once for the transcription, the read-back and the
        # iterates; the seed packed into its z0
        transcribe_t0 = time.perf_counter()
        dynamics = dynamics_function(problem, backend)
        program = self.transcription.transcribe(
            problem, backend=backend, dynamics=dynamics
        )
        transcribe_s = time.perf_counter() - transcribe_t0
        seed = decision_vector_of(seed, program)
        z0 = self.transcription.pack_initial_guess(problem, seed)
        compile_t0 = time.perf_counter()
        optimizer = self.build_optimizer(program, z0)
        warm_bound_form(optimizer)
        compile_s = time.perf_counter() - compile_t0

        if verbose:
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
            callback=self.build_callback(
                optimizer,
                problem,
                backend,
                history,
                dynamics,
                callback=callback,
                live_plot=live_plot,
                record_history=record_history,
            ),
            verbose=False,
        )
        reconstruct_t0 = time.perf_counter()
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=problem,
            backend=backend,
            dynamics=dynamics,
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
            optimizer_success=bool(optimization_result.success),
            message=str(optimization_result.message),
            n_iterations=dict(optimization_result.stats).get("nit"),
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

        if verbose:
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

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_guess(self, problem, guess) -> np.ndarray | Trajectory:
        """The seed: the given guess (a solution's trajectory), the planner's own guess, else the default on ``problem``."""
        if guess is None:
            guess = self.guess
        if guess is not None:
            return getattr(guess, "trajectory", guess)

        t = self.transcription.initial_guess_time_grid(problem)
        return default_initial_trajectory(problem, t)

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
        """The transcription's numbers as a dictionary (``{"n_steps": N}`` on a fixed grid)."""
        return dict(vars(self.transcription))

    def build_callback(
        self,
        optimizer: Optimizer,
        problem: PlanningProblem,
        backend: str,
        history: list,
        dynamics,
        *,
        callback: Callable | None = None,
        live_plot: bool = False,
        record_history: bool = False,
    ) -> OptimizationProgressCallback | None:
        """The optimizer's progress hook (``history`` appended to, live plot, user callback) on the solve's compiled ``dynamics``, or ``None``."""
        # The live figure, beside the user's callback when there is one
        if live_plot:
            # Lazy import: the graphics load on first use, not with the core
            from minilink.planning.trajectory_optimization.live_plot import (
                LiveTrajectoryPlotCallback,
            )

            live_callback = LiveTrajectoryPlotCallback(problem.sys)
            if callback is None:
                callback = live_callback
            else:
                user_callback = callback

                def callback(iteration):  # Both the live figure and the user's hook
                    live_callback(iteration)
                    user_callback(iteration)

        if not record_history and callback is None:
            return None

        iteration_index = 0

        def planner_progress(z: np.ndarray, J: float, _t: float) -> None:
            nonlocal iteration_index
            z_arr = np.asarray(z, dtype=float).reshape(-1)
            iteration = self.iteration_from_z(
                optimizer,
                problem,
                z_arr,
                backend,
                iteration_index,
                cost=J,
                dynamics=dynamics,
            )
            if record_history:
                history.append(iteration)
            if callback is not None:
                callback(iteration)
            iteration_index += 1

        return planner_progress

    def iteration_from_z(
        self,
        optimizer: Optimizer,
        problem: PlanningProblem,
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
            problem=problem,
            backend=backend,
            dynamics=dynamics,
        )
        max_eq, min_ineq, _ = optimizer.program_evaluator.constraint_violations(z)

        return TrajectoryOptimizationIteration(
            iteration=iteration_index,
            z=z.copy(),
            trajectory=trajectory,
            cost=cost,
            max_equality_violation=max_eq,
            min_inequality_margin=min_ineq,
        )

    def trajectory_solution(
        self, trajectory, record, evaluate, n_trials, problem=None
    ) -> PlanningSolution:
        """The schedule as a solution of the planner's problem (or ``problem``): its input replayed by a source block, scored over the draws when asked."""
        problem = self.problem if problem is None else problem
        controller = open_loop_controller_of(trajectory, self.problem.sys)

        evaluation = None
        if evaluate:
            dt = float(trajectory.t[1] - trajectory.t[0])
            evaluation = planner_evidence(
                self,
                controller,
                dt=dt,
                n_trials=n_trials,
                episode_length=float(trajectory.tf - trajectory.t[0]),
                problem=problem,
            )

        return PlanningSolution(
            problem=problem,
            controller=controller,
            record=record,
            trajectory=trajectory,
            evaluation=evaluation,
        )

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


@dataclass(frozen=True)
class TrajectoryOptimizationRecord:
    """
    The optimizer's account of one solve, and the plan's constraint check.

    ``success`` is *the returned plan satisfies the program constraints to
    ``feasibility_tol``*, nothing else: an iteration-limit stop on a feasible
    plan is not a failure, a solver that reports convergence on an infeasible
    plan is. The optimizer's own flag, ``optimizer_success``, and its message stay beside it.
    """

    cost: float | None
    optimizer_success: bool
    message: str
    n_iterations: int | None
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
            return bool(self.optimizer_success)

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

        solver = (
            f"solver {'ok' if self.optimizer_success else 'stopped'}: {self.message}"
        )
        if self.n_iterations is not None:
            solver += f", {self.n_iterations} iterations"
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
    max_equality_violation: float
    min_inequality_margin: float | None


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
    """The transcription passed in, or a preset (direct collocation by default) on ``n_steps`` intervals."""
    if isinstance(transcription, Transcription):
        if n_steps is not None:
            existing = getattr(transcription, "n_steps", None)
            if existing is not None and int(existing) != int(n_steps):
                raise ValueError(
                    f"n_steps={n_steps} conflicts with transcription.n_steps="
                    f"{existing}."
                )
        return transcription

    if n_steps is None:
        raise ValueError(
            "n_steps is required when transcription is omitted or a string preset."
        )
    n_steps = int(n_steps)
    if n_steps < 1:
        raise ValueError(f"n_steps must be >= 1; got {n_steps}.")

    name = "direct_collocation" if transcription is None else str(transcription)
    if name == "direct_collocation":
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.direct_collocation import (
            DirectCollocationTranscription,
        )

        return DirectCollocationTranscription(n_steps=n_steps)

    if name == "multiple_shooting":
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.multiple_shooting import (
            MultipleShootingTranscription,
        )

        return MultipleShootingTranscription(n_steps=n_steps)

    if name == "single_shooting":
        # Lazy import: a preset's transcription loads when it is chosen, not with
        # the planner
        from minilink.planning.trajectory_optimization.shooting import (
            ShootingTranscription,
        )

        return ShootingTranscription(n_steps=n_steps)

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
            f"TrajectoryOptimizationPlanner got unexpected keyword(s) {unknown}: "
            + (
                f"method {method!r} takes the numbers {sorted(own)}"
                if own
                else f"method {method!r} takes no numbers of its own; an external "
                "solver's knobs go in options= (options={'maxiter': 50})"
            )
        )

    return dict(numbers)


def type_name(value: object | None) -> str | None:
    """The type name of ``value``, ``None`` for ``None``."""
    if value is None:
        return None

    return type(value).__name__


def decision_vector_of(guess, program):
    """A guess as the transcription reads it: a compiled face's ``z`` (its origin appended) without its origin."""
    if isinstance(guess, np.ndarray) and guess.ndim == 1:
        if guess.size == int(program.n_z) + 1:
            return guess[:-1]

    return guess
