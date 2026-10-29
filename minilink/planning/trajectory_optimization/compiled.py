"""The compiled trajectory-optimization planner: the planner's pure face, ``pv = planner.compile()``."""

import copy

import numpy as np

from minilink.core.backends import array_module
from minilink.core.params import merge_known
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.results import PlanningSolution
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)

###############################################################################
# Public API
###############################################################################


class CompiledTrajectoryPlanner:
    """
    The pure face of a trajectory-optimization planner: its solve as functions of ``(params, z)``.

    ``z`` is the planner's solution vector: the transcription's decision vector
    (the knots, ``(rows, N)`` row after row: collocation's states then inputs,
    shooting's inputs) with the origin ``t_start`` it was solved from appended,
    ``n_z`` entries. ``params`` is the tree the plan depends on, the program's:
    ``x_start``, ``t_start``, the plant's, the cost's, the discount and the sets
    read as constraints. :meth:`solve`, :meth:`shift`, :meth:`command` and
    :meth:`prediction` are pure wherever the method is (a native method with
    ``iterations=K`` on JAX traces under ``jit``, ``vmap`` and ``grad``); SLSQP
    and Ipopt run on the host. :meth:`solution` rebuilds the rich
    :class:`~minilink.planning.results.PlanningSolution` on the host. The face
    keeps no last solution: every answer is a function of its arguments.
    """

    def __init__(self, planner, backend):
        problem = planner.problem
        transcription = planner.transcription

        # The structure: the problem, its transcription, the knots' offsets and sizes
        self.planner = planner
        self.problem = problem
        self.transcription = transcription
        self.backend = backend
        self.offsets = np.asarray(transcription.options.offsets(problem))
        self.n = int(problem.sys.n)
        self.m = int(problem.sys.m)

        # The tree the program is compiled with: a later edit of the problem is not the
        # face's (compile again to take it)
        self.defaults = copy.deepcopy(transcription.program_params(problem))

        # The program, compiled once with its optimizer at the default guess
        self.program = transcription.transcribe(problem, backend=backend)
        z_guess = self.initial()
        self.optimizer = planner.build_optimizer(self.program, z_guess[:-1])
        self.n_z = int(self.program.n_z) + 1

        # The face solves on bound values (an MPC tick, a sweep): its parametric form,
        # compiled now
        warm_parametric = getattr(
            self.optimizer.program_evaluator, "warm_parametric", None
        )
        if self.program.params is not None and warm_parametric is not None:
            warm_parametric()

    @property
    def params(self) -> dict:
        """The compiled defaults: the tree the plan depends on, as it was at :meth:`planner.compile`."""
        return copy.deepcopy(self.defaults)

    @property
    def predicts(self) -> bool:
        """Whether the plan carries its states, so :meth:`prediction` reads ``x`` (collocation)."""
        return isinstance(self.transcription, DirectCollocationTranscription)

    @property
    def knot_spacing(self) -> float:
        """The time between the plan's knots: the model's step for a prediction."""
        return float(self.offsets[1] - self.offsets[0])

    def initial(self, params=None):
        """The default guess in ``z`` form (``default_initial_trajectory``, packed), at ``params`` when they have values, else at the defaults."""
        problem = self.problem_at(params if has_values(params) else self.defaults)

        guess = default_initial_trajectory(
            problem, self.transcription.initial_guess_time_grid(problem)
        )
        z_dec = self.transcription.pack_initial_guess(problem, guess)

        return np.concatenate([z_dec, [problem.t_start]])

    def solve(self, params=None, z0=None):
        """
        The solve as a function: ``(z, status)`` from the guess ``z0`` (``None``: :meth:`initial`) on ``params``.

        ``params`` is a partial tree merged once onto the defaults; the
        solution records its origin ``params["t_start"]``. ``status`` is
        ``{"success", "iterations", "cost"}``. A NumPy program has no
        parameters: the values rebuild it on a copy of the problem.
        """
        tree = self.merged(params)
        z_start = self.initial(params) if z0 is None else z0
        z_dec0 = z_start[:-1]

        # z* = argmin J(z; p) from the guess, by the planner's method
        if self.program.params is None:
            result = self.optimizer_at(params, z_dec0).solve(z0=z_dec0)
        else:
            result = self.optimizer.solve(params=params, z0=z_dec0)

        # z = [z*, t_start]: the decision vector and the origin it was solved from
        xp = array_module(result.z, tree["t_start"])
        t_start = xp.reshape(xp.asarray(tree["t_start"], dtype=float), (1,))
        z = xp.concatenate([result.z, t_start])
        status = {
            "success": result.success,
            "iterations": result.stats.get("nit"),
            "cost": result.cost,
        }

        return z, status

    def shift(self, z, t):
        """The plan rebased to the absolute time ``t``: each knot row re-read at ``t + offset``, the tail held."""
        xp = array_module(z, t)

        rows = self.rows(z)
        origin = z[-1]
        knots = origin + self.offsets
        moved = t + self.offsets

        # r_new(t + offset_k) = r(origin + offset), linear between knots, held past the end
        shifted = xp.stack([xp.interp(moved, knots, row) for row in rows])
        z_shifted = xp.concatenate(
            [xp.reshape(shifted, (-1,)), xp.reshape(xp.asarray(t, dtype=float), (1,))]
        )

        return z_shifted

    def command(self, z, x, t, params=None):
        """The command the plan gives at time ``t``: its input knots read at ``t``, linear between knots (an open-loop plan ignores ``x``)."""
        xp = array_module(z, t)

        u_rows = self.rows(z)[-self.m :]
        knots = z[-1] + self.offsets

        # u(t), each input interpolated on the knots, held past the end
        u = xp.stack([xp.interp(t, knots, row) for row in u_rows])

        return u

    def prediction(self, z, t, params=None):
        """The planned ``(x, u)`` at ``t``: the state knots read at ``t`` (collocation and multiple shooting carry them in ``z``)."""
        require_state_knots(self.transcription)
        xp = array_module(z, t)

        x_rows = self.rows(z)[: self.n]
        knots = z[-1] + self.offsets

        # x(t) on the state knots, linear between them, held past the end
        x = xp.stack([xp.interp(t, knots, row) for row in x_rows])
        u = self.command(z, x, t, params)

        return x, u

    def solution(self, z, params=None, status=None) -> PlanningSolution:
        """
        The host-side :class:`~minilink.planning.results.PlanningSolution` of ``z`` on ``params``.

        The plan read back by the transcription, its constraints checked on the
        tree it was solved with, the open-loop policy replaying its input.
        ``status`` (from :meth:`solve`) fills the solver's record.
        """
        # Lazy import: the planner module imports this one
        from minilink.planning.trajectory_optimization.planner import (
            TrajectoryOptimizationRecord,
        )

        status = {} if status is None else status
        problem = self.problem_at(params)
        z_dec = np.asarray(z[:-1], dtype=float)
        cost = status.get("cost")

        result = OptimizationResult(
            z=z_dec,
            success=bool(status.get("success", True)),
            cost=None if cost is None else float(cost),
            stats={}
            if status.get("iterations") is None
            else {"nit": status["iterations"]},
        )
        trajectory = self.transcription.reconstruct_result(
            result, problem=problem, backend=self.backend
        )

        # Feasible to tol: max |h(z*)| ≤ tol, min g(z*) ≥ −tol, z* within its bounds,
        # on the program the plan was solved on
        if self.program.params is None:
            evaluator = self.optimizer_at(params, z_dec).program_evaluator
        else:
            evaluator = self.optimizer.program_evaluator
            evaluator.bind(params)
        max_eq, min_ineq, max_bound = evaluator.constraint_violations(z_dec)
        tol = float(self.planner.options.feasibility_tol)
        feasible = bool(
            max_eq <= tol
            and (min_ineq is None or min_ineq >= -tol)
            and max_bound <= tol
        )

        record = TrajectoryOptimizationRecord(
            cost=result.cost,
            solver_success=result.success,
            message="",
            iterations=status.get("iterations"),
            solve_time_s=None,
            feasibility_tol=tol,
            max_equality_violation=float(max_eq),
            min_inequality_margin=None if min_ineq is None else float(min_ineq),
            max_bound_violation=float(max_bound),
            feasible=feasible,
            stats=dict(result.stats),
        )
        policy = self.planner.open_loop_policy(trajectory)

        return PlanningSolution(problem, policy, record, trajectory, None)

    def pack(self, guess, params=None):
        """A guess in ``z`` form: a :class:`~minilink.core.trajectory.Trajectory` or a :class:`~minilink.planning.results.PlanningSolution` resampled on the knots."""
        problem = self.problem_at(params)
        trajectory = getattr(guess, "trajectory", guess)

        z_dec = self.transcription.pack_initial_guess(problem, trajectory)

        return np.concatenate([z_dec, [problem.t_start]])

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def merged(self, params):
        """The defaults with ``params`` merged over them, checked against the tree."""
        if not params:
            return self.params

        return merge_known(self.params, params)

    def optimizer_at(self, params, z_dec):
        """The optimizer of a NumPy program at ``params``: the compiled one for none, else one rebuilt on a copy of the problem holding them."""
        if not params:
            return self.optimizer

        program = self.transcription.transcribe(
            self.problem_at(params), backend=self.backend
        )

        return self.planner.build_optimizer(program, np.asarray(z_dec, dtype=float))

    def rows(self, z):
        """The knot rows of ``z``'s decision vector, ``(rows, N)``."""
        n_steps = self.offsets.size

        return z[:-1].reshape(-1, n_steps)

    def problem_at(self, params):
        """The problem itself for no values, else a copy holding them (its parts copied)."""
        # Lazy import: the planner module imports this one
        from minilink.planning.trajectory_optimization.planner import (
            host_tree,
            problem_with,
        )

        return problem_with(self.problem, host_tree(params))


###############################################################################
# Internal machinery
###############################################################################


def has_values(tree) -> bool:
    """Whether every leaf of a tree has a value (none is a tracer)."""
    # Lazy import: the planner module imports this one
    from minilink.planning.trajectory_optimization.planner import host_tree

    if not tree:
        return False
    try:
        host_tree(tree)
    except TypeError:  # a tracer has no value
        return False

    return True


def require_state_knots(transcription):
    """Refuse a prediction from a transcription whose ``z`` holds no state knots (single shooting)."""
    if not isinstance(transcription, DirectCollocationTranscription):
        raise NotImplementedError(
            f"{type(transcription).__name__} rolls its states out: read "
            "pv.solution(z).trajectory for the planned states"
        )
