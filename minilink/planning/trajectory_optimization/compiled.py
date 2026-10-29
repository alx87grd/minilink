"""The compiled trajectory-optimization planner: the planner's pure face, ``pv = planner.compile()``."""

import copy

import numpy as np

from minilink.core.backends import array_module
from minilink.core.compile.evaluators.base import assigned
from minilink.core.compile.evaluators.stepping import step_body
from minilink.core.params import merge_known
from minilink.core.trajectory import Trajectory
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
    ``n_z`` entries. ``params`` is the planner's tree: the program's leaves
    (``x_start``, ``t_start``, the plant's, the cost's, the discount and the
    sets read as constraints), the method's numbers under ``"method"`` and
    ``"seed"``. :meth:`solve`, :meth:`shift`, :meth:`command` and
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
        self.defaults = copy.deepcopy(planner.params)

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
        """
        The default guess in ``z`` form (``default_initial_trajectory``, packed), at ``params``.

        On the host the guess is built at ``params`` (the defaults when none).
        Under a trace it is a function of the traced ``x_start`` and
        ``t_start``: the guess is affine in its start (a straight line or a
        cubic from it), ``z(x̂) = z(x̄) + M (x̂ − x̄)``, ``M`` read once on the
        host, so a traced loop's first plan starts from the same guess as the
        host's; the tree's other numbers are taken at the compiled defaults.
        """
        if not params or has_values(params):
            return self.host_initial(self.defaults if not params else params)

        P = self.merged(params)
        z_default, sensitivity, x_default = self.start_sensitivity
        xp = array_module(P["x_start"], P["t_start"])

        # z(x̂) = z(x̄) + M (x̂ − x̄), the origin t_start appended
        x_hat = xp.reshape(xp.asarray(P["x_start"], dtype=float), (-1,))
        z_dec = z_default[:-1] + sensitivity @ (x_hat - x_default)
        t_start = xp.reshape(xp.asarray(P["t_start"], dtype=float), (1,))
        z = xp.concatenate([z_dec, t_start])

        return z

    def host_initial(self, params):
        """The default guess built on the host at ``params``."""
        problem = self.problem_at(params)

        guess = default_initial_trajectory(
            problem, self.transcription.initial_guess_time_grid(problem)
        )
        z_dec = self.transcription.pack_initial_guess(problem, guess)

        return np.concatenate([z_dec, [problem.t_start]])

    @property
    def start_sensitivity(self):
        """``(z(x̄), M, x̄)``: the default guess at the compiled start and its derivative in the start, one column per state (the guess is affine in it)."""
        cached = self.__dict__.get("start_sensitivity_cache")
        if cached is not None:
            return cached

        x_default = np.asarray(self.defaults["x_start"], dtype=float).reshape(-1)
        z_default = self.host_initial(self.defaults)

        # M e_i = z(x̄ + e_i) − z(x̄): exact for a guess affine in its start
        columns = []
        for i in range(x_default.size):
            x_i = x_default.copy()
            x_i[i] += 1.0
            z_i = self.host_initial({**self.defaults, "x_start": x_i})
            columns.append(z_i[:-1] - z_default[:-1])
        sensitivity = np.stack(columns, axis=1) if columns else np.zeros((0, 0))

        cached = (z_default, sensitivity, x_default)
        self.__dict__["start_sensitivity_cache"] = cached

        return cached

    def solve(self, params=None, z0=None):
        """
        The solve as a function: ``(z, status)`` from the guess ``z0`` (``None``: :meth:`initial`) on ``params``.

        ``params`` is a partial tree merged once onto the defaults; the
        solution records its origin ``params["t_start"]``. ``status`` is
        ``{"success", "iterations", "cost"}``. A NumPy program has no
        parameters: the values rebuild it on a copy of the problem.
        """
        tree = self.merged(params)
        problem_params, _ = planner_parts(params)
        numbers = {"method": tree["method"]}
        z_start = self.initial(params) if z0 is None else z0
        z_dec0 = z_start[:-1]

        # z* = argmin J(z; p) from the guess, by the planner's method on its numbers
        if self.program.params is None:
            optimizer = self.optimizer_at(problem_params, z_dec0)
            result = optimizer.solve(params=numbers, z0=z_dec0)
        else:
            result = self.optimizer.solve(
                params={**(problem_params or {}), **numbers}, z0=z_dec0
            )

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

    def shift(self, z, t, x_start=None):
        """
        The plan rebased to the absolute time ``t``: each knot row re-read at ``t + offset``, the tail held.

        With ``x_start`` (a plan that carries its states), the first state knot
        is pinned at it: the warm start of a solve from that measured start.
        """
        xp = array_module(z, t)

        offsets = self.offsets

        rows = self.rows(z)
        origin = z[-1]
        knots = origin + offsets
        moved = t + offsets

        # r_new(t + offset_k) = r(origin + offset), linear between knots, held past the end
        shifted = xp.stack([xp.interp(moved, knots, row) for row in rows])
        z_shifted = xp.concatenate(
            [xp.reshape(shifted, (-1,)), xp.reshape(xp.asarray(t, dtype=float), (1,))]
        )
        if x_start is None or not self.predicts:
            return z_shifted

        # x_i(t) = x̂_i: the state rows come first in z, N knots each, x_i's first at i·N
        first_knots = xp.arange(self.n) * offsets.size
        x_hat = xp.reshape(xp.asarray(x_start, dtype=float), (self.n,))
        z_pinned = assigned(xp, z_shifted, first_knots, x_hat)

        return z_pinned

    def predict_start(self, x, u, t, delta, params=None):
        """
        ``x̂(t + Δ)``: the planner's model from ``x`` at ``t``, the command ``u`` held over ``delta``.

        RK4 steps a quarter of a knot apart, on the plant numbers of ``params``
        (the defaults when ``None``): a predictor block's equation, for a user
        who compensates the computation delay on the start the plan binds.
        """
        return model_prediction(self, x, u, t, delta, params)

    def command(self, z, x, t, params=None):
        """The command the plan gives at time ``t``: its input knots read at ``t``, linear between knots (an open-loop plan ignores ``x``)."""
        xp = array_module(z, t)

        m = self.m
        offsets = self.offsets

        u_rows = self.rows(z)[-m:]
        knots = z[-1] + offsets

        # u(t), each input interpolated on the knots, held past the end
        u = xp.stack([xp.interp(t, knots, row) for row in u_rows])

        return u

    def prediction(self, z, t, params=None):
        """The planned ``(x, u)`` at ``t``: the state knots read at ``t`` (collocation and multiple shooting carry them in ``z``)."""
        require_state_knots(self)
        xp = array_module(z, t)

        n = self.n
        offsets = self.offsets

        x_rows = self.rows(z)[:n]
        knots = z[-1] + offsets

        # x(t) on the state knots, linear between them, held past the end
        x = xp.stack([xp.interp(t, knots, row) for row in x_rows])
        u = self.command(z, x, t, params)

        return x, u

    def planned_trajectory(self, z, params=None) -> Trajectory:
        """The plan as a trajectory: its state and input knots from its origin (collocation and multiple shooting carry them in ``z``)."""
        if self.n_z - 1 != (self.n + self.m) * self.offsets.size:
            raise NotImplementedError(
                "the plan carries no state knots (single shooting): read "
                "pv.solution(z).trajectory for its planned states"
            )

        rows = np.asarray(z[:-1], dtype=float).reshape(-1, self.offsets.size)
        t = float(z[-1]) + self.offsets

        return Trajectory(t=t, x=rows[: self.n], u=rows[-self.m :])

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
            evaluator.bind(planner_parts(params)[0])
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
        params, _ = planner_parts(params)
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
        """A copy of the problem holding the compiled tree with ``params`` merged over it (its parts copied): the face's values, never a later edit of the problem's."""
        # Lazy import: the planner module imports this one
        from minilink.planning.trajectory_optimization.planner import (
            host_tree,
            problem_with,
        )

        problem_params, _ = planner_parts(self.merged(params))

        return problem_with(self.problem, host_tree(problem_params))


###############################################################################
# Internal machinery
###############################################################################


def planner_parts(params):
    """``(problem, method)``: a planner's tree split into the problem's leaves and the method's numbers (the planner's :func:`~minilink.planning.trajectory_optimization.planner.planner_parts`)."""
    # Lazy import: the planner module imports this one
    from minilink.planning.trajectory_optimization import planner

    return planner.planner_parts(params)


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


def require_state_knots(pv):
    """Refuse a plan whose ``z`` holds no state knots (single shooting, MPPI): its states are rolled out."""
    if pv.n_z - 1 != (pv.n + pv.m) * pv.offsets.size:
        raise NotImplementedError(
            "the plan carries no state knots (single shooting): read "
            "pv.solution(z).trajectory for its planned states"
        )


def model_prediction(pv, x, u, t, delta, params=None):
    """``x̂(t + Δ) = Φ(x, u, t)``: a face's plant model from ``x``, ``u`` held, RK4 steps a quarter of a knot apart."""
    params = pv.params if params is None else params

    f = pv.problem.sys.f
    plant = params.get("plant") or None
    spacing = pv.knot_spacing
    substeps = max(1, int(np.ceil(4.0 * float(delta) / spacing)))

    def dynamics(x, u, tau):
        return f(x, u, tau, plant)

    # x̂(t + Δ) = Φ(x, u, t), the command held over Δ as the plant sees it
    x_ahead = step_body(dynamics, x, u, u, t, delta, "rk4", substeps)

    return x_ahead
