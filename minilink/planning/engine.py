"""The problem evaluator: a controller on a problem, compiled once, run to the horizon and scored."""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    ensure_jax_x64,
)
from minilink.core.compile.evaluators.stepping import require_step_knobs
from minilink.core.options import require_one_of
from minilink.core.trajectory import Trajectory

BACKENDS = ("auto", BACKEND_JAX, BACKEND_NUMPY)
EPISODE_LENGTH = 10.0  # s, the run of an infinite-horizon problem when none is named
PLANT_INPUT = "plant.u"  # the recorded signal of the plant's input, the plant role's u
PROBLEM_PRICE = "problem"  # score_run's price read from the problem

###############################################################################
# Public API
###############################################################################


class ProblemEvaluator:
    """
    A planning problem and a controller, compiled into one parametric run and its score.

    The loop is the controller closed on the problem's plant (``controller @ plant``,
    every free input held at its port's nominal value), or a built loop whose block
    ``plant`` is the problem's system. One run integrates the loop to the horizon on
    the fixed-step model (``solver``, ``dt``, ``substeps``), records every block
    output, and scores the plant's run by the problem's formula::

        pe = problem.compile(pid, dt=0.01)
        report = pe.evaluate()                                   # the Evaluation
        dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
        program = pe.program(wrt=["controller.Kp", "controller.Kd"])

    Parameters
    ----------
    problem : PlanningProblem
        The task: plant, cost, sets, horizon, price switch (and, for a stochastic
        problem, the draws and the criterion).
    controller : System or PlanningSolution
        A controller block (or a solution's policy) closed on the plant, a source
        block (no input) driving the plant's command, or a built loop with a
        ``plant`` block.
    dt : float
        Sample period of the run: the free inputs are held over it and the run is
        recorded on its grid.
    solver : {"rk4", "euler"}
        Integration formula of one step.
    substeps : int
        Integration steps per sample period.
    backend : {"auto", "jax", "numpy"}
        ``"auto"`` is JAX when it is installed and the loop traces on it.
    episode_length : float, optional
        Length of the run of an infinite-horizon problem (10 s by default).
    """

    def __init__(
        self,
        problem,
        controller,
        dt,
        *,
        solver="rk4",
        substeps=1,
        backend="auto",
        episode_length=None,
    ):
        require_one_of("backend", backend, BACKENDS)
        self.adaptive = adaptive_options(solver)
        if self.adaptive is None:
            require_step_knobs(solver, substeps)
        else:
            backend = require_numpy_for_adaptive(backend)

        # The task and the loop that runs it
        self.problem = problem
        self.loop = closed_loop_of(problem, controller)
        warn_if_disturbed(problem)
        self.solver = solver
        self.substeps = int(substeps)

        # The grid: N periods of dt to the horizon
        self.dt = float(dt)
        self.tf = horizon_of(problem, episode_length)
        self.n_steps = steps_of(self.tf, self.dt)

        # The loop, compiled once on its backend
        self.backend, self.evaluator = compile_loop(self.loop, backend)

        # The static layout of the run: the plant's state slice, the sources of its
        # input, the recorded signals and the free inputs
        self.plant_slice = self.loop.state_index["plant"]
        self.plant_input = plant_input_sources(self.loop, self.evaluator)
        self.block_ids = tuple(self.loop.subsystems)
        self.free_inputs = tuple(self.loop.inputs)
        require_free_names(self.free_inputs, self.block_ids)

        # The forms generated once: the batched trial, jitted on JAX
        self.trials_fn = None

        # "auto" keeps JAX only when the whole score traces (the cost and the sets too)
        if backend == "auto" and self.backend == BACKEND_JAX and not self.traces():
            self.backend, self.evaluator = compile_loop(self.loop, BACKEND_NUMPY)

    @property
    def params(self) -> dict:
        """
        The run's parameters as one nested tree, fresh from the objects.

        ``{block_id: block params}`` for every block of the loop; the problem's
        own leaves (``"cost"``, the parameters of the sets that hold any by role,
        ``"x_start"``, ``"discount_rate"``); and each free input of the loop at its
        nominal value. Every leaf a float array, so ``jax.grad`` differentiates the
        whole tree.
        """
        loop = self.loop
        problem = self.problem

        tree = {sys_id: loop.subsystems[sys_id].params for sys_id in self.block_ids}
        tree.update(problem_leaves(problem))
        for port in self.free_inputs:
            tree[port] = loop.inputs[port].nominal_value

        return float_leaves(tree)

    def trajectory(self, params=None, key=None) -> Trajectory:
        """
        One run of the loop, every block output recorded: the loop's trajectory.

        ``params`` overrides entries of :attr:`params` (a partial tree is merged);
        ``key=None`` is the nominal run (a stochastic problem at its mean start).
        The plant's run is ``traj.of("plant")``.
        """
        params = self.merged(params)

        # The nominal run, or the draw of one trial
        if key is None:
            x_start, params = params["x_start"], params
        else:
            x_start, params = self.realize(params, key, 1)
            x_start = x_start[0]
            params = take_trial(params, 0)

        return self.run(params, x_start)

    def evaluate(self, params=None, key=0, n_trials=None, *, record=False):
        """
        The :class:`Evaluation` of the loop on the problem: one run, or ``n_trials`` draws.

        A deterministic problem is one run (``n_trials`` 1 or unset); a stochastic
        problem draws ``n_trials`` starts and parameter overrides from ``key``. On
        JAX the whole evaluation traces, so ``jax.grad`` of ``.objective`` gives
        the gradient with respect to every leaf of ``params``. ``record=True``
        keeps the runs' trajectories (outside a trace).
        """
        params = self.merged(params)
        n_trials = trials_of(self.problem, n_trials)

        # The draws, the price of a failure, then the runs and their scores, batched on JAX
        x_starts, trial_params = self.realize(params, key, n_trials)
        price = price_of(self.problem, self.tf)
        rows, trajectories = self.run_trials(trial_params, x_starts, record, price)

        return Evaluation(
            **rows,
            x0=x_starts,
            trajectories=trajectories,
            criterion=criterion_of(self.problem),
        )

    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None):
        """
        The tuning program over the leaves ``wrt``: ``min_z J(z)`` with the declared sets as rows.

        ``J(z)`` is the objective of :meth:`evaluate` with the leaves named by
        ``wrt`` (dotted paths into :attr:`params`, as ``"controller.Kp"``) read
        from ``z`` and every other leaf at its current value; the draws are
        realized once, from ``key``. The constraint rows ``g(z) ≥ 0`` are the
        per-trial feasibility margin when ``X`` or ``U`` is bounded and the
        reachability margin when ``Xf`` is set. ``bounds`` maps a path to
        ``(lower, upper)`` (``None`` for an open side). ``params`` overrides
        entries of :attr:`params` at the base point (the leaves not tuned keep
        these values; the start ``z0`` reads the tuned ones).

        Three caveats of the price switch: the price is a step, with no gradient;
        with sets declared it counts beside their rows; with the switch off, one
        diverging trial makes ``J`` infinite and a line search fails, so a search
        that may cross into unstable gains sets ``infeasible_cost=True``. A smooth
        alternative for gradient tuning is a penalty term in the cost,
        ``cost + w * X.as_cost()``, with ``X`` left undeclared.
        """
        # Lazy import: the optimization package loads with its first program
        from minilink.optimization.mathematical_program import MathematicalProgram

        wrt = paths_of(wrt)
        base = self.merged(params)
        z0 = self.pack(base, wrt)
        n_trials = trials_of(self.problem, n_trials)

        # J(z): the objective of the evaluation at the unpacked leaves
        def J(z):
            return self.evaluate(self.unpack(z, wrt, base), key, n_trials).objective

        # g(z) ≥ 0: the margins of the declared sets, one row per trial
        rows = declared_rows(self.problem)

        def g(z):
            report = self.evaluate(self.unpack(z, wrt, base), key, n_trials)
            xp = array_module(report.J)
            return xp.concatenate([getattr(report, row) for row in rows])

        # The objective's gradient, by reverse mode on JAX
        grad_J = None
        if self.backend == BACKEND_JAX:
            jax = ensure_jax_x64()
            grad_J = jax.grad(J)

        lower, upper = box_of(bounds, wrt, base)
        warn_if_infinite(J(z0), self.problem)

        return MathematicalProgram(
            n_z=z0.size,
            J=J,
            g=g if rows else None,
            lower=lower,
            upper=upper,
            grad_J=grad_J,
            backend=self.backend,
            metadata={"wrt": wrt, "z0": z0, "rows": rows},
        )

    def pack(self, params, wrt) -> np.ndarray:
        """The leaves of ``params`` named by ``wrt``, flattened into one vector ``z``."""
        wrt = paths_of(wrt)
        xp = array_module(*(lookup(params, path) for path in wrt))

        leaves = [xp.reshape(lookup(params, path), (-1,)) for path in wrt]

        return xp.concatenate(leaves)

    def unpack(self, z, wrt, params=None) -> dict:
        """``params`` (default :attr:`params`) with the leaves named by ``wrt`` read from ``z``."""
        wrt = paths_of(wrt)
        tree = self.params if params is None else params

        # Each leaf takes the next size(leaf) entries of z, in the leaf's shape
        start = 0
        for path in wrt:
            shape = np.shape(lookup(tree, path))
            size = int(np.prod(shape))
            tree = with_leaf(tree, path, z[start : start + size].reshape(shape))
            start += size

        return tree

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def merged(self, params):
        """:attr:`params`, with the entries of ``params`` overriding."""
        if params is None:
            return self.params

        return merge_tree(self.params, params)

    def realize(self, params, key, n_trials):
        """The trials' starts ``(n_trials, n)`` and parameters (one tree, a leading trial axis where drawn)."""
        problem = self.problem

        # A deterministic problem: its one start, the parameters as they are
        if not problem.is_stochastic:
            xp = array_module(params["x_start"])
            return xp.reshape(params["x_start"], (1, -1)), params

        # x0 ~ p(x0) and each law, n_trials times, from one key on the backend
        x_starts, draws = problem.realize(trial_key(key, self.backend), n_trials)

        # The plant's draws override its entries
        overrides = {"plant": draws["params"]} if draws["params"] else {}

        axes = merge_tree(axes_like(params, None), axes_like(overrides, 0))

        return x_starts, Batched(merge_tree(params, overrides), axes)

    def run(self, params, x_start) -> Trajectory:
        """One run of the loop from the plant state ``x_start``: the loop's trajectory, signals recorded."""
        ev = self.evaluator
        loop = self.loop
        xp = array_module(x_start, *flat_leaves(params))
        dt = self.dt
        N = self.n_steps
        a, b = self.plant_slice

        # The loop's start: its own initial state, the plant's slice at x_start
        x0_loop = xp.asarray(loop.x0, dtype=float)
        x0 = xp.concatenate([x0_loop[:a], xp.reshape(x_start, (-1,)), x0_loop[b:]])

        # The free inputs held at their parameters over every period
        u = self.held_inputs(params, xp)
        u_seq = xp.broadcast_to(u, (N, u.size))

        # x_{k+1} = Φ(x_k, u, t_k), t_k = k dt, k = 0 … N − 1; or the adaptive
        # integral reported on that grid
        loop_params = {sys_id: params[sys_id] for sys_id in self.block_ids}
        ts = xp.arange(N + 1) * dt
        if self.adaptive is None:
            xs = ev.jit.rollout(
                x0,
                u_seq,
                0.0,
                dt,
                loop_params,
                solver=self.solver,
                substeps=self.substeps,
                n_steps=N,
            )
        else:
            xs = integrate_adaptive(ev.rhs_of(loop_params), x0, u, ts, self.adaptive)
        us = xp.broadcast_to(u, (N + 1, u.size))

        # Every block output at every sample, and the plant's input among them
        signals = signals_along(ev, xs, us, ts, loop_params)
        u_plant = gather_samples(self.plant_input, signals, us, xp)

        recorded = {
            f"{sys_id}:{port}": signals[:, sl].T
            for (sys_id, port), sl in ev.plan.output_slices.items()
        }
        recorded[PLANT_INPUT] = u_plant.T

        return Trajectory(
            t=ts,
            x=xs.T,
            u=us.T,
            signals=recorded,
            roles={"plant": {"x": (a, b), "u": PLANT_INPUT}},
        )

    def traces(self) -> bool:
        """Whether one trial's score traces on JAX (the loop does; the cost, ``X``, ``U`` and ``Xf`` may not)."""
        jax = ensure_jax_x64()
        params = self.params
        x_start = np.asarray(self.problem.x_start, dtype=float)

        # An abstract trace of the run and its score: no compile, no numbers
        try:
            price = price_of(self.problem, self.tf)
            jax.eval_shape(
                lambda p, x0: self.score(p, x0, False, price)[0], params, x_start
            )
        except (TypeError, jax.errors.JAXTypeError, jax.errors.ConcretizationTypeError):
            return False

        return True

    def held_inputs(self, params, xp):
        """The loop's boundary input: each free input's parameter, stacked in port order."""
        if not self.free_inputs:
            return xp.zeros(0)

        return xp.concatenate(
            [xp.reshape(params[port], (-1,)) for port in self.free_inputs]
        )

    def score(self, params, x_start, record, price):
        """One trial's row of the record (and its trajectory when ``record``)."""
        traj = self.run(params, x_start)
        plant_run = traj.of("plant")

        row = score_run(self.problem, plant_run, tf=self.tf, params=params, price=price)

        return row, (traj if record else None)

    def run_trials(self, params, x_starts, record, price):
        """The rows of every trial: a ``vmap`` on JAX, a loop on NumPy."""
        if self.backend == BACKEND_JAX:
            return self.run_trials_jax(params, x_starts, record, price)

        return self.run_trials_numpy(params, x_starts, record, price)

    def run_trials_jax(self, params, x_starts, record, price):
        """The trials as one ``vmap`` of the run, jitted once per batch layout and problem parts."""
        jax = ensure_jax_x64()
        register_evaluation_pytree()

        if self.trials_fn is None:
            self.trials_fn = {}

        # The batched axes: a drawn leaf carries the trial axis, the others are shared.
        # The trace reads the problem's cost and sets, so a part replaced on the
        # problem traces anew (the entry holds the parts, so their ids stay theirs)
        tree, axes = batch_axes(params)
        parts = problem_parts(self.problem)
        key = (record, tree_signature(axes), tuple(id(part) for part in parts))
        if key not in self.trials_fn:
            batched = jax.vmap(
                lambda p, x0, c: self.score(p, x0, record, c), in_axes=(axes, 0, None)
            )
            self.trials_fn[key] = (jax.jit(batched), parts)

        trials_fn, _ = self.trials_fn[key]
        rows, trajs = trials_fn(tree, x_starts, price)

        return rows, unbatch(trajs, x_starts.shape[0]) if record else None

    def run_trials_numpy(self, params, x_starts, record, price):
        """The trials one at a time, stacked into the record's arrays."""
        rows, trajs = [], []
        for i, x_start in enumerate(x_starts):
            row, traj = self.score(take_trial(params, i), x_start, record, price)
            rows.append(row)
            trajs.append(traj)

        stacked = {name: np.stack([row[name] for row in rows]) for name in rows[0]}

        return stacked, (trajs if record else None)


@dataclass(frozen=True)
class Evaluation:
    """
    The scores of the runs of one evaluation: one row per run, and their statistics.

    A pytree record (traceable inside a trace, plain arrays outside), each field
    of shape ``(n_runs,)``:

    - ``cost``: ``∫₀^{tf} e^{−ρt} g dt + e^{−ρ tf} h(x(tf))``, ``+inf`` for a run
      that is not finite;
    - ``J``: the cost, plus the problem's price when the run failed or missed the
      goal (with the price switch on);
    - ``feasibility_margin``: the least margin of ``x`` to ``X`` and of ``u`` to
      ``U`` along the run, and ``failed`` (not finite, or a negative margin);
    - ``reachability_margin``: the margin of ``x(tf)`` to ``Xf`` (``+inf`` without
      ``Xf``), and ``reached``;
    - ``x0``: the starts, ``(n_runs, n)``.

    :attr:`objective` is the problem's criterion over ``J``: the one number a
    program minimizes.
    """

    cost: object
    J: object
    feasibility_margin: object
    failed: object
    reachability_margin: object
    reached: object
    x0: object
    trajectories: list | None = None
    criterion: object = "expectation"

    @property
    def n_trials(self) -> int:
        """Number of runs."""
        return int(np.shape(self.J)[0])

    @property
    def objective(self):
        """The criterion over the runs' ``J``: the mean, a CVaR or the worst."""
        return reduce_criterion(self.J, self.criterion)

    @property
    def mean(self):
        """Mean of ``J`` over the runs."""
        return array_module(self.J).mean(self.J)

    @property
    def std(self):
        """Standard deviation of ``J`` over the runs."""
        return array_module(self.J).std(self.J)

    @property
    def worst(self):
        """Largest ``J`` over the runs."""
        return array_module(self.J).max(self.J)

    def cvar(self, alpha):
        """Mean of the worst ``ceil(alpha · n)`` values of ``J``."""
        return reduce_criterion(self.J, ("cvar", alpha))

    @property
    def feasibility_rate(self):
        """Fraction of the runs that stayed finite, in ``X`` and in ``U``."""
        return array_module(self.J).mean(~self.failed)

    @property
    def reachability_rate(self):
        """Fraction of the runs that ended in ``Xf``."""
        return array_module(self.J).mean(self.reached)

    @property
    def success_rate(self):
        """Fraction of the runs that stayed feasible and ended in ``Xf``."""
        return array_module(self.J).mean(self.reached & ~self.failed)

    # Display

    def __str__(self) -> str:
        if self.n_trials == 1:
            notes = []
            if bool(self.failed[0]):
                notes.append("failed")
            if not bool(self.reached[0]):
                notes.append("goal missed")
            note = f" ({', '.join(notes)})" if notes else ""
            return (
                f"J = {float(self.J[0]):.4g}{note}, cost = {float(self.cost[0]):.4g}, "
                f"feasibility margin = {float(self.feasibility_margin[0]):.3g}"
            )

        return (
            f"J over {self.n_trials} runs: objective {float(self.objective):.4g}, "
            f"mean {float(self.mean):.4g}, std {float(self.std):.4g}, "
            f"worst {float(self.worst):.4g}; feasible {100 * float(self.feasibility_rate):.0f}%, "
            f"reached {100 * float(self.reachability_rate):.0f}%"
        )


def evaluate(
    problem,
    controller,
    *,
    dt,
    solver="rk4",
    substeps=1,
    n_trials=None,
    key=0,
    record=False,
    backend="auto",
    episode_length=None,
) -> Evaluation:
    """
    The :class:`Evaluation` of a controller on a problem: ``problem.compile(...).evaluate(...)``.

    One run for a deterministic problem, ``n_trials`` draws for a stochastic one;
    see :class:`ProblemEvaluator` for the loop, the run and the score.
    """
    pe = ProblemEvaluator(
        problem,
        controller,
        dt,
        solver=solver,
        substeps=substeps,
        backend=backend,
        episode_length=episode_length,
    )

    return pe.evaluate(key=key, n_trials=n_trials, record=record)


def score_run(
    problem, traj: Trajectory, *, tf=None, params=None, price=PROBLEM_PRICE
) -> dict:
    """
    The scores of one plant run under the problem: the formula every evaluation reports.

    ``traj`` is the plant's run (its state and its input on a time grid). On the
    trajectory's array module, so it traces under JAX::

        finite              = every x_k and u_k finite
        feasibility_margin  = min over the run of X.margin(x_k, t_k), U.margin(u_k, x_k, t_k)
        failed              = not finite, or feasibility_margin < 0
        reachability_margin = Xf.margin(x(tf), tf)       reached = margin ≥ 0  (no Xf: reached)
        cost                = ∫ e^{−ρt} g dt + e^{−ρ tf} h(x(tf))   (+inf when not finite)
        J                   = cost, or cost + price · (failed or not reached) with the price on

    A run that is not finite is scored on zeros (a safe ``where``), so its
    score stays finite with the price on; its gradient is not defined, since
    the rollout itself overflowed before the ``where``. ``params`` overrides the
    problem's own leaves (``"cost"``, the sets' by role, ``"discount_rate"``);
    ``None`` reads the problem's. ``price`` is the failure price as
    :func:`price_of` resolves it (``None`` the switch off); by default the
    problem's.
    """
    cost_fn = problem.require_cost()
    xp = array_module(traj.x)
    params = problem_leaves(problem) if params is None else params
    rho = params.get("discount_rate", problem.discount_rate)
    finite_horizon = problem.horizon_kind() == "finite"
    X = problem.X
    U = problem.U
    cost_params = params.get("cost")
    X_params = params.get("X")
    U_params = params.get("U")
    Xf_params = params.get("Xf")

    t = traj.t
    x = traj.x.T
    u = traj.u.T
    tf = t[-1] if tf is None else tf
    price = price_of(problem, tf) if price is PROBLEM_PRICE else price

    # Finite: every sample, then the run on zeros where it is not (the safe where)
    finite = xp.all(xp.isfinite(x)) & xp.all(xp.isfinite(u))
    x = xp.where(finite, x, 0.0)
    u = xp.where(finite, u, 0.0)

    # The least margin to X and U along the run, the margin of x(tf) to Xf
    feasibility_margin = xp.minimum(
        least_margin(lambda xk, uk, tk: X.margin(xk, tk, X_params), x, u, t),
        least_margin(lambda xk, uk, tk: U.margin(uk, xk, tk, U_params), x, u, t),
    )
    failed = ~finite | (feasibility_margin < 0.0)
    if problem.Xf is None:
        reachability_margin = xp.asarray(xp.inf)
    else:
        reachability_margin = xp.min(problem.Xf.margin(x[-1], tf, Xf_params))
    reached = reachability_margin >= 0.0

    # cost = ∫₀^{tf} e^{−ρt} g dt + e^{−ρ tf} h(x(tf)), the integral by the trapezoid
    safe_run = Trajectory(t=t, x=x.T, u=u.T)
    _, running = cost_fn.integrate(safe_run, discount_rate=rho, params=cost_params)
    cost = running[-1]
    if finite_horizon:
        cost = cost + xp.exp(-rho * tf) * cost_fn.h(x[-1], tf, cost_params)

    # J = cost, or cost + price · (failed or not reached); a run not finite costs +inf
    if price is None:
        J = xp.where(finite, cost, xp.inf)
    else:
        J = xp.where(finite, cost, 0.0) + xp.where(failed | ~reached, price, 0.0)

    return {
        "cost": xp.where(finite, cost, xp.inf),
        "J": J,
        "feasibility_margin": feasibility_margin,
        "failed": failed,
        "reachability_margin": reachability_margin,
        "reached": reached,
    }


def evaluate_trajectory(problem, traj: Trajectory) -> Evaluation:
    """
    The :class:`Evaluation` of a trajectory you already hold: one row, by :func:`score_run`.

    A loop's run is projected to the plant's by its ``plant`` role
    (``traj.of("plant")``); a trajectory without roles is the plant's own only
    when its ``x`` and ``u`` have the plant's sizes and it records no signals
    (a logged run). On the trajectory's array module.
    """
    run = plant_run_of(problem, traj)
    xp = array_module(run.x)

    row = score_run(problem, run)

    return Evaluation(
        **{name: xp.reshape(value, (1,)) for name, value in row.items()},
        x0=xp.reshape(run.x[:, 0], (1, -1)),
        trajectories=[traj],
        criterion=criterion_of(problem),
    )


def nominal_trajectory(problem, policy, *, dt, tf=None) -> Trajectory:
    """
    The planner's evidence: ``policy`` held over each period of ``dt`` from the problem's start.

    ``u_k = π(x_k, t_k)`` held while one step of the plant takes ``x_k`` to
    ``x_{k+1}``, nominal parameters, on NumPy: the sampled loop a feedback
    planner designs for (its grid, its model), reported beside its policy;
    :func:`evaluate` scores the continuous loop. A stochastic problem runs at
    its mean start; ``tf`` is the run's length for an infinite horizon. ``u`` is
    the plant's command port.
    """
    # Lazy import: the loop roles load with the first rollout
    from minilink.core.feedback import command_port

    nominal = problem.nominal() if problem.is_stochastic else problem
    policy = policy_of(policy)
    plant = nominal.sys
    n_steps = steps_of(horizon_of(nominal, tf), float(dt))

    # The plant's step, the law on the grid, the command's slice of the plant input
    ev = plant.compile(backend=BACKEND_NUMPY, verbose=False)
    t = float(dt) * np.arange(n_steps + 1)
    law = held_law(policy, t)
    command = plant.get_input_port_slice(command_port(plant))
    u_full = np.asarray(plant.get_u_from_input_ports(), dtype=float)

    # u_k = π(x_k, t_k) held over the period, one step to x_{k+1}; the last sample
    # takes its input only
    xs = np.zeros((plant.n, n_steps + 1))
    us = np.zeros((command.stop - command.start, n_steps + 1))
    xs[:, 0] = nominal.x_start
    for k in range(n_steps + 1):
        us[:, k] = law(xs[:, k], t[k])
        if k == n_steps:
            break
        u_full[command] = us[:, k]
        xs[:, k + 1] = ev.step(xs[:, k], u_full, t[k], dt=float(dt))

    return Trajectory(t=t, x=xs, u=us)


def policy_of(controller):
    """The block itself, or the policy of a :class:`~minilink.planning.results.PlanningSolution`."""
    # A real cycle: results imports Evaluation from this module
    from minilink.planning.results import PlanningSolution

    if isinstance(controller, PlanningSolution):
        return controller.policy

    return controller


def closed_loop_of(problem, controller):
    """The loop a :class:`ProblemEvaluator` runs: ``controller`` closed on the problem's plant.

    A built loop (a diagram with a block ``plant`` of the plant's state size) is
    taken as it is; a solution gives its policy; a source block (no input) drives
    the plant's command; any other block is closed by ``controller @ plant``.
    """
    # Lazy imports: the composition tools and the results load with the first loop
    from minilink.core.composition import closed_loop
    from minilink.core.diagram import DiagramSystem
    from minilink.core.system import StepSystem

    plant = problem.sys
    controller = policy_of(controller)

    require_continuous(controller)
    if isinstance(controller, DiagramSystem) and "plant" in controller.subsystems:
        require_plant_block(controller, plant)
        return controller
    if isinstance(plant, StepSystem):
        raise TypeError(
            "the problem's system is a discrete-time StepSystem: the evaluator runs "
            "a continuous loop"
        )
    if int(controller.m) == 0:
        return open_loop_of(controller, plant)

    return closed_loop(controller, plant)


###############################################################################
# Internal machinery
###############################################################################


def plant_run_of(problem, traj):
    """The plant's run in ``traj``: its ``plant`` role, else ``traj`` when it is a logged plant run."""
    if "plant" in traj.roles:
        return traj.of("plant")

    plant = problem.sys
    sizes = (int(traj.x.shape[0]), int(traj.u.shape[0]))
    if traj.signals:
        raise ValueError(
            "a trajectory with recorded signals and no 'plant' role: score the loop's "
            "run from problem.compile(...).trajectory(), or pass traj.of('plant')"
        )
    if sizes != (int(plant.n), int(plant.m)):
        raise ValueError(
            f"the trajectory's (n, m) = {sizes} is not the plant's "
            f"({int(plant.n)}, {int(plant.m)}): pass the plant's run"
        )

    return traj


@dataclass(frozen=True)
class Batched:
    """A parameter tree whose drawn leaves carry a leading trial axis, and its axes tree (``0`` drawn, ``None`` shared)."""

    tree: dict
    axes: dict


def adaptive_options(solver):
    """The SciPy method and tolerances of an adaptive solver label (``"scipy"`` …), or ``None`` for a step."""
    # Lazy import: the simulator's labels load with the first adaptive run
    from minilink.simulation.simulator import _USER_SOLVER_MODES

    if solver not in _USER_SOLVER_MODES or _USER_SOLVER_MODES[solver][0] != "scipy":
        return None

    options = dict(_USER_SOLVER_MODES[solver][1])
    options.pop("use_jac", None)

    return options


def require_numpy_for_adaptive(backend):
    """An adaptive run integrates on NumPy (SciPy): ``"auto"`` resolves to it, JAX is refused."""
    if backend == BACKEND_JAX:
        raise ValueError(
            "an adaptive solver integrates with SciPy on NumPy: pass backend='numpy' "
            "(or 'auto'), or a step solver ('rk4', 'euler') for a traceable run"
        )

    return BACKEND_NUMPY


def integrate_adaptive(f, x0, u, ts, options):
    """``x(t_k)`` on the grid ``ts`` by SciPy's adaptive integrator, the input ``u`` held: ``(N + 1, n)``."""
    # Lazy import: SciPy's integrators load with the first adaptive run
    from scipy.integrate import solve_ivp

    # ẋ = f(x, u, t) from x(0) = x₀, reported at the samples
    solution = solve_ivp(
        lambda t, x: f(x, u, t),
        (float(ts[0]), float(ts[-1])),
        np.asarray(x0, dtype=float),
        t_eval=np.asarray(ts, dtype=float),
        **options,
    )

    return solution.y.T


def held_law(policy, t):
    """``u = π(x, t)`` of a policy block on NumPy: a source by its table on the grid ``t``, a feedback block by its state law."""
    if int(policy.m) == 0:
        return time_law(policy, t)

    return static_law(policy)


def time_law(source, t):
    """``u = π(t)`` of a source block, sampled on the grid ``t`` and held over each period."""
    empty = np.zeros(0)

    # u_k = π(t_k): the source's output at each sample of the grid
    table = np.stack([np.asarray(source.h(empty, empty, float(t_k))) for t_k in t])
    dt = float(t[1] - t[0])

    # u(t) = u_k at the nearest sample, k = round(t / dt)
    return lambda x, t: table[int(round(t / dt))]


def static_law(controller):
    """
    ``u = π(x, t)`` of a static feedback block on NumPy: the state on its measurement port.

    Every other input of the block holds its nominal value (an unconnected
    reference its set point); the time reaches the block, so a time-varying
    gain ``K(t)`` is read at each instant.
    """
    # Lazy import: the loop roles load with the first law
    from minilink.core.feedback import block_roles

    roles = block_roles(controller)
    require_feedback_roles(controller, roles)
    require_static_law(controller)
    ev = controller.compile(backend=BACKEND_NUMPY, verbose=False)

    # The measurement's slice of the block's stacked input vector
    start = 0
    for port_id, port in controller.inputs.items():
        if port_id == roles.measurement:
            measurement = slice(start, start + port.dim)
        start += port.dim
    u_nominal = np.asarray(controller.get_u_from_input_ports(), dtype=float)
    x_ctl = np.zeros(0)

    # u = π(x, t): the state on the measurement port, every other input at nominal
    def law(x, t=0.0):
        u_in = u_nominal.copy()
        u_in[measurement] = x
        return np.asarray(ev.outputs(x_ctl, u_in, float(t))[roles.command])

    return law


def require_feedback_roles(controller, roles):
    """Refuse a block that reads no state or commands no ``u``."""
    if roles.measurement is None or roles.command is None:
        raise ValueError(
            "the policy must read the state on an input named 'x' (or 'y') "
            f"and command 'u'; {controller.name!r} reads {list(controller.inputs)} "
            f"and gives {list(controller.outputs)}"
        )


def require_static_law(controller):
    """Refuse a policy with internal state for the held rollout: a sample holds none."""
    if int(controller.n) > 0:
        raise ValueError(
            "the planner's rollout holds a static law u = pi(x, t); a controller "
            "with internal state is scored by evaluate(problem, controller, dt=)"
        )


def compile_loop(loop, backend):
    """``(backend, evaluator)`` of the loop: JAX when asked, or when ``"auto"`` and it traces."""
    # Lazy import: the compiler loads with the first evaluator
    from minilink.core.compile.compiler import compile, compile_auto

    if backend == "auto":
        backend, evaluator = compile_auto(loop)
    else:
        evaluator = compile(loop, backend=backend)
    if backend == BACKEND_JAX:
        ensure_jax_x64()

    return backend, evaluator


def open_loop_of(source, plant):
    """A source block driving the plant's command: the loop ``{"controller", "plant"}``."""
    # Lazy imports: the diagram tools load with the first loop
    from minilink.core.diagram import DiagramSystem
    from minilink.core.feedback import command_port

    loop = DiagramSystem()
    loop.add_subsystem(source, "controller")
    loop.add_subsystem(plant, "plant")
    loop.connect("controller", next(iter(source.outputs)), "plant", command_port(plant))
    loop.name = f"{source.name} >> {plant.name}"

    return loop


def warn_if_disturbed(problem):
    """Warn when the problem draws disturbances: the evaluated loop holds those ports at nominal."""
    if not problem.is_stochastic or not problem.disturbances:
        return
    ports = sorted(problem.disturbances)

    warnings.warn(
        f"the evaluation does not draw the disturbances on {ports}: the loop reads "
        "those ports at their nominal values. The per-step draws run in the stepped "
        "environment (Sys2Gym.from_problem); a disturbance inside the loop is a source "
        "block (closed_loop(controller, plant, disturbance=WhiteNoise(...)))",
        stacklevel=4,
    )


def require_continuous(controller):
    """Refuse a sampled controller: the evaluator runs a continuous loop."""
    # Lazy import: the system module imports the compile chain, which loads after this one
    from minilink.core.system import StepSystem

    if isinstance(controller, StepSystem):
        raise TypeError(
            f"{getattr(controller, 'name', controller)!r} is a discrete-time "
            "StepSystem: the evaluator runs a continuous loop; a sampled controller "
            "joins it with the hybrid loop"
        )


def require_plant_block(loop, plant):
    """Refuse a built loop whose ``plant`` block is not the problem's plant."""
    block = loop.subsystems["plant"]
    if block is not plant and int(block.n) != int(plant.n):
        raise ValueError(
            "the loop's block 'plant' must be the problem's system (or a box with its "
            f"state): it has n={int(block.n)}, the problem's system n={int(plant.n)}"
        )


def plant_input_sources(loop, evaluator):
    """The sources of the plant's input, port by port: ``(kind, where, dim)``.

    ``kind`` is ``"signal"`` (a block output, ``where`` its slice in the signal
    buffer), ``"input"`` (a free input of the loop, ``where`` its slice in the
    loop's input) or ``"nominal"`` (an unconnected port, ``where`` its value).
    """
    plant = loop.subsystems["plant"]
    wires = loop.connections.get("plant", {})
    output_slices = evaluator.plan.output_slices

    sources = []
    for port_id, port in plant.inputs.items():
        dim = int(port.dim)
        if wires.get(port_id) is None:
            sources.append(("nominal", np.asarray(port.nominal_value, float), dim))
            continue

        src_id, src_port = wires[port_id]
        if src_id == "input":
            sources.append(("input", loop.get_input_port_slice(src_port), dim))
        else:
            sources.append(("signal", output_slices[(src_id, src_port)], dim))

    return tuple(sources)


def signals_along(ev, xs, us, ts, params):
    """Every block output at every sample, shape ``(N + 1, n_signals)``: ``vmap`` on JAX."""
    xp = array_module(xs)
    if xp is np:
        return xp.stack([ev.signals(x, u, t, params) for x, u, t in zip(xs, us, ts)])

    jax = ensure_jax_x64()
    return jax.vmap(lambda x, u, t: ev.signals(x, u, t, params))(xs, us, ts)


def gather_samples(sources, signals, us, xp):
    """The plant's input at every sample, ``(N + 1, m)``, from its port sources."""
    n_samples = signals.shape[0]

    pieces = []
    for kind, where, dim in sources:
        if kind == "signal":
            pieces.append(signals[:, where])
        elif kind == "input":
            pieces.append(us[:, where])
        else:
            pieces.append(xp.broadcast_to(xp.asarray(where), (n_samples, dim)))

    return xp.concatenate(pieces, axis=1)


def least_margin(margin, x, u, t):
    """``min_k min margin(x_k, u_k, t_k)`` over the run (``+inf`` for no rows)."""
    xp = array_module(x)
    if xp is np:
        rows = [xp.reshape(margin(xk, uk, tk), (-1,)) for xk, uk, tk in zip(x, u, t)]
        values = xp.concatenate(rows) if rows else xp.zeros(0)
    else:
        jax = ensure_jax_x64()
        values = jax.vmap(lambda xk, uk, tk: xp.reshape(margin(xk, uk, tk), (-1,)))(
            x, u, t
        )

    if values.size == 0:
        return xp.asarray(xp.inf)

    return xp.min(values)


def problem_leaves(problem) -> dict:
    """The problem's own leaves of a run's tree: ``"cost"``, the sets' parameters by role, ``"x_start"``, ``"discount_rate"``."""
    leaves = {"cost": problem.cost.params if problem.cost is not None else None}
    for role in ("X", "U", "Xf"):
        set_params = getattr(getattr(problem, role), "params", None)
        if set_params is not None:
            leaves[role] = set_params
    leaves["x_start"] = problem.x_start
    leaves["discount_rate"] = problem.discount_rate

    return leaves


def price_of(problem, tf):
    """The price of a failed or missed run of length ``tf``: ``None`` (switch off), the bound (``True``), or the number."""
    price = problem.infeasible_cost
    if price is None:
        return None
    if price is True:
        return problem.cost_bound(tf)

    return price


def problem_parts(problem):
    """The objects a trial's trace reads from the problem: its cost and its sets."""
    return (problem.cost, problem.X, problem.U, problem.Xf)


def horizon_of(problem, episode_length):
    """The length of a run: the finite horizon, else ``episode_length`` (10 s by default)."""
    if problem.horizon_kind() == "finite":
        return float(problem.tf)

    return EPISODE_LENGTH if episode_length is None else float(episode_length)


def steps_of(tf, dt):
    """The number of periods of ``dt`` in ``tf``; refuse a ``dt`` that does not divide it."""
    if not dt > 0.0:
        raise ValueError(f"dt must be positive, got {dt!r}")

    n_steps = int(round(tf / dt))
    if n_steps < 1 or abs(n_steps * dt - tf) > 1e-9 * max(1.0, tf):
        raise ValueError(
            f"dt={dt!r} must divide the horizon tf={tf!r} into a whole number of periods"
        )

    return n_steps


def trials_of(problem, n_trials):
    """The number of runs: one for a deterministic problem, ``n_trials`` (required) for a stochastic one."""
    if not problem.is_stochastic:
        if n_trials not in (None, 1):
            raise ValueError(
                "a deterministic problem is one run: n_trials must be 1 or unset, "
                f"got {n_trials!r}"
            )
        return 1

    if n_trials is None:
        raise ValueError("a stochastic problem needs n_trials, the number of draws")

    return int(n_trials)


def criterion_of(problem):
    """The criterion over the runs: the problem's, or the expectation of one run."""
    return getattr(problem, "criterion", "expectation")


def reduce_criterion(J, criterion):
    """The criterion over ``J``: ``"expectation"``, ``"worst_case"`` or ``("cvar", α)``."""
    xp = array_module(J)

    if criterion == "expectation":
        return xp.mean(J)
    if criterion == "worst_case":
        return xp.max(J)

    # CVaR_α: the mean of the worst ⌈α n⌉ runs
    _, alpha = criterion
    n = J.shape[0]
    k = max(1, math.ceil(round(float(alpha) * n, 9)))  # 0.07 · 100 is 7.000000000000001
    worst = xp.sort(J)[n - k :]

    return xp.mean(worst)


def require_free_names(free_inputs, block_ids):
    """Refuse a free input or a block named like another key of the tree (or a problem leaf)."""
    reserved = {"cost", "X", "U", "Xf", "x_start", "discount_rate"}
    clash = sorted(
        (set(free_inputs) & set(block_ids))
        | ((set(free_inputs) | set(block_ids)) & reserved)
    )
    if clash:
        raise ValueError(
            f"the loop's name(s) {clash} clash in the run's parameter tree (a block, a "
            "free input, or the problem's cost, sets, x_start, discount_rate): rename "
            "the port or the block"
        )


def declared_rows(problem):
    """The program's constraint rows: the feasibility margin for a bounded ``X`` or ``U``, the reachability margin for ``Xf``."""
    rows = []
    if is_bounded(problem.X.bounding_box()) or is_bounded(problem.U.bounding_box()):
        rows.append("feasibility_margin")
    if problem.Xf is not None:
        rows.append("reachability_margin")

    return rows


def is_bounded(box):
    """Whether a box has any finite face (``None``, no box, counts as bounded: a general set)."""
    if box is None:
        return True

    return bool(np.any(np.isfinite(box.lower)) or np.any(np.isfinite(box.upper)))


def box_of(bounds, wrt, base):
    """The program's box on ``z`` from ``{path: (lower, upper)}``; ``None`` sides open."""
    if not bounds:
        return None, None

    unknown = sorted(set(bounds) - set(wrt))
    if unknown:
        raise ValueError(f"bounds name {unknown}, which are not in wrt={list(wrt)}")

    lower, upper = [], []
    for path in wrt:
        size = int(np.size(lookup(base, path)))
        lo, hi = bounds.get(path, (None, None))
        lower.append(
            np.full(size, -np.inf) if lo is None else np.broadcast_to(lo, size)
        )
        upper.append(np.full(size, np.inf) if hi is None else np.broadcast_to(hi, size))

    return np.concatenate(lower).astype(float), np.concatenate(upper).astype(float)


def warn_if_infinite(J0, problem):
    """Warn once when the program's objective is infinite at its start (the price switch off)."""
    if np.isfinite(float(J0)):
        return

    warnings.warn(
        "the program's objective is infinite at its start: a run diverged or left "
        "the horizon unfinished. Set infeasible_cost=True on the problem to price it, "
        "or start from a stabilizing point",
        stacklevel=3,
    )


def paths_of(wrt):
    """``wrt`` as a tuple of paths (one path given as a string)."""
    return (wrt,) if isinstance(wrt, str) else tuple(wrt)


def split_path(path):
    """A dotted path (``"controller.Kp"``; a nested box ``"plant/motor.R"``) as its keys."""
    return path.replace("/", ".").split(".")


def lookup(tree, path):
    """The leaf of ``tree`` at ``path``; refuse a path that names none."""
    node = tree
    for part in split_path(path):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"{path!r} names no leaf of the parameters")
        node = node[part]

    return node


def with_leaf(tree, path, value):
    """A copy of ``tree`` (copied along ``path`` only) with the leaf at ``path`` replaced."""
    head, *rest = split_path(path)
    copy = dict(tree)
    copy[head] = value if not rest else with_leaf(tree[head], ".".join(rest), value)

    return copy


def merge_tree(tree, override):
    """``tree`` with the entries of ``override`` replacing its own, recursively."""
    if isinstance(override, Batched):
        override = override.tree

    merged = dict(tree)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_tree(merged[key], value)
        else:
            merged[key] = value

    return merged


def float_leaves(tree):
    """``tree`` with every leaf a float NumPy array (``None``, an empty entry, kept)."""
    if isinstance(tree, dict):
        return {key: float_leaves(value) for key, value in tree.items()}
    if tree is None:
        return None

    return np.asarray(tree, dtype=float)


def flat_leaves(tree):
    """The leaves of a nested dict, in order."""
    if isinstance(tree, Batched):
        tree = tree.tree
    if isinstance(tree, dict):
        return [leaf for value in tree.values() for leaf in flat_leaves(value)]

    return [tree]


def batch_axes(params):
    """``(tree, axes)``: ``0`` on the drawn leaves (a leading trial axis), ``None`` on the shared ones."""
    if isinstance(params, Batched):
        return params.tree, params.axes

    return params, axes_like(params, None)


def axes_like(tree, value):
    """A tree of the same shape as ``tree``, every leaf ``value``."""
    if isinstance(tree, dict):
        return {key: axes_like(sub, value) for key, sub in tree.items()}

    return value


def tree_signature(axes):
    """A hashable form of an axes tree, a key of the jitted forms."""
    if isinstance(axes, dict):
        return tuple(
            sorted((key, tree_signature(value)) for key, value in axes.items())
        )

    return axes


def take_trial(params, i):
    """The parameters of trial ``i``: each drawn leaf indexed, the shared ones as they are."""
    tree, axes = batch_axes(params)

    return index_tree(tree, axes, i)


def index_tree(tree, axes, i):
    """``tree`` with every leaf whose axis is ``0`` indexed at ``i``."""
    if isinstance(tree, dict):
        return {key: index_tree(tree[key], axes[key], i) for key in tree}

    return tree if axes is None else tree[i]


def unbatch(trajs, n):
    """The batched trajectory of the runs as a list of NumPy trajectories."""
    out = []
    for i in range(n):
        out.append(
            Trajectory(
                t=np.asarray(trajs.t[i]),
                x=np.asarray(trajs.x[i]),
                u=np.asarray(trajs.u[i]),
                signals={k: np.asarray(v[i]) for k, v in trajs.signals.items()},
                roles=dict(trajs.roles),
            )
        )

    return out


def trial_key(key, backend):
    """The draws' key on the backend: a JAX PRNG key, or a NumPy generator."""
    if backend == BACKEND_JAX:
        jax = ensure_jax_x64()
        return jax.random.PRNGKey(key) if isinstance(key, int) else key

    return np.random.default_rng(key)


@lru_cache(maxsize=None)
def register_evaluation_pytree():
    """Register :class:`Evaluation` as a JAX pytree: the arrays as children, the criterion static."""
    jax = ensure_jax_x64()

    fields = (
        "cost",
        "J",
        "feasibility_margin",
        "failed",
        "reachability_margin",
        "reached",
        "x0",
        "trajectories",
    )

    def flatten(record):
        return [getattr(record, name) for name in fields], record.criterion

    def unflatten(criterion, children):
        return Evaluation(**dict(zip(fields, children)), criterion=criterion)

    jax.tree_util.register_pytree_node(Evaluation, flatten, unflatten)
