"""The problem evaluator: a controller on a problem, compiled once, run to the horizon and scored."""

from __future__ import annotations

import warnings

import numpy as np

from minilink.core.assembly import is_box
from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    ensure_jax_x64,
)
from minilink.core.compile.evaluators.stepping import require_step_knobs
from minilink.core.compile.plan import PLANT_INPUT
from minilink.core.distributions import child_seed
from minilink.core.options import require_one_of
from minilink.core.params import (
    Batched,
    DeviceTree,
    canonical_tree,
    flat_leaves,
    flatten,
    float_leaves,
    lookup,
    merge_tree,
    set_leaf,
    split_path,
    with_leaf,
)
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import (
    RUNNING_COST,
    Evaluation,
    criterion_of,
    diverged_cost_of,
    price_of,
    problem_leaves,
    register_evaluation_pytree,
    score_run,
)
from minilink.planning.problems import seeds_of, source_id, stack_leaves

BACKENDS = ("auto", BACKEND_JAX, BACKEND_NUMPY)
EPISODE_LENGTH = 10.0  # s, the run of an infinite-horizon problem when none is named


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
    dt : float, optional
        Sample period of the run: the free inputs are held over it and the run is
        recorded on its grid. A sampled controller (``pid % 0.01``, a loop on a
        clock) runs on its own base period: ``dt`` omitted reads it, and a ``dt``
        that differs is refused.
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
        dt=None,
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

        # The task and the loop that runs it: a continuous diagram, or a sampled
        # loop on its own clock
        self.problem = problem
        self.loop = closed_loop_of(problem, controller)
        self.sampled = is_sampled(self.loop)
        if self.sampled:
            dt = sampled_dt_of(self.loop, dt)
            require_loop_knobs(solver, substeps, self.adaptive)
        self.solver = solver
        self.substeps = int(substeps)

        # The grid: N periods of dt to the horizon
        self.dt = float(require_dt(dt))
        self.tf = horizon_of(problem, episode_length)
        self.n_steps = steps_of(self.tf, self.dt)

        # The loop, compiled once on its backend
        self.backend, self.evaluator = compile_loop(self.loop, backend)

        # The static layout of the run: the plant's state slice, the sources of its
        # input, the recorded signals and the free inputs
        self.plant_slice = self.loop.state_index["plant"]
        self.plant_input = plant_input_sources(self.loop, self.evaluator, self.sampled)
        self.block_ids = tuple(self.loop.subsystems)
        self.start_ids = stateful_blocks_of(self.loop)
        self.draws = problem.is_stochastic or self.loop.is_random
        self.free_inputs = tuple(self.loop.inputs)
        require_free_names(self.free_inputs, self.block_ids)
        require_no_x0_key(self.loop, self.start_ids)

        # The forms generated once: the batched trial, jitted on JAX; the price
        # and bound of each value of the cost's numbers
        self.trials_fn = None
        self.run_fn = None
        self.price_cache = {}
        self.on_device = DeviceTree()

        # "auto" keeps JAX only when the whole score traces (the cost and the sets too)
        if backend == "auto" and self.backend == BACKEND_JAX and not self.traces():
            self.backend, self.evaluator = compile_loop(self.loop, BACKEND_NUMPY)

    @property
    def params(self) -> dict:
        """
        The run's parameters as one nested tree, fresh from the objects.

        ``{block_id: block params}`` for every block of the loop, a stateful block
        other than the plant with its start as the leaf ``"x0"`` (``controller.x0``,
        ``observer.x0``; the plant's start is the problem's ``x_start``); the
        problem's own leaves (``"cost"``, the parameters of the sets that hold any by
        role, ``"x_start"``, ``"discount_rate"``); and each free input of the loop at
        its nominal value. Every leaf a float array, so ``jax.grad`` differentiates
        the whole tree.
        """
        loop = self.loop
        problem = self.problem

        tree = {sys_id: loop.subsystems[sys_id].params for sys_id in self.block_ids}
        for sys_id in self.start_ids:
            block = loop.subsystems[sys_id]
            tree[sys_id] = {**(block.params or {}), "x0": block.x0}
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

        # One jitted run on JAX (its scan and its recorded signals one dispatch)
        if self.backend == BACKEND_JAX and self.adaptive is None:
            return self.run_compiled(params, x_start)

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
        n_trials = trials_of(self.draws, n_trials)

        # The draws, the price of a failure and the cost bound of a run not finite,
        # then the runs and their scores, batched on JAX
        x_starts, trial_params = self.realize(params, key, n_trials)
        price, bound = self.price_and_bound(params)
        rows, trajectories = self.run_trials(
            trial_params, x_starts, record, price, bound
        )

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
        require_no_seed(wrt)
        if tunes(wrt, "x_start") and self.problem.is_stochastic:
            raise ValueError(
                "cannot put 'x_start' in wrt on a stochastic problem: the starts are drawn"
            )

        base = self.merged(params)
        z0 = self.pack(base, wrt)
        n_trials = trials_of(self.draws, n_trials)
        rows = declared_rows(self.problem)

        # The draws, the price and the cost bound stay fixed when the start and
        # the cost are not among the tuned leaves
        freeze = not tunes(wrt, "x_start") and not tunes(wrt, "cost")
        if freeze:
            x_starts, trial_params = self.realize(base, key, n_trials)
            price, bound = self.price_and_bound(base)
            batched = isinstance(trial_params, Batched)
            frozen_tree = trial_params.tree if batched else trial_params
            frozen_axes = trial_params.axes if batched else None

            def overlay(z):
                tree = self.unpack(z, wrt, frozen_tree)
                return Batched(tree, frozen_axes) if batched else tree

            def report_of(z):
                trial_rows, _ = self.run_trials(
                    overlay(z), x_starts, False, price, bound
                )
                return Evaluation(
                    **trial_rows,
                    x0=x_starts,
                    trajectories=None,
                    criterion=criterion_of(self.problem),
                )

        else:

            def report_of(z):
                return self.evaluate(self.unpack(z, wrt, base), key, n_trials)

        def J(z):
            return report_of(z).objective

        def g(z):
            report = report_of(z)
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
        """:attr:`params`, with the entries of ``params`` overriding, each in its default's dtype and shape (one trace for every value)."""
        defaults = self.params
        if params is None:
            return defaults

        return canonical_tree(merge_tree(defaults, params), defaults)

    def device_tree(self, tree):
        """``tree`` on the device: put there once and kept while its values repeat, so a repeat transfers nothing."""
        jax = ensure_jax_x64()

        return self.on_device.get(tree, lambda same: same, jax.device_put)

    def run_compiled(self, params, x_start):
        """:meth:`run` jitted once per problem parts: the recorded run in one dispatch."""
        jax = ensure_jax_x64()

        parts = problem_parts(self.problem)
        if self.run_fn is None or any(
            a is not b for a, b in zip(self.run_fn[1], parts)
        ):
            self.run_fn = (jax.jit(self.run), parts)
        run, _ = self.run_fn

        return run(self.device_tree(params), np.asarray(x_start, dtype=float))

    def price_and_bound(self, params):
        """
        The price of a failure and the cost bound of a run not finite, at the cost's numbers of ``params``.

        The bound follows the overridden weights: it is computed once per value
        of the cost's leaves and cached by it (and by the problem's parts and
        price switch, which may be replaced after the compile), since each bound
        samples ``g`` over the boxes. Under a trace the weights have no value, and the bound
        is the problem's own (a price is a step: it carries no gradient).
        """
        problem = self.problem
        cost_params = params.get("cost")
        numbers = numbers_key(cost_params)
        if numbers is None:
            cost_params, numbers = None, "own"

        # The key: the price switch and the numbers; the entry holds the parts it
        # was taken on, so a part replaced since takes a new bound
        parts = problem_parts(problem)
        key = (problem.infeasible_cost, problem.discount_rate, numbers)
        cached = self.price_cache.get(key)
        if cached is None or any(a is not b for a, b in zip(cached[0], parts)):
            price = price_of(problem, self.tf, cost_params)
            bound = diverged_cost_of(problem, self.tf, price, cost_params)
            cached = self.price_cache[key] = (parts, price, bound)

        return cached[1:]

    def realize(self, params, key, n_trials):
        """
        The trials' starts ``(n_trials, n)`` and parameters (one tree, a leading trial axis where drawn).

        A stochastic problem draws the starts and its laws from ``key``, and each
        trial ``i`` realizes the loop's random blocks under
        ``child_seed(key, "trial i")``, whether they are the problem's sources or
        blocks of the loop itself (a block's own seed is only its default
        realization, never an evaluation's): an integer key gives the
        same draws on NumPy and JAX. A run with nothing random is the one run.
        """
        problem = self.problem
        loop = self.loop
        xp = array_module(params["x_start"])

        # Nothing random: the one start, the parameters as they are
        if not self.draws:
            return xp.reshape(params["x_start"], (1, -1)), params

        # x0 ~ p(x0), each law and each source's seed, n_trials times: one tree,
        # drawn on the host from an integer key whatever the backend; a point
        # start, the same for every trial
        overrides = {}
        x_starts = xp.tile(xp.asarray(params["x_start"], dtype=float), (n_trials, 1))
        if problem.is_stochastic:
            tree = problem.realize(key, n_trials)
            if problem.start_law is not None:
                x_starts = tree["x_start"]
            for path in problem.drawn_paths()[1:]:
                set_leaf(overrides, path, lookup(tree, path))

        # Each trial's seeds of the loop's random blocks override theirs
        if loop.is_random:
            overrides = merge_tree(overrides, trial_seeds(loop, key, n_trials))

        axes = merge_tree(axes_like(params, None), axes_like(overrides, 0))

        return x_starts, Batched(merge_tree(params, overrides), axes)

    def loop_start(self, params, x_start, xp):
        """The loop's start ``x(0)``: the plant at ``x_start``, every other stateful block at its ``x0`` leaf, a held output at its own."""
        loop = self.loop

        pieces = []
        for sys_id in self.block_ids:
            if sys_id == "plant":
                pieces.append(xp.reshape(xp.asarray(x_start, dtype=float), (-1,)))
            elif sys_id in self.start_ids:
                pieces.append(xp.reshape(params[sys_id]["x0"], (-1,)))
        pieces.append(xp.asarray(getattr(loop, "hold_x0", np.zeros(0)), dtype=float))

        x0 = xp.concatenate(pieces)

        return x0

    def run(self, params, x_start) -> Trajectory:
        """One run of the loop from the plant state ``x_start``: the loop's trajectory, signals recorded."""
        ev = self.evaluator
        loop = self.loop
        xp = array_module(x_start, *flat_leaves(params))
        dt = self.dt
        N = self.n_steps

        # The loop's start: each block's x0 leaf, the plant's slice at x_start
        x0 = self.loop_start(params, x_start, xp)

        # The free inputs held at their parameters over every period
        u = self.held_inputs(params, xp)
        u_seq = xp.broadcast_to(u, (N, u.size))

        # x_{k+1} = Φ(x_k, u, t_k), t_k = t_start + k dt, k = 0 … N − 1, with the
        # plant's stage cost integrated beside the state; or the adaptive
        # integral reported on that grid
        loop_params = {
            sys_id: block_params_of(loop, params, sys_id) for sys_id in self.block_ids
        }
        loop_params = integer_seeds(loop, loop_params)
        t_start = params["t_start"]
        ts = t_start + xp.arange(N + 1) * dt
        rate = self.cost_rate(params)
        if self.sampled:
            xs, running, signals = ev.rollout_with_cost(
                x0, u_seq, loop_params, rate, n_steps=N, t_start=t_start, signals=True
            )
        elif self.adaptive is None:
            xs, running = ev.rollout_with_cost(
                x0,
                u_seq,
                t_start,
                dt,
                loop_params,
                rate,
                solver=self.solver,
                substeps=self.substeps,
                n_steps=N,
            )
        else:
            f_aug = ev.rhs_with_cost(loop_params, "plant", rate)
            z0 = xp.concatenate([xp.asarray(x0, dtype=float), xp.zeros(1)])
            zs = integrate_adaptive(f_aug, z0, u, ts, self.adaptive)
            xs, running = zs[:, :-1], zs[:, -1]
        us = xp.broadcast_to(u, (N + 1, u.size))

        # Every block output at every sample, and the plant's input among them: a
        # sampled loop's from the run's own signal pass
        if not self.sampled:
            signals = signals_along(ev, xs, us, ts, loop_params)
        u_plant = gather_samples(self.plant_input, signals, us, xp)

        recorded = {
            f"{signal_name_of(ev, sys_id, self.sampled)}:{port}": signals[:, sl].T
            for (sys_id, port), sl in ev.plan.output_slices.items()
        }
        recorded[PLANT_INPUT] = u_plant.T
        recorded[RUNNING_COST] = xp.reshape(running, (1, -1))

        return Trajectory(
            t=ts,
            x=xs.T,
            u=us.T,
            signals=recorded,
            roles={"plant": {"x": self.plant_slice, "u": PLANT_INPUT}},
        )

    def cost_rate(self, params):
        """The plant's discounted cost rate ``e^{−ρ(τ − t_start)} g(x, u, τ)`` on the run's tree: the integrand the run carries."""
        g = self.problem.require_cost().g
        cost_params = params["cost"]
        rho = params["discount_rate"]
        t_start = params["t_start"]

        # ċ = e^{−ρ(τ − t_start)} g(x_plant, u_plant, τ)
        def rate(x, u, tau):
            xp = array_module(x, u, tau)
            c_dot = xp.exp(-rho * (tau - t_start)) * g(x, u, tau, cost_params)
            return c_dot

        return rate

    def traces(self) -> bool:
        """Whether one trial's score traces on JAX (the loop does; the cost, ``X``, ``U`` and ``Xf`` may not)."""
        jax = ensure_jax_x64()
        params = self.params
        x_start = np.asarray(self.problem.x_start, dtype=float)

        # An abstract trace of the run and its score: no compile, no numbers
        try:
            price = price_of(self.problem, self.tf)
            bound = diverged_cost_of(self.problem, self.tf, price)
            jax.eval_shape(
                lambda p, x0: self.score(p, x0, False, price, bound)[0],
                params,
                x_start,
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

    def score(self, params, x_start, record, price, bound):
        """One trial's row of the record (and its trajectory when ``record``)."""
        traj = self.run(params, x_start)
        plant_run = traj.of("plant")

        row = score_run(
            self.problem,
            plant_run,
            tf=self.tf,
            params=params,
            price=price,
            cost_bound=bound,
            running_cost=traj.signals[RUNNING_COST][0],
        )

        return row, (traj if record else None)

    def run_trials(self, params, x_starts, record, price, bound):
        """The rows of every trial: a ``vmap`` on JAX, a loop on NumPy."""
        if self.backend == BACKEND_JAX:
            return self.run_trials_jax(params, x_starts, record, price, bound)

        return self.run_trials_numpy(params, x_starts, record, price, bound)

    def run_trials_jax(self, params, x_starts, record, price, bound):
        """The trials as one ``vmap`` of the run, jitted once per batch layout and problem parts."""
        jax = ensure_jax_x64()
        register_evaluation_pytree()

        if self.trials_fn is None:
            self.trials_fn = {}

        # The batched axes: a drawn leaf carries the trial axis, the others are shared.
        # The trace reads the problem's cost and sets, so a part replaced on the
        # problem traces anew, in place of the entry it replaces
        tree, axes = batch_axes(params)
        tree = self.device_tree(tree)
        parts = problem_parts(self.problem)
        layout = (record, tree_signature(axes))
        cached = self.trials_fn.get(layout)
        if cached is None or any(a is not b for a, b in zip(cached[1], parts)):
            batched = jax.vmap(
                lambda p, x0, c, b: self.score(p, x0, record, c, b),
                in_axes=(axes, 0, None, None),
            )
            self.trials_fn[layout] = (jax.jit(batched), parts)

        trials_fn, _ = self.trials_fn[layout]
        rows, trajs = trials_fn(tree, x_starts, price, bound)

        return rows, unbatch(trajs, x_starts.shape[0]) if record else None

    def run_trials_numpy(self, params, x_starts, record, price, bound):
        """The trials one at a time, stacked into the record's arrays."""
        rows, trajs = [], []
        for i, x_start in enumerate(x_starts):
            row, traj = self.score(take_trial(params, i), x_start, record, price, bound)
            rows.append(row)
            trajs.append(traj)

        stacked = {name: np.stack([row[name] for row in rows]) for name in rows[0]}

        return stacked, (trajs if record else None)


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


def nominal_trajectory(problem, policy, *, dt, tf=None) -> Trajectory:
    """
    The planner's evidence: ``policy`` held over each period of ``dt`` from the problem's start.

    ``u_k = π(x_k, t_k)`` held while one step of the problem's step face
    takes ``x_k`` to ``x_{k+1}`` (one RK4 step per period), nominal
    parameters, on NumPy, from ``t_start``: the sampled loop a feedback planner
    designs for (its grid, its model), reported beside its policy; the
    planner's evaluation scores the same law held on the plant's own clock. A stochastic problem runs at its mean start; ``tf`` is the run's
    length for an infinite horizon. ``u`` is the plant's command port.
    """
    # Lazy import: the loop roles load with the first rollout
    from minilink.core.roles import command_port

    nominal = problem.nominal() if problem.is_stochastic else problem
    policy = policy_of(policy)
    plant = nominal.sys
    n_steps = steps_of(horizon_of(nominal, tf), float(dt))

    # The face's step, the law on the grid, the command's slice of the plant input
    face = nominal.compile(dt=float(dt))
    plant_params = face.params["plant"]
    t = nominal.t_start + float(dt) * np.arange(n_steps + 1)
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
        xs[:, k + 1] = face.step_at(xs[:, k], u_full, t[k], plant_params)

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
    from minilink.core.closed_loop import closed_loop
    from minilink.core.diagram import DiagramSystem
    from minilink.core.discrete.step_diagram import StepDiagramSystem
    from minilink.core.discrete.step_system import StepSystem

    plant = problem.sys
    controller = policy_of(controller)

    require_continuous_or_clocked(controller)
    require_no_stateless_mpc(controller)
    is_loop = isinstance(controller, (DiagramSystem, StepDiagramSystem))
    if is_loop and "plant" in controller.subsystems:
        require_plant_block(controller, plant)
        require_no_problem_sources(problem)
        return controller
    if isinstance(plant, StepSystem):
        raise TypeError(
            "the problem's system is a discrete-time StepSystem: the evaluator runs "
            "a continuous loop"
        )
    sources = dict(problem.disturbances)
    if int(controller.m) == 0:
        return with_sources(open_loop_of(controller, plant), sources)

    # The loop closed_loop builds with the same sources: d and n under its role names
    loop = closed_loop(
        controller,
        plant,
        disturbance=sources.pop("d", False),
        noise=sources.pop("n", False),
    )

    return with_sources(loop, sources)


###############################################################################
# Internal machinery
###############################################################################


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
    from minilink.core.roles import block_roles

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
    """``(backend, evaluator)`` of the loop: JAX when asked, or when ``"auto"`` and it traces; a JAX form reused."""
    # Lazy import: the compiler loads with the first evaluator
    from minilink.core.compile.cache import compiled_of
    from minilink.core.compile.compiler import compile_auto

    if backend == "auto":
        backend, evaluator = compile_auto(loop, reuse=True)
    else:
        evaluator = compiled_of(loop, backend)
    if backend == BACKEND_JAX:
        ensure_jax_x64()

    return backend, evaluator


def open_loop_of(source, plant):
    """A source block driving the plant's command: the loop ``{"controller", "plant"}``."""
    # Lazy imports: the diagram tools load with the first loop
    from minilink.core.diagram import DiagramSystem
    from minilink.core.roles import command_port

    loop = DiagramSystem()
    loop.add_subsystem(source, "controller")
    loop.add_subsystem(plant, "plant")
    loop.connect("controller", next(iter(source.outputs)), "plant", command_port(plant))
    loop.name = f"{source.name} >> {plant.name}"

    return loop


def with_sources(loop, sources):
    """The loop with each disturbance source wired onto its plant port, under its block id (:func:`~minilink.planning.problems.source_id`)."""
    for port, block in sources.items():
        sys_id = source_id(port)
        loop.add_subsystem(block, sys_id)
        loop.connect(sys_id, next(iter(block.outputs)), "plant", port)

    return loop


def require_no_problem_sources(problem):
    """Refuse the problem's disturbance sources on a loop built by hand: that loop wires its own."""
    if problem.disturbances:
        raise ValueError(
            f"the problem's disturbances {sorted(problem.disturbances)} cannot be wired "
            "into a loop built by hand: wire the source blocks in the loop, or pass "
            "the controller and let the problem close the loop"
        )


def require_continuous_or_clocked(controller):
    """Refuse a tick law with no clock: the loop runs in seconds, continuous or on a clock."""
    # Lazy import: the system module imports the compile chain, which loads after this one
    from minilink.core.discrete.step_system import StepSystem

    if isinstance(controller, StepSystem) and controller.sample_period is None:
        raise TypeError(
            f"{getattr(controller, 'name', controller)!r} is a tick law with no clock: "
            "put it on one (controller % dt) and the loop runs on that clock"
        )


def require_no_stateless_mpc(controller):
    """Refuse the pure MPC with no clock: closed as a continuous law, it would re-solve at every integrator stage."""
    # Lazy import: the MPC loads with the first one evaluated
    from minilink.planning.trajectory_optimization.mpc.predictive import (
        PurePredictiveController,
    )

    if isinstance(controller, PurePredictiveController):
        raise TypeError(
            "an MPC solves once per period on its clock: the pure MPC closed as a "
            "continuous law would re-solve at every integrator stage; put it on its "
            "clock (mpc % dt)"
        )


def is_sampled(loop) -> bool:
    """Whether the loop is a sampled diagram, run on its own clock."""
    # Lazy import: the sampled diagram loads with the first sampled loop
    from minilink.core.discrete.sampled_diagram import SampledDiagramSystem

    return isinstance(loop, SampledDiagramSystem)


def sampled_dt_of(loop, dt):
    """A sampled loop's period: its base period, ``dt`` omitted or equal to it; else refused."""
    base = loop.sample_period
    if dt is not None and abs(float(dt) - base) > 1e-12 * base:
        raise ValueError(
            f"the loop runs on its own clock, Δ = {base:g} s: omit dt, or pass dt={base:g}"
        )

    return base


def require_loop_knobs(solver, substeps, adaptive):
    """Refuse integration knobs on a sampled loop: its plant's solver and substeps are the plant block's."""
    if adaptive is not None or solver != "rk4" or int(substeps) != 1:
        raise ValueError(
            "a sampled loop integrates its plant as its plant block says "
            "(discretize(plant, dt, solver=, substeps=)); leave solver and substeps "
            "at their defaults"
        )


def require_dt(dt):
    """Refuse a continuous loop with no sample period: its run is recorded on the grid of ``dt``."""
    if dt is None:
        raise TypeError(
            "a continuous loop needs dt, the sample period of its run: "
            "problem.compile(controller, dt=0.01)"
        )

    return dt


def require_plant_block(loop, plant):
    """Refuse a built loop whose ``plant`` block is not the problem's plant."""
    block = loop.subsystems["plant"]
    if block is not plant and int(block.n) != int(plant.n):
        raise ValueError(
            "the loop's block 'plant' must be the problem's system (or a box with its "
            f"state): it has n={int(block.n)}, the problem's system n={int(plant.n)}"
        )


def plant_input_sources(loop, evaluator, sampled=False):
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
        elif sampled:
            path, leaf_port = leaf_source(loop, src_id, src_port)
            sources.append(
                ("signal", evaluator.plan.output_slice_of(path, leaf_port), dim)
            )
        else:
            sources.append(("signal", output_slices[(src_id, src_port)], dim))

    return tuple(sources)


def leaf_source(diagram, sys_id, port_id, prefix=()):
    """The leaf block's path and port behind a block's output: a box's boundary output followed in."""
    block = diagram.subsystems[sys_id]
    if not is_box(block):
        return (*prefix, sys_id), port_id

    inner_id, inner_port = block.connections["output"][port_id]

    return leaf_source(block, inner_id, inner_port, (*prefix, sys_id))


def signal_name_of(ev, sys_id, sampled):
    """A recorded block's name: its id, or its path through the boxes of a sampled loop."""
    if not sampled:
        return sys_id

    return "/".join(ev.plan.paths[sys_id])


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


def trials_of(draws, n_trials):
    """The number of runs: one when nothing is random, ``n_trials`` (required) when the problem or the loop draws."""
    if not draws:
        if n_trials not in (None, 1):
            raise ValueError(
                "a deterministic problem is one run: n_trials must be 1 or unset, "
                f"got {n_trials!r}"
            )
        return 1

    if n_trials is None:
        raise ValueError(
            "a stochastic problem (or a loop with a random block) needs n_trials, "
            "the number of draws"
        )

    return int(n_trials)


def require_no_seed(wrt):
    """Refuse a seed among the tuned leaves: an integer leaf, never a decision."""
    seeds = [path for path in wrt if split_path(path)[-1] == "seed"]
    if seeds:
        raise ValueError(
            f"cannot put {seeds} in wrt: a seed is an integer leaf, a parameter of the "
            "program, never a decision; fix it, or average over seeds with n_trials"
        )


def numbers_key(tree):
    """A hashable key of the values of a tree's leaves, ``None`` when one has no value (a tracer)."""
    entries = []
    for path, leaf in flatten(tree or {}):
        try:
            value = np.asarray(leaf, dtype=float)
        except TypeError:  # a tracer has no value
            return None
        entries.append((path, value.shape, value.tobytes()))

    return tuple(entries)


def stateful_blocks_of(loop):
    """The ids of the loop's blocks other than the plant that hold a state: each has an ``x0`` leaf."""
    return tuple(
        sys_id
        for sys_id, block in loop.subsystems.items()
        if sys_id != "plant" and block.n > 0
    )


def require_no_x0_key(loop, start_ids):
    """Refuse a stateful block whose own params name ``"x0"``: the tree's ``x0`` leaf is its start."""
    for sys_id in start_ids:
        if "x0" in (loop.subsystems[sys_id].params or {}):
            raise ValueError(
                f"the block {sys_id!r} has a parameter named 'x0', the name of its "
                "start in the evaluator's tree: rename the parameter"
            )


def block_params_of(loop, params, sys_id):
    """A block's entry of the run's tree as the block reads it: its ``x0`` leaf taken out."""
    entry = params[sys_id]
    if not isinstance(entry, dict) or "x0" not in entry:
        return entry

    rest = {key: value for key, value in entry.items() if key != "x0"}
    if not rest and loop.subsystems[sys_id].params is None:
        return None

    return rest


def require_free_names(free_inputs, block_ids):
    """Refuse a free input or a block named like another key of the tree (or a problem leaf)."""
    reserved = {"cost", "X", "U", "Xf", "x_start", "t_start", "discount_rate"}
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


def tunes(wrt, name):
    """Whether ``wrt`` names ``name`` or a leaf under it (``"cost"``, ``"cost.Q"``)."""
    return any(path == name or path.startswith(name + ".") for path in wrt)


def trial_seeds(loop, key, n_trials):
    """The seeds of the loop's random blocks for each trial, stacked on a leading trial axis.

    Trial ``i`` is the loop's realization under ``child_seed(key, "trial i")``,
    the library's cipher on the host, so the seeds do not depend on the backend.
    """
    require_integer_key(key)

    seeds = []
    for i in range(n_trials):
        realized = loop.realize(child_seed(int(key), f"trial {i}"))
        seeds.append(seeds_of(loop, realized))

    return stack_leaves(seeds)


def require_integer_key(key):
    """Refuse a key that is not an integer: the loop's random blocks are realized from one."""
    if isinstance(key, bool) or not isinstance(key, (int, np.integer)):
        raise TypeError(
            "the loop holds random blocks, whose seeds are realized from an integer "
            f"key: pass key as an int, got {type(key).__name__}"
        )


def integer_seeds(system, params):
    """``params`` with the seed of every random block of ``system`` an integer (the tree holds floats)."""
    # Lazy import: the diagram tools load with the first loop
    from minilink.core.wiring import WiredDiagramMixin

    if not system.is_random:
        return params
    if isinstance(system, WiredDiagramMixin):
        return {
            sys_id: integer_seeds(system.subsystems[sys_id], value)
            for sys_id, value in params.items()
        }

    seed = params["seed"]
    if seed is None:
        return params
    xp = array_module(seed)

    return dict(params, seed=xp.asarray(seed).astype(int))


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
