"""The evaluators of discrete-time systems, ``x_{k+1} = step(x_k, u_k, k; p)``: a leaf and a step diagram."""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import (
    Evaluator,
    added,
    assigned,
    blocks_params,
    check_jax_compatible,
    every_block,
    gather_u,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.compiled import body_of, compiled
from minilink.core.compile.evaluators.dynamic import check_leaf_traces
from minilink.core.compile.evaluators.stepping import iterate, iterate_dense
from minilink.core.compile.plan import SampledPortOperation, StepExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.core.discrete.step_system import StepSystem
from minilink.core.params import lookup

###############################################################################
# Public API
###############################################################################


class DiscreteEvaluator(Evaluator):
    """A compiled discrete-time system: its own ``step``, its ``outputs`` and the ``rollout`` of the steps."""

    def step(self, x, u, k=0, params=None, *, t_start=0.0):
        """``x_{k+1} = step(x, u, k; p)``; ``params=None`` reads the compiled parameters.

        ``t_start`` is the clock's origin of a system on a clock (its tick ``k``
        at ``t_start + k·Δ``); a tick law has no clock and ignores it.
        """
        raise NotImplementedError

    @compiled(static=("n_steps",))
    def rollout(self, x0, u_seq=None, params=None, *, n_steps=None, t_start=0.0):
        """``[x_0, …, x_N]``, shape ``(N + 1, n)``: the steps run over the rows of ``u_seq``.

        ``u_seq`` is ``(N, m)``, one input per step; ``None`` holds the nominal
        input for ``n_steps`` steps. Native arrays in and out: on JAX it is one
        ``lax.scan``, traced and differentiable end to end.
        """
        xp = self.xp
        step = self.step
        n = self.n

        x0 = xp.asarray(x0, dtype=float).reshape(n)
        u_seq = self.input_sequence_of(u_seq, n_steps)
        ks = self.indices(u_seq.shape[0])

        # x_{k+1} = step(x_k, u_k, k), a bare number for n = 1 read as its one entry
        def transition(x, k, u_k):
            x_next = xp.reshape(step(x, u_k, k, params, t_start=t_start), (n,))
            return x_next

        x_samples = iterate(self, transition, x0, (ks, u_seq))

        return x_samples

    def rollout_batch(
        self, x0s, u_sequences=None, *, n_steps=None, params=None, t_start=0.0
    ):
        """Roll out a family of step runs in one ``jax.vmap`` call (JAX).

        ``x0s`` ``(B, n)``; ``u_sequences`` ``None`` (the nominal input for
        ``n_steps``), ``(N, m)`` shared or ``(B, N, m)``; a ``params`` leaf with one
        more dimension than the compiled value is a family. Returns
        ``(B, N + 1, n)``.
        """
        t_start = self.jnp.asarray(t_start, dtype=float)

        return self.batched_rollout(
            lambda x0, u, p, t_start: self.rollout(x0, u, p, t_start=t_start),
            x0s,
            u_sequences,
            n_steps,
            params,
            t_start,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def input_sequence_of(self, u_seq, n_steps):
        """The step inputs as a ``(N, m)`` array; the nominal input repeated when None."""
        xp = self.xp
        if u_seq is None:
            return xp.tile(xp.asarray(self._u_nominal, dtype=float), (int(n_steps), 1))

        u_seq = xp.asarray(u_seq, dtype=float)
        if u_seq.ndim != 2 or u_seq.shape[1] != self.m:
            raise ValueError(f"u_seq must have shape (N, {self.m}), got {u_seq.shape}")

        return u_seq

    def rollout_inputs_of(self, n_steps, u):
        """The rollout's inputs ``(m, n_steps + 1)``: the nominal one, a constant, a sequence or ``u(k)``."""
        if n_steps < 0:
            raise ValueError("n_steps must be nonnegative")

        u_nominal = np.asarray(self._u_nominal, dtype=float).reshape(self.m)

        if u is None:
            u_steps = (
                np.tile(u_nominal, (n_steps, 1)) if n_steps else np.zeros((0, self.m))
            )
        elif callable(u):
            u_steps = np.asarray(
                [np.asarray(u(k), dtype=float).reshape(self.m) for k in range(n_steps)]
            )
        else:
            u_arr = np.asarray(u, dtype=float)
            if u_arr.ndim == 1:
                if u_arr.shape != (self.m,):
                    raise ValueError(f"Constant u must have shape ({self.m},)")
                u_steps = np.tile(u_arr, (n_steps, 1))
            elif u_arr.ndim == 2:
                if u_arr.shape != (n_steps, self.m):
                    raise ValueError(
                        f"u sequence must have shape ({n_steps}, {self.m})"
                    )
                u_steps = u_arr
            else:
                raise ValueError(
                    "u must be None, a constant vector, a sequence, or callable(k)"
                )

        n_samples = n_steps + 1
        u_samples = np.zeros((self.m, n_samples), dtype=float)
        if n_samples:
            if n_steps:
                u_samples[:, :n_steps] = u_steps.T
                u_samples[:, -1] = u_samples[:, n_steps - 1]
            elif self.m:
                u_samples[:, 0] = u_nominal

        return u_samples


class StepEvaluator(DiscreteEvaluator):
    """A compiled :class:`StepSystem` on NumPy or JAX."""

    def __init__(self, system: StepSystem, backend="numpy", verbose=False):
        if not isinstance(system, StepSystem):
            raise TypeError("StepEvaluator requires a StepSystem")
        super().__init__(
            system.n, system.m, system.p, backend, system.get_u_from_input_ports()
        )
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self.params_reader = lambda: system.params

        # A system on a clock steps its seconds face at t_start + k Δ
        self.seconds = isinstance(system, SampledSystem) and not system.counts

        if backend == BACKEND_JAX:
            check_leaf_traces(self, system, system.step, 0, verbose)

        self._jac_setup(
            system,
            "step",
            state_fn=body_of(self, "step"),
            outputs_fn=body_of(self, "outputs"),
        )

    @compiled
    def step(self, x, u, k=0, params=None, *, t_start=0.0):
        xp = self.xp
        params = self._frozen_params if params is None else params
        x, u = xp.asarray(x), xp.asarray(u)
        system = self._system
        if self.seconds:
            return system.update(x, u, t_start + k * system.sample_period, params)
        return system.step(x, u, k, params)

    @compiled
    def outputs(self, x, u, k=0, params=None, *, t_start=0.0):
        xp = self.xp
        params = self._frozen_params if params is None else params
        system = self._system

        # The ports of a system on a clock read seconds, t_k = t_start + k Δ
        t = t_start + k * system.sample_period if self.seconds else k

        return outputs_from_ports(system, xp.asarray(x), xp.asarray(u), t, params)

    @compiled(static=("n_steps",))
    def rollout_dense(self, x0, u_seq=None, params=None, *, n_steps=None, t_start=0.0):
        """The run of a system on a clock with its substeps kept: ``(N·s + 1, n)``, the ticks bit for bit."""
        xp = self.xp
        n = self.n
        system = self._system
        params = self._frozen_params if params is None else params

        x0 = xp.asarray(x0, dtype=float).reshape(n)
        u_seq = self.input_sequence_of(u_seq, n_steps)
        ks = self.indices(u_seq.shape[0])

        def period(x, k, u_k):
            x_next = xp.reshape(self.step(x, u_k, k, params, t_start=t_start), (n,))
            if not system.resolves:
                return x_next, x_next[None, :]
            t_k = t_start + k * system.sample_period
            path = system.update_path(x, u_k, t_k, params)
            points = assigned(xp, path, -1, x_next)
            return x_next, points

        x_samples = iterate_dense(self, period, x0, (ks, u_seq))

        return x_samples


class StepDiagramEvaluator(DiscreteEvaluator):
    """A compiled step diagram on NumPy or JAX: its frozen plan swept in order, stateless."""

    def __init__(
        self, plan: StepExecutionPlan, diagram, backend="numpy", verbose=False
    ):
        p = sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
        super().__init__(
            plan.state_dim, diagram.m, p, backend, diagram.get_u_from_input_ports()
        )
        self.plan = plan
        self.dtype = self.xp.float64
        self._frozen_params = every_block(
            {
                op.sys_id: op.bound_params
                for op in (*plan.port_ops, *plan.step_ops)
                if op.sys_id
            },
            diagram,
        )
        self.params_reader = lambda: blocks_params(diagram, self._frozen_params)
        self._subsystem_ids = tuple(diagram.subsystems)

        if backend == BACKEND_JAX:
            check_plan_traces(self, plan, verbose)

        self._jac_setup(diagram, "step", state_fn=None, outputs_fn=None, plan=plan)

    @compiled
    def step(self, x, u, k=0, params=None, *, t_start=0.0):
        """``x_{k+1}`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return self.next_from_signals(x, u, k, params, signals)

    @compiled
    def tick(self, x, u, k=0, params=None, *, t_start=0.0):
        """``(x_{k+1}, outputs_k)``: the next state and the boundary outputs from one signal pass."""
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        x_next = self.next_from_signals(x, u, k, params, signals)
        outputs = {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }
        return x_next, outputs

    @compiled
    def outputs(self, x, u, k=0, params=None):
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def compute_internal_signals_dict(self, x, u, k=0, params=None) -> dict:
        """Every block output at ``(x, u, k)``, keyed ``"block:port"``."""
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_params(self, params):
        """Refuse a params tree naming a block the diagram does not hold."""
        if params is not None:
            check_diagram_params(params, self._subsystem_ids)

    def float_arrays(self, x, u):
        """``x`` and ``u`` as float arrays of the diagram's dtype."""
        xp = self.xp
        return xp.asarray(x, dtype=self.dtype), xp.asarray(u, dtype=self.dtype)

    def port_signals(self, x, u, k, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        xp = self.xp
        dtype = self.dtype
        signals = xp.zeros(self.plan.signal_dim, dtype=dtype)
        x_arr = xp.asarray(x, dtype=dtype).reshape(self.plan.state_dim)
        params = self._frozen_params if params is None else params
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            y_out = op.compute_func(local_x, local_u, k, params[op.sys_id])
            signals = assigned(xp, signals, op.out_slice, y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = added(xp, signals, op.out_slice, inject[1])
        return signals

    def next_from_signals(self, x, u, k, params, signals):
        """Every block's ``x_{k+1}`` from the filled signal buffer."""
        xp = self.xp
        dtype = self.dtype
        x_new = xp.array(x, dtype=dtype).reshape(self.plan.state_dim)
        params = self._frozen_params if params is None else params
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            x_new = assigned(
                xp,
                x_new,
                op.local_x_slice,
                op.step_func(local_x, local_u, k, params[op.sys_id]),
            )
        return x_new

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)

    def _jac_probe(self, target, wire=None):
        kind, key = target
        signals_of = self.port_signals

        if kind == "state":
            next_of = self.next_from_signals

            def probe(x, u, k, params, delta):
                signals = signals_of(x, u, k, params, inject=(wire, delta))
                return next_of(x, u, k, params, signals)

            return probe

        sl = self.plan.external_output_slices[key] if kind == "port" else key

        def probe(x, u, k, params, delta):
            return signals_of(x, u, k, params, inject=(wire, delta))[sl]

        return probe


###############################################################################
# Internal machinery
###############################################################################


def check_plan_traces(evaluator, plan, verbose):
    """Refuse a step diagram holding a block whose ``step`` or an output port does not trace."""
    jax, jnp = evaluator.jax, evaluator.jnp
    if verbose:
        t0 = time.perf_counter()
        print(
            "[compile] Checking JAX compatibility of step diagram...",
            end="",
            flush=True,
        )

    dummy_x = jnp.zeros(plan.state_dim)
    for op in plan.port_ops:
        check_jax_compatible(
            op.compute_func,
            op.label,
            dummy_x[op.local_x_slice],
            jnp.zeros(op.u_dim),
            0,
            op.bound_params,
            jax,
        )
    for op in plan.step_ops:
        check_jax_compatible(
            op.step_func,
            op.label,
            dummy_x[op.local_x_slice],
            jnp.zeros(op.u_dim),
            0,
            op.bound_params,
            jax,
        )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")


class SampledDiagramEvaluator(StepDiagramEvaluator):
    """
    A compiled sampled diagram on NumPy or JAX: its plan swept once per base tick, on its clock.

    At tick ``k`` the plan forms ``t_k = t_start + k·Δ`` once and hands each block
    its time (seconds, its own count ``k // d``, or the base count); a block of
    divisor ``d > 1`` fires on ``k mod d = 0`` (a ``lax.cond`` on JAX, its
    predicate on the tick alone) and its held outputs are read from the state in
    between. ``params`` is the diagram's nested tree.
    """

    def __init__(self, plan, diagram, backend="numpy", verbose=False):
        p = sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
        DiscreteEvaluator.__init__(
            self,
            plan.state_dim,
            diagram.m,
            p,
            backend,
            diagram.get_u_from_input_ports(),
        )
        self.plan = plan
        self.dtype = self.xp.float64
        self._frozen_params = copy.deepcopy(diagram.params)
        self.params_reader = lambda: diagram.params
        self._subsystem_ids = tuple(diagram.subsystems)
        self.sample_period = plan.base_period

        if backend == BACKEND_JAX:
            check_sampled_plan_traces(self, plan, verbose)

        self._jac_setup(diagram, "step", state_fn=None, outputs_fn=None, plan=plan)

    @compiled
    def step(self, x, u, k=0, params=None, *, t_start=0.0):
        """``X_{k+1}`` of the loop at tick ``k``, its clock's origin at ``t_start``."""
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params, t_start=t_start)
        return self.next_from_signals(x, u, k, params, signals, t_start=t_start)

    @compiled
    def tick(self, x, u, k=0, params=None, *, t_start=0.0):
        """``(X_{k+1}, outputs_k)``: the next state and the boundary outputs from one signal pass."""
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params, t_start=t_start)
        x_next = self.next_from_signals(x, u, k, params, signals, t_start=t_start)
        outputs = {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }
        return x_next, outputs

    @compiled
    def outputs(self, x, u, k=0, params=None, *, t_start=0.0):
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params, t_start=t_start)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    @compiled(static=("n_steps",))
    def rollout_dense(self, x0, u_seq=None, params=None, *, n_steps=None, t_start=0.0):
        """
        The run with the plant's substeps kept: ``(N·S + 1, n)`` at ``t_start + (k + j/S)·Δ``.

        ``S`` is :attr:`~minilink.core.compile.plan.SampledExecutionPlan.dense_points`.
        Each period is the tick of :meth:`rollout`, its state at the period's end
        the same, bit for bit; inside the period the loop's plant (a continuous
        plant integrated by its substeps) shows its substep ends, every other
        block its state at the tick, the input held.
        """
        xp = self.xp
        n = self.n

        x0 = xp.asarray(x0, dtype=float).reshape(n)
        u_seq = self.input_sequence_of(u_seq, n_steps)
        ks = self.indices(u_seq.shape[0])

        def period(x, k, u_k):
            x_next = xp.reshape(self.step(x, u_k, k, params, t_start=t_start), (n,))
            points = self.substep_points(x, u_k, k, params, x_next, t_start)
            return x_next, points

        x_samples = iterate_dense(self, period, x0, (ks, u_seq))

        return x_samples

    def rollout_with_cost(self, x0, u_seq, params, rate, *, n_steps=None, t_start=0.0):
        """
        ``([x_0 … x_N], [C_0 … C_N])``: :meth:`rollout` with the loop's plant carrying its stage cost.

        The plant's period is :meth:`SampledSystem.update_with_cost`, the
        cost's integrand ``rate(x_plant, u_plant, τ)`` integrated beside the
        plant's state by its solver and substeps, the input held; ``C_k`` is
        the running sum of the periods' costs, ``C_0 = 0``. The states are
        :meth:`rollout`'s.
        """
        xp = self.xp
        n = self.n
        plant = self.plan.plant_op

        x0 = xp.asarray(x0, dtype=float).reshape(n)
        u_seq = self.input_sequence_of(u_seq, n_steps)
        ks = self.indices(u_seq.shape[0])

        # z_{k+1} = [x_{k+1}; C_k + c_k]: the loop's tick, its plant advanced with its cost
        def transition(z, k, u_k):
            x, u_f = self.float_arrays(z[:n], u_k)
            signals = self.port_signals(x, u_f, k, params, t_start=t_start)
            costs = []

            def advance(system, local_x, local_u, time, p_op):
                x_i, c = system.update_with_cost(local_x, local_u, time, p_op, rate)
                costs.append(c)
                return x_i

            x_next = self.next_from_signals(
                x, u_f, k, params, signals, t_start=t_start, advance=advance
            )
            require_one_plant_period(costs, plant)
            z_next = xp.concatenate(
                [xp.reshape(x_next, (n,)), xp.reshape(z[n] + costs[0], (1,))]
            )
            return z_next

        z0 = xp.concatenate([x0, xp.zeros(1)])
        z_samples = iterate(self, transition, z0, (ks, u_seq))

        return z_samples[:, :n], z_samples[:, n]

    def signals(self, x, u, k=0, params=None, *, t_start=0.0):
        """The signal buffer at tick ``k``: every block output, flat (``plan.output_slices``)."""
        x, u = self.float_arrays(x, u)
        return self.port_signals(x, u, k, params, t_start=t_start)

    def compute_internal_signals_dict(self, x, u, k=0, params=None, *, t_start=0.0):
        """Every block output at ``(x, u, k)``, keyed ``"path/block:port"`` by its nested path."""
        signals = self.signals(x, u, k, params, t_start=t_start)
        paths = self.plan.paths
        return {
            f"{'/'.join(paths[sys_id])}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def port_signals(self, x, u, k, params, inject=None, *, t_start=0.0):
        """Fill the signal buffer at tick ``k``; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        xp = self.xp
        dtype = self.dtype
        signals = xp.zeros(self.plan.signal_dim, dtype=dtype)
        x_arr = xp.asarray(x, dtype=dtype).reshape(self.plan.state_dim)
        params = self._frozen_params if params is None else params
        t_k = t_start + k * self.sample_period
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            time = time_of(op, k, t_k)
            p_op = lookup(params, op.params_path)
            dim = op.out_slice.stop - op.out_slice.start

            def fire(op=op, local_x=local_x, local_u=local_u, time=time, p_op=p_op):
                y = op.compute_func(local_x, local_u, time, p_op)
                return xp.reshape(xp.asarray(y, dtype=dtype), (dim,))

            if op.divisor == 1:
                y_out = fire()
            else:
                held = x_arr[op.hold_slice]
                y_out = self.fired(k, op.divisor, fire, lambda held=held: held)
            signals = assigned(xp, signals, op.out_slice, y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = added(xp, signals, op.out_slice, inject[1])
        return signals

    def next_from_signals(self, x, u, k, params, signals, *, t_start=0.0, advance=None):
        """Every block's ``x_{k+1}`` and every held output from the filled signal buffer.

        ``advance(system, x, u, t, params)``, when given, takes the loop's plant
        (a continuous plant on the base clock) over the period in place of its
        own step: the host clock runner's adaptive integrator.
        """
        xp = self.xp
        dtype = self.dtype
        x_old = xp.asarray(x, dtype=dtype).reshape(self.plan.state_dim)
        x_new = xp.array(x_old, dtype=dtype)
        params = self._frozen_params if params is None else params
        t_k = t_start + k * self.sample_period
        for op in self.plan.step_ops:
            local_x = x_old[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            time = time_of(op, k, t_k)
            p_op = lookup(params, op.params_path)
            n_i = op.local_x_slice.stop - op.local_x_slice.start

            def fire(op=op, local_x=local_x, local_u=local_u, time=time, p_op=p_op):
                x_i = op.step_func(local_x, local_u, time, p_op)
                return xp.reshape(xp.asarray(x_i, dtype=dtype), (n_i,))

            if advance is not None and op is self.plan.plant_op:
                x_i = advance(op.system, local_x, local_u, time, p_op)
            elif op.divisor == 1:
                x_i = fire()
            else:
                x_i = self.fired(k, op.divisor, fire, lambda local_x=local_x: local_x)
            x_new = assigned(xp, x_new, op.local_x_slice, x_i)

        # The held outputs: every slow block's output this tick, fired or held
        for op in self.plan.port_ops:
            if op.hold_slice is not None:
                x_new = assigned(xp, x_new, op.hold_slice, signals[op.out_slice])
        return x_new

    def plant_input(self, x, u, k=0, params=None, *, t_start=0.0):
        """The input the loop feeds its plant at tick ``k``: the plant's gathered ``u``, held over the period."""
        x, u = self.float_arrays(x, u)
        params = self._frozen_params if params is None else params
        signals = self.port_signals(x, u, k, params, t_start=t_start)
        op = self.plan.plant_op

        return gather_u(op.gather_sources, op.u_dim, signals, u, self.xp, self.dtype)

    def substep_points(self, x, u, k, params, x_next, t_start):
        """The loop's state at the ``S`` points of the period after ``t_k``: ``(S, n)``, the last ``x_{k+1}``."""
        xp = self.xp
        dtype = self.dtype
        S = self.plan.dense_points
        x_old = xp.asarray(x, dtype=dtype).reshape(self.plan.state_dim)
        points = xp.tile(x_old, (S, 1))
        if S == 1:
            return xp.reshape(x_next, (1, -1))

        params = self._frozen_params if params is None else params
        t_k = t_start + k * self.sample_period
        op = self.plan.plant_op
        local_u = self.plant_input(x_old, u, k, params, t_start=t_start)
        path = op.system.update_path(
            x_old[op.local_x_slice], local_u, t_k, lookup(params, op.params_path)
        )

        # The plant at its substep ends, every other block at the tick
        points = assigned(xp, points, (slice(0, S), op.local_x_slice), path)
        points = assigned(xp, points, S - 1, x_next)

        return points

    def fired(self, k, d, fire, hold):
        """``fire()`` on the ticks ``k mod d = 0``, ``hold()`` otherwise: a branch, never a ``where``.

        One branch runs per tick; under ``vmap`` the ``lax.cond`` is lowered to
        a select, which evaluates both (the result is the same).
        """
        if self.xp is np:
            return fire() if k % d == 0 else hold()

        return self.jax.lax.cond(k % d == 0, fire, hold)

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)


def time_of(op, k, t_k):
    """What a block reads as time at tick ``k``: ``t_k`` in seconds, its own count, or the base count."""
    if op.face == "seconds":
        return t_k
    if op.face == "count":
        return k // op.divisor

    return k


def check_sampled_plan_traces(evaluator, plan, verbose):
    """Refuse a sampled diagram holding a block whose step, update or an output port does not trace."""
    jax, jnp = evaluator.jax, evaluator.jnp
    dummy_x = jnp.zeros(plan.state_dim)
    for op in (*plan.port_ops, *plan.step_ops):
        func = op.compute_func if isinstance(op, SampledPortOperation) else op.step_func
        check_jax_compatible(
            func,
            op.label,
            dummy_x[op.local_x_slice],
            jnp.zeros(op.u_dim),
            0.0 if op.face == "seconds" else 0,
            op.bound_params,
            jax,
        )
    if verbose:
        print("[compile] The sampled plan traces on JAX.")


def require_one_plant_period(costs, plant):
    """Refuse a sampled loop whose plant did not take its one period: its stage cost has no owner."""
    if len(costs) != 1:
        raise ValueError(
            f"the loop's plant {getattr(plant, 'sys_id', plant)!r} took {len(costs)} periods "
            "in one tick; the stage cost needs exactly one"
        )
