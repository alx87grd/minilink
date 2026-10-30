"""The evaluator of a :class:`~minilink.core.diagram.DiagramSystem`: one signal sweep, then every block's ``dx``."""

from __future__ import annotations

import time

import numpy as np

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import (
    ContinuousEvaluator,
    added,
    assigned,
    blocks_params,
    check_jax_compatible,
    every_block,
    gather_u,
    gathered_on_host,
)
from minilink.core.compile.evaluators.compiled import compiled
from minilink.core.compile.evaluators.stepping import (
    held_inputs,
    held_step_body,
    iterate,
    step_body,
)
from minilink.core.compile.plan import ExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.system import DynamicSystem

# The common buffer dtype, returned without a promotion
FLOAT64 = np.dtype(np.float64)

###############################################################################
# Public API
###############################################################################


class DiagramEvaluator(ContinuousEvaluator):
    """A compiled diagram on NumPy or JAX: its frozen plan swept in order, stateless."""

    def __init__(self, plan: ExecutionPlan, diagram, backend="numpy", verbose=False):
        p = sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
        super().__init__(
            plan.state_dim, diagram.m, p, backend, diagram.get_u_from_input_ports()
        )
        self.plan = plan
        self._frozen_params = every_block(
            {
                op.sys_id: op.bound_params
                for op in (*plan.port_ops, *plan.state_ops)
                if op.sys_id
            },
            diagram,
        )
        self.params_reader = lambda: blocks_params(diagram, self._frozen_params)
        self._subsystem_ids = tuple(diagram.subsystems)

        # The port operations of the blocks that hold a signal over a sample period
        self.op_holds = tuple(
            holds_signal(diagram.subsystems[op.sys_id]) for op in plan.port_ops
        )

        if backend == BACKEND_JAX:
            check_blocks_trace(self, diagram, verbose)

        self._jac_setup(diagram, "dynamic", state_fn=None, outputs_fn=None, plan=plan)

    @compiled
    def f(self, x, u, t=0.0, params=None):
        """``dx`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params)
        return self.dx_from_signals(x, u, t, dtype, params, signals)

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    @compiled
    def signals(self, x, u, t=0.0, params=None):
        """The internal signal buffer at ``(x, u, t)``: every block output."""
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        return self.port_signals(x, u, t, dtype, params)

    @property
    def holds(self) -> bool:
        """Whether a block holds a signal over a sample period: a noise source, a stochastic policy."""
        return any(self.op_holds)

    def f_held(self, x, u, t, t_held, params=None):
        """``dx`` with the held blocks read at ``t_held``, the step's start, and every other block at ``t``."""
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params, t_held=t_held)
        return self.dx_from_signals(x, u, t, dtype, params, signals)

    def f_and_block(self, x, u, t, params, sys_id, t_held=None):
        """``(dx, x_b, u_b)``: the loop's ``dx`` and one block's own state and input, from one signal pass."""
        self.require_params(params)
        xp = self.xp
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params, t_held=t_held)
        dx = self.dx_from_signals(x, u, t, dtype, params, signals)

        # The block's state slice and its gathered input, read off the same buffer
        op = next(op for op in self.plan.state_ops if op.sys_id == sys_id)
        x_b = x[op.local_x_slice]
        u_b = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)

        return dx, x_b, u_b

    def rhs_with_cost(self, params, sys_id, rate, t_held=None):
        """``ż = [dx; rate(x_b, u_b, t)]`` of the loop augmented with one block's stage cost; the held blocks read at ``t_held`` when given."""
        xp = self.xp

        def f_aug(z, u, t):
            dx, x_b, u_b = self.f_and_block(z[:-1], u, t, params, sys_id, t_held)
            c_dot = xp.reshape(rate(x_b, u_b, t), (1,))
            z_dot = xp.concatenate([dx, c_dot])
            return z_dot

        return f_aug

    def rollout_with_cost(
        self,
        x0,
        u_seq,
        t_start,
        dt,
        params,
        rate,
        *,
        sys_id="plant",
        solver="rk4",
        substeps=1,
        n_steps=None,
    ):
        """
        ``([x_0 … x_N], [C_0 … C_N])``: :meth:`rollout` with the running stage cost of one block integrated beside the state.

        ``C(t) = ∫_{t_start}^t rate(x_b, u_b, τ) dτ``, ``x_b`` and ``u_b`` the
        block's own state and input at every stage of the same scheme and
        substeps (one signal pass per stage), the input held; the states are
        :meth:`rollout`'s.
        """
        xp = self.xp
        n = self.n
        f_aug = self.rhs_with_cost(params, sys_id, rate)

        x0 = xp.asarray(x0, dtype=float).reshape(n)
        u_seq = self.input_sequence_of(u_seq, n_steps, "zoh")
        u_start, u_end = held_inputs(u_seq, "zoh")
        ks = self.indices(u_start.shape[0])

        # A held block read once per substep, at its start
        def f_aug_at(s):
            return self.rhs_with_cost(params, sys_id, rate, t_held=s)

        # z_{k+1} = Φ(z_k, u_k, t_k) for z = [x; C], t_k = t_start + k dt
        def period(z, k, ua, ub):
            t_k = t_start + k * dt
            if self.holds:
                return held_step_body(f_aug_at, z, ua, ub, t_k, dt, solver, substeps)
            return step_body(f_aug, z, ua, ub, t_k, dt, solver, substeps)

        z0 = xp.concatenate([x0, xp.zeros(1)])
        z_samples = iterate(self, period, z0, (ks, u_start, u_end))

        return z_samples[:, :n], z_samples[:, n]

    def compute_internal_signals_dict(self, x, u, t=0.0, params=None):
        """Every block output at ``(x, u, t)``, keyed ``"block:port"``."""
        signals = self.signals(x, u, t, params)
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

    def dtype_of(self, x, u):
        """The floating dtype of the buffers: ``x``'s (else ``u``'s), at least float."""
        xp = self.xp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        if dtype is None:
            return xp.float32 if self.backend == BACKEND_JAX else float
        if dtype == FLOAT64:
            return FLOAT64
        # An integer x0 still gets float dx; float inputs keep their width.
        return xp.result_type(dtype, float)

    def port_signals(self, x, u, t, dtype, params, inject=None, t_held=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire; a held block reads ``t_held`` when given."""
        xp = self.xp
        signals = xp.zeros(self.plan.signal_dim, dtype=dtype)
        params = self._frozen_params if params is None else params
        times = self.op_times(t, t_held)

        # NumPy: the buffer written in place, the hot path of a host integration
        if xp is np and inject is None:
            for op, t_op in zip(self.plan.port_ops, times):
                local_u = gathered_on_host(
                    op.gather_sources, op.u_dim, signals, u, dtype
                )
                signals[op.out_slice] = op.compute_func(
                    x[op.local_x_slice], local_u, t_op, params[op.sys_id]
                )
            return signals

        for op, t_op in zip(self.plan.port_ops, times):
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            y_out = op.compute_func(local_x, local_u, t_op, params[op.sys_id])
            signals = assigned(xp, signals, op.out_slice, y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = added(xp, signals, op.out_slice, inject[1])
        return signals

    def op_times(self, t, t_held):
        """The time each port operation reads: ``t_held`` for a held block when given, ``t`` otherwise."""
        if t_held is None:
            return [t] * len(self.op_holds)

        return [t_held if holds else t for holds in self.op_holds]

    def rhs_of(self, params, t_held=None):
        """``f(x, u, t)`` on the compiled parameters, or on ``params``; the held blocks read at ``t_held`` when given."""
        if t_held is None:
            return super().rhs_of(params)

        return lambda x, u, t: self.f_held(x, u, t, t_held, params)

    def dx_from_signals(self, x, u, t, dtype, params, signals):
        """Every block's ``dx`` from the filled signal buffer."""
        xp = self.xp
        dx = xp.zeros(self.plan.state_dim, dtype=dtype)
        params = self._frozen_params if params is None else params

        # NumPy: every block's dx written in place
        if xp is np:
            for op in self.plan.state_ops:
                local_u = gathered_on_host(
                    op.gather_sources, op.u_dim, signals, u, dtype
                )
                dx[op.local_x_slice] = op.f_func(
                    x[op.local_x_slice], local_u, t, params[op.sys_id]
                )
            return dx

        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            dx = assigned(
                xp,
                dx,
                op.local_x_slice,
                op.f_func(local_x, local_u, t, params[op.sys_id]),
            )
        return dx

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)

    def _jac_probe(self, target, wire=None):
        kind, key = target
        dtype_of = self.dtype_of
        signals_of = self.port_signals

        if kind == "state":
            dx_of = self.dx_from_signals

            def probe(x, u, t, params, delta):
                dtype = dtype_of(x, u)
                signals = signals_of(x, u, t, dtype, params, inject=(wire, delta))
                return dx_of(x, u, t, dtype, params, signals)

            return probe

        sl = self.plan.external_output_slices[key] if kind == "port" else key

        def probe(x, u, t, params, delta):
            dtype = dtype_of(x, u)
            return signals_of(x, u, t, dtype, params, inject=(wire, delta))[sl]

        return probe


###############################################################################
# Internal machinery
###############################################################################


def check_blocks_trace(evaluator, diagram, verbose):
    """Refuse a diagram holding a block whose ``f`` or an output port does not trace."""
    jax, jnp = evaluator.jax, evaluator.jnp
    if verbose:
        t0 = time.perf_counter()
        n_blocks = len(diagram.subsystems)
        print(
            f"[compile] Checking JAX compatibility of {n_blocks} blocks...",
            end="",
            flush=True,
        )

    for sys_id, sys in diagram.subsystems.items():
        dummy_x = jnp.zeros(sys.n)
        dummy_u = jnp.zeros(sys.m)
        params = getattr(sys, "params", {})
        if isinstance(sys, DynamicSystem):
            label = f"{sys_id} ({sys.name})"
            check_jax_compatible(sys.f, label, dummy_x, dummy_u, 0.0, params, jax)
        for port_id, port in sys.outputs.items():
            label = f"{sys_id}:{port_id} ({sys.name})"
            check_jax_compatible(
                port.compute, label, dummy_x, dummy_u, 0.0, params, jax
            )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")


def holds_signal(block) -> bool:
    """Whether a leaf block holds its output over a sample period (its ``solver_info["sample_period"]``)."""
    is_leaf = not hasattr(block, "subsystems")

    return is_leaf and "sample_period" in block.solver_info
