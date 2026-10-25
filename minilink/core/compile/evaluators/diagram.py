"""The evaluator of a :class:`~minilink.core.diagram.DiagramSystem`: one signal sweep, then every block's ``dx``."""

from __future__ import annotations

import time

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import (
    ContinuousEvaluator,
    added,
    assigned,
    check_jax_compatible,
    gather_u,
)
from minilink.core.compile.evaluators.compiled import compiled
from minilink.core.compile.plan import ExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.system import DynamicSystem

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
        self._frozen_params = {
            op.sys_id: op.bound_params
            for op in (*plan.port_ops, *plan.state_ops)
            if op.sys_id
        }
        self._subsystem_ids = tuple(diagram.subsystems)

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
        # An integer x0 still gets float dx; float inputs keep their width.
        return xp.result_type(dtype, float)

    def port_signals(self, x, u, t, dtype, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        xp = self.xp
        signals = xp.zeros(self.plan.signal_dim, dtype=dtype)
        params = self._frozen_params if params is None else params
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            y_out = op.compute_func(local_x, local_u, t, params[op.sys_id])
            signals = assigned(xp, signals, op.out_slice, y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = added(xp, signals, op.out_slice, inject[1])
        return signals

    def dx_from_signals(self, x, u, t, dtype, params, signals):
        """Every block's ``dx`` from the filled signal buffer."""
        xp = self.xp
        dx = xp.zeros(self.plan.state_dim, dtype=dtype)
        params = self._frozen_params if params is None else params
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
