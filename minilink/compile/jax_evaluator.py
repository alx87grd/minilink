"""
JAX evaluation backends for compiled systems.

Provides two evaluators:

- :class:`JaxLeafEvaluator` — for a single (non-diagram) System.
- :class:`JaxDiagramEvaluator` — for a compiled DiagramSystem.

Both inherit from :class:`~minilink.compile.evaluator.DynamicsEvaluator`.

**Limitations**:
- Subsystem ``f`` / ``h`` / port compute functions must be JAX-traceable
  (no in-place mutation, no Python-side branching on traced values).
- Lazy import: JAX is imported at class instantiation, not at module level.
"""

from __future__ import annotations

import copy
from typing import Any, Callable

from minilink.compile.evaluator import DynamicsEvaluator
from minilink.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)


# =====================================================================
# Leaf evaluator
# =====================================================================


class JaxLeafEvaluator(DynamicsEvaluator):
    """Compiled evaluator for a single System using JAX.

    Core callables (``f``, ``h``, ``f_p``, ``h_p``, ``f_ivp``, ``h_ivp``)
    are JIT-compiled at construction and warm-started with dummy data so that
    the first real call incurs no compilation latency.

    Parameters
    ----------
    system : System
        The system to wrap.  Its ``params`` are deep-copied at construction
        (frozen), and nominal input port values are snapshotted.
    """

    def __init__(self, system):
        import jax
        import jax.numpy as jnp

        self.jax = jax
        self.jnp = jnp

        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports(0))

        # Store references to the raw system functions
        f_raw = system.f
        h_raw = system.h
        frozen_p = self._frozen_params
        u_nom = self._u_nominal

        # --- JIT-compile core callables ----------------------------------
        self._jit_f = jax.jit(lambda x, u, t: f_raw(x, u, t, frozen_p))
        self._jit_h = jax.jit(lambda x, u, t: h_raw(x, u, t, frozen_p))
        self._jit_f_p = jax.jit(lambda x, u, t, p: f_raw(x, u, t, p))
        self._jit_h_p = jax.jit(lambda x, u, t, p: h_raw(x, u, t, p))
        self._jit_f_ivp = jax.jit(lambda x, t: f_raw(x, u_nom, t, frozen_p))
        self._jit_h_ivp = jax.jit(lambda x, t: h_raw(x, u_nom, t, frozen_p))

        # --- Warm-start with dummy data ----------------------------------
        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        self._jit_f(dummy_x, dummy_u, dummy_t)
        self._jit_h(dummy_x, dummy_u, dummy_t)
        self._jit_f_p(dummy_x, dummy_u, dummy_t, frozen_p)
        self._jit_h_p(dummy_x, dummy_u, dummy_t, frozen_p)
        self._jit_f_ivp(dummy_x, dummy_t)
        self._jit_h_ivp(dummy_x, dummy_t)

    # -- Standard tier (frozen params) ------------------------------------

    def f(self, x, u, t=0.0):
        return self._jit_f(x, u, t)

    def h(self, x, u, t=0.0):
        return self._jit_h(x, u, t)

    def outputs(self, x, u, t=0.0):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, self._frozen_params)
        return result

    # -- Parametric tier (caller-supplied params) -------------------------

    def f_p(self, x, u, t, params):
        return self._jit_f_p(x, u, t, params)

    def h_p(self, x, u, t, params):
        return self._jit_h_p(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, params)
        return result

    # -- IVP tier (override ABC defaults with JIT versions) ---------------

    def f_ivp(self, x, t=0.0):
        return self._jit_f_ivp(x, t)

    def h_ivp(self, x, t=0.0):
        return self._jit_h_ivp(x, t)


# =====================================================================
# JAX signal gathering
# =====================================================================


def _gather_u_jax(gather_sources, u_dim, signals, u, jnp, dtype):
    """Assemble the local input vector using JAX operations.

    Unlike the NumPy version, this uses ``jnp.concatenate`` on collected
    pieces (no in-place mutation) so the computation stays JAX-traceable.

    Parameters
    ----------
    gather_sources : tuple of (source_type, source_value, dim)
    u_dim : int
    signals : jax array
        Internal signal buffer.
    u : jax array
        Diagram external input vector.
    jnp : module
        ``jax.numpy`` (passed explicitly to avoid module-level import).
    dtype : jax dtype
        Output dtype for constant arrays.

    Returns
    -------
    jax array
    """
    if u_dim == 0:
        return jnp.array([], dtype=dtype)

    pieces = []
    for src_type, src_val, dim in gather_sources:
        if src_type == INTERNAL_SIGNAL:
            pieces.append(signals[src_val])
        elif src_type == NOMINAL:
            pieces.append(jnp.asarray(src_val, dtype=dtype))
        elif src_type == EXTERNAL_INPUT:
            pieces.append(u[src_val])
        else:
            raise RuntimeError(f"Unknown source_type={src_type}")

    return jnp.concatenate(pieces, axis=0) if pieces else jnp.array([], dtype=dtype)


# =====================================================================
# Diagram evaluator
# =====================================================================


class JaxDiagramEvaluator(DynamicsEvaluator):
    """JAX-compatible evaluator for a compiled diagram.

    Inherits from :class:`DynamicsEvaluator`.  Only ``f`` is mapped to the
    ABC interface; ``h`` / ``outputs`` and the parametric tier raise
    ``NotImplementedError``.  Use diagram-specific methods
    (``compute_outputs``, ``compute_internal_signals``,
    ``compute_internal_signals_dict``) for port-level access.

    All operations use functional array updates so they are traceable
    by JAX's transformation system (``jit``, ``grad``, ``vmap``, etc.).

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.compile.compiler.build_execution_plan`.
    diagram : DiagramSystem
        The source diagram (used only to snapshot ``m`` and ``_u_nominal``).

    Examples
    --------
    >>> evaluator = compile_diagram(diagram, backend="jax")
    >>> dx = evaluator.f(x_jax, u_jax, t)
    >>> f_jit = evaluator.get_f_jit()
    >>> dx = f_jit(x_jax, u_jax, t)
    """

    def __init__(self, plan: ExecutionPlan, diagram):
        try:
            import jax
            import jax.numpy as jnp
        except ImportError as e:
            raise ImportError(
                "JAX is required for the 'jax' backend. "
                "Install with: pip install jax jaxlib"
            ) from e

        self.plan = plan
        self._jax = jax
        self._jnp = jnp

        self.n = plan.state_dim
        self.m = diagram.m
        self.p = plan.signal_dim
        self.backend = "jax"
        self._frozen_params = None  # per-op binding, not diagram-level
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports(0))

    # ── Dtype inference ──────────────────────────────────────────────

    def _infer_dtype(self, x, u):
        """Best-effort dtype inference from input arrays."""
        jnp = self._jnp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        return dtype if dtype is not None else jnp.float32

    # ── ABC: Standard tier ──────────────────────────────────────────

    def f(self, x, u, t=0.0):
        """Compute the diagram's state derivative vector (JAX-traceable).

        Parameters
        ----------
        x : jax array, shape (state_dim,)
            Global state vector.
        u : jax array, shape (m,)
            External input vector.
        t : float or jax scalar
            Current time.

        Returns
        -------
        jax array, shape (state_dim,)
            State derivative vector ``dx/dt``.
        """
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)

        signals = self._compute_port_signals(x, u, t, dtype)

        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            dx_piece = op.f_func(local_x, local_u, t, op.bound_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def h(self, x, u, t=0.0):
        raise NotImplementedError(
            "Diagram h() not supported. Use compute_outputs() for port-level access."
        )

    def outputs(self, x, u, t=0.0):
        raise NotImplementedError(
            "Diagram outputs() not supported. "
            "Use compute_outputs() or compute_internal_signals_dict()."
        )

    # ── ABC: Parametric tier ────────────────────────────────────────

    def f_p(self, x, u, t, params):
        raise NotImplementedError("Parametric tier not supported for diagrams yet.")

    def h_p(self, x, u, t, params):
        raise NotImplementedError("Parametric tier not supported for diagrams yet.")

    def outputs_p(self, x, u, t, params):
        raise NotImplementedError("Parametric tier not supported for diagrams yet.")

    # ── JIT convenience ─────────────────────────────────────────────

    def get_f_jit(self):
        """Return a ``jax.jit``-wrapped version of :meth:`f`."""
        return self._jax.jit(self.f)

    # ── Diagram-specific methods ────────────────────────────────────

    def compute_outputs(self, x, u, t=0.0, ports=None):
        """Evaluate selected output ports (JAX-traceable).

        Parameters
        ----------
        x : jax array, shape (state_dim,)
        u : jax array, shape (m,)
        t : float or jax scalar
        ports : list of (sys_id, port_id), optional

        Returns
        -------
        jax array
            Concatenated output signals.
        """
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)

        signals = self._compute_port_signals(x, u, t, dtype)

        if ports is None:
            slices = list(self.plan.output_slices.values())
        else:
            slices = [self.plan.output_slices[key] for key in ports]

        if not slices:
            return jnp.array([], dtype=dtype)
        out_pieces = [signals[s] for s in slices]
        return jnp.concatenate(out_pieces, axis=0)

    def compute_internal_signals(self, x, u, t=0.0):
        """Evaluate and return the full internal signal buffer (flat JAX array).

        Parameters
        ----------
        x : jax array, shape (state_dim,)
        u : jax array, shape (m,)
        t : float or jax scalar

        Returns
        -------
        jax array, shape (signal_dim,)
        """
        dtype = self._infer_dtype(x, u)
        return self._compute_port_signals(x, u, t, dtype)

    def compute_internal_signals_dict(self, x, u, t=0.0):
        """Evaluate all port signals and return as a labelled dict.

        Returns
        -------
        dict mapping ``"sys_id:port_id"`` -> jax array
            One entry per output port in topological order.
        """
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals(x, u, t, dtype)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    # ── Private ──────────────────────────────────────────────────────

    def _compute_port_signals(self, x, u, t, dtype):
        """Evaluate all port signals in topological order (JAX-traceable)."""
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)

        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            y_out = op.compute_func(local_x, local_u, t, op.bound_params)
            signals = signals.at[op.out_slice].set(y_out)
        return signals
