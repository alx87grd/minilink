"""
JAX evaluation backend for compiled diagrams.

Provides :class:`JaxEvaluator`, a JAX-compatible evaluator that mirrors
:class:`~minilink.compile.numpy_backend.NumpyEvaluator` but uses
``jax.numpy`` and functional array updates (``.at[].set()``) so that all
operations are traceable and differentiable.

**Limitations**:
- Subsystem ``f`` / ``h`` / port compute functions must be JAX-traceable
  (no in-place mutation, no Python-side branching on traced values).
- Lazy import: JAX is imported at class instantiation, not at module level.
"""

from __future__ import annotations

from minilink.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)


# ── JAX signal gathering ────────────────────────────────────────────


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


# ── Evaluator ────────────────────────────────────────────────────────


class JaxEvaluator:
    """JAX-compatible evaluator for a compiled diagram.

    All operations use functional array updates so they are traceable
    by JAX's transformation system (``jit``, ``grad``, ``vmap``, etc.).

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.compile.compiler.build_execution_plan`.

    Examples
    --------
    >>> evaluator = compile_diagram(diagram, backend="jax")
    >>> dx = evaluator.compute_dx(x_jax, u_jax, t)
    >>> jit_dx = evaluator.get_jit_compute_dx()
    >>> dx = jit_dx(x_jax, u_jax, t)
    """

    def __init__(self, plan: ExecutionPlan):
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

    # ── Dtype inference ──────────────────────────────────────────────

    def _infer_dtype(self, x, u):
        """Best-effort dtype inference from input arrays."""
        jnp = self._jnp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        return dtype if dtype is not None else jnp.float32

    # ── Public interface ─────────────────────────────────────────────

    def compute_dx(self, x, u, t=0.0):
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
            dx_piece = op.f_func(local_x, local_u, t)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

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
        """Evaluate and return the full internal signal buffer (JAX-traceable).

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

    # ── JIT wrappers ─────────────────────────────────────────────────

    def get_jit_compute_dx(self):
        """Return a ``jax.jit``-wrapped version of :meth:`compute_dx`."""
        return self._jax.jit(self.compute_dx)

    def get_jit_compute_outputs(self):
        """Return a ``jax.jit``-wrapped version of :meth:`compute_outputs`."""
        return self._jax.jit(self.compute_outputs)

    # ── Private ──────────────────────────────────────────────────────

    def _compute_port_signals(self, x, u, t, dtype):
        """Evaluate all port signals in topological order (JAX-traceable)."""
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)

        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(
                op.gather_sources, op.u_dim, signals, u, jnp, dtype
            )
            y_out = op.compute_func(local_x, local_u, t)
            signals = signals.at[op.out_slice].set(y_out)
        return signals
