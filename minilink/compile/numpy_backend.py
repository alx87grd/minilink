"""
NumPy evaluation backend for compiled diagrams.

Provides :class:`NumpyEvaluator`, a stateless evaluator that uses pure NumPy
to compute state derivatives and output signals from an
:class:`~minilink.compile.execution_plan.ExecutionPlan`.

Each call allocates its own signal buffer, so this evaluator is **thread-safe**
(concurrent calls do not share mutable state).
"""

from __future__ import annotations

import numpy as np

from minilink.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)


# ── Shared helper ────────────────────────────────────────────────────


def _gather_u(
    gather_sources: tuple[tuple[int, object, int], ...],
    u_dim: int,
    signals: np.ndarray,
    u: np.ndarray,
) -> np.ndarray:
    """Assemble the local input vector from pre-mapped sources.

    This is THE single implementation of the signal-gathering dispatch
    that was previously copy-pasted across the codebase.

    Parameters
    ----------
    gather_sources : tuple of (source_type, source_value, dim)
        Recipe built by :func:`~minilink.compile.compiler._build_gather_sources`.
    u_dim : int
        Total dimension of the local input vector.
    signals : np.ndarray
        The internal signal buffer (output of all port evaluations so far).
    u : np.ndarray
        The diagram's external input vector.

    Returns
    -------
    np.ndarray
        The assembled local input vector for the subsystem.
    """
    if u_dim == 0:
        return np.array([])

    local_u = np.empty(u_dim)
    idx = 0
    for src_type, src_val, dim in gather_sources:
        if src_type == INTERNAL_SIGNAL:
            local_u[idx : idx + dim] = signals[src_val]
        elif src_type == NOMINAL:
            local_u[idx : idx + dim] = src_val
        elif src_type == EXTERNAL_INPUT:
            local_u[idx : idx + dim] = u[src_val]
        else:
            raise RuntimeError(f"Unknown source_type={src_type}")
        idx += dim
    return local_u


# ── Evaluator ────────────────────────────────────────────────────────


class NumpyEvaluator:
    """Stateless NumPy evaluator for a compiled diagram.

    All public methods allocate a fresh signal buffer per call, making
    concurrent evaluation safe.

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.compile.compiler.build_execution_plan`.

    Examples
    --------
    >>> from minilink.compile import compile_diagram
    >>> evaluator = compile_diagram(diagram, backend="numpy")
    >>> dx = evaluator.compute_dx(x, u, t)
    >>> y  = evaluator.compute_outputs(x, u, t, ports=[("plant", "y")])
    """

    def __init__(self, plan: ExecutionPlan):
        self.plan = plan

    # ── Public interface ─────────────────────────────────────────────

    def compute_dx(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0
    ) -> np.ndarray:
        """Compute the diagram's state derivative vector.

        Parameters
        ----------
        x : np.ndarray, shape (state_dim,)
            Global state vector.
        u : np.ndarray, shape (m,)
            External input vector.
        t : float
            Current time.

        Returns
        -------
        np.ndarray, shape (state_dim,)
            State derivative vector ``dx/dt``.
        """
        signals = self._compute_port_signals(x, u, t)

        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t)
        return dx

    def compute_outputs(
        self,
        x: np.ndarray,
        u: np.ndarray,
        t: float = 0.0,
        ports: list[tuple[str, str]] | None = None,
    ) -> np.ndarray:
        """Evaluate selected output ports and return their concatenation.

        Parameters
        ----------
        x : np.ndarray, shape (state_dim,)
            Global state vector.
        u : np.ndarray, shape (m,)
            External input vector.
        t : float
            Current time.
        ports : list of (sys_id, port_id), optional
            Which ports to return.  If ``None``, returns all ports
            concatenated in the order they appear in ``output_slices``.

        Returns
        -------
        np.ndarray
            Concatenated output signals.
        """
        signals = self._compute_port_signals(x, u, t)

        if ports is None:
            slices = list(self.plan.output_slices.values())
        else:
            slices = [self.plan.output_slices[key] for key in ports]

        if not slices:
            return np.array([])
        return np.concatenate([signals[s] for s in slices])

    def compute_internal_signals(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0
    ) -> np.ndarray:
        """Evaluate and return the full internal signal buffer.

        This is useful for reconstructing all subsystem output signals
        at a given time step (e.g., for plotting internal signals).

        Parameters
        ----------
        x : np.ndarray, shape (state_dim,)
            Global state vector.
        u : np.ndarray, shape (m,)
            External input vector.
        t : float
            Current time.

        Returns
        -------
        np.ndarray, shape (signal_dim,)
            The complete internal signal buffer after evaluation.
        """
        return self._compute_port_signals(x, u, t)

    # ── Private ──────────────────────────────────────────────────────

    def _compute_port_signals(
        self, x: np.ndarray, u: np.ndarray, t: float
    ) -> np.ndarray:
        """Evaluate all port signals in topological order.

        Returns the filled signal buffer (fresh allocation each call).
        """
        signals = np.zeros(self.plan.signal_dim)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            signals[op.out_slice] = op.compute_func(local_x, local_u, t)
        return signals
