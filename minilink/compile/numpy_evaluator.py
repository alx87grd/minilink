"""
NumPy evaluation backends for compiled systems.

Provides two evaluators:

- :class:`NumpyLeafEvaluator` — for a single (non-diagram) System.
- :class:`NumpyDiagramEvaluator` — for a compiled DiagramSystem.

Both inherit from :class:`~minilink.compile.evaluator.DynamicsEvaluator`.
"""

from __future__ import annotations

import copy
from typing import Any, Callable

import numpy as np

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


class NumpyLeafEvaluator(DynamicsEvaluator):
    """Compiled evaluator for a single System using NumPy.

    Parameters
    ----------
    system : System
        The system to wrap.  Its ``params`` are deep-copied at construction
        (frozen), and nominal input port values are snapshotted.
    """

    def __init__(self, system):
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports(0))

    # -- Standard tier (frozen params) ------------------------------------

    def f(self, x, u, t=0.0):
        return self._system.f(x, u, t, self._frozen_params)

    def h(self, x, u, t=0.0):
        return self._system.h(x, u, t, self._frozen_params)

    def outputs(self, x, u, t=0.0):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, self._frozen_params)
        return result

    # -- Parametric tier (caller-supplied params) -------------------------

    def f_p(self, x, u, t, params):
        return self._system.f(x, u, t, params)

    def h_p(self, x, u, t, params):
        return self._system.h(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, params)
        return result


# =====================================================================
# Shared helper
# =====================================================================


def _gather_u(
    gather_sources: tuple[tuple[int, object, int], ...],
    u_dim: int,
    signals: np.ndarray,
    u: np.ndarray,
) -> np.ndarray:
    """Assemble the local input vector from pre-mapped sources.

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


# =====================================================================
# Diagram evaluator
# =====================================================================


class NumpyDiagramEvaluator(DynamicsEvaluator):
    """Stateless NumPy evaluator for a compiled diagram.

    Inherits from :class:`DynamicsEvaluator`.  Only ``f`` is mapped to the
    ABC interface; ``h`` / ``outputs`` and the parametric tier raise
    ``NotImplementedError``.  Use diagram-specific methods
    (``compute_outputs``, ``compute_internal_signals``,
    ``compute_internal_signals_dict``) for port-level access.

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.compile.compiler.build_execution_plan`.
    diagram : DiagramSystem
        The source diagram (used only to snapshot ``m`` and ``_u_nominal``).

    Examples
    --------
    >>> evaluator = compile_diagram(diagram, backend="numpy")
    >>> dx = evaluator.f(x, u, t)
    >>> y  = evaluator.compute_outputs(x, u, t, ports=[("plant", "y")])
    """

    def __init__(self, plan: ExecutionPlan, diagram):
        self.plan = plan
        self.n = plan.state_dim
        self.m = diagram.m
        self.p = plan.signal_dim
        self.backend = "numpy"
        self._frozen_params = None  # per-op binding, not diagram-level
        self._u_nominal = np.copy(diagram.get_u_from_input_ports(0))

    # ── ABC: Standard tier ──────────────────────────────────────────

    def f(self, x: np.ndarray, u: np.ndarray, t: float = 0.0) -> np.ndarray:
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
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op.bound_params)
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

    # ── Diagram-specific methods ────────────────────────────────────

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
        """Evaluate and return the full internal signal buffer (flat array).

        Parameters
        ----------
        x : np.ndarray, shape (state_dim,)
        u : np.ndarray, shape (m,)
        t : float

        Returns
        -------
        np.ndarray, shape (signal_dim,)
            The complete internal signal buffer after evaluation.
        """
        return self._compute_port_signals(x, u, t)

    def compute_internal_signals_dict(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0
    ) -> dict:
        """Evaluate all port signals and return as a labelled dict.

        Returns
        -------
        dict mapping ``"sys_id:port_id"`` -> ``np.ndarray``
            One entry per output port in topological order.
        """
        signals = self._compute_port_signals(x, u, t)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    # ── Private ─────────────────────────────────────────────────────

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
            signals[op.out_slice] = op.compute_func(
                local_x, local_u, t, op.bound_params
            )
        return signals
