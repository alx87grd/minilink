"""
NumPy evaluation backends for compiled systems.

Provides two evaluators:

- :class:`NumpyLeafEvaluator` — for a single (non-diagram) System.
- :class:`NumpyDiagramEvaluator` — for a compiled DiagramSystem.

Both inherit from :class:`~minilink.core.compile.evaluators.evaluator.DynamicsEvaluator`.
"""

import copy

import numpy as np

from minilink.core.compile.evaluators.evaluator import DynamicsEvaluator
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)
from minilink.core.diagram import validate_diagram_params


# Leaf evaluator
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
        self._u_nominal = np.copy(system.get_u_from_input_ports())

    # Standard tier (frozen params)
    def f(self, x, u, t=0.0):
        return self._system.f(x, u, t, self._frozen_params)

    def h(self, x, u, t=0.0):
        return self._system.h(x, u, t, self._frozen_params)

    def outputs(self, x, u, t=0.0):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, self._frozen_params)
        return result

    # Parametric tier (caller-supplied params)
    def f_p(self, x, u, t, params):
        return self._system.f(x, u, t, params)

    def h_p(self, x, u, t, params):
        return self._system.h(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        result = {}
        for port_id, port in self._system.outputs.items():
            result[port_id] = port.compute(x, u, t, params)
        return result


# Shared helper
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
        Recipe built by :func:`~minilink.core.compile.compiler._build_gather_sources`.
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


# Diagram evaluator
class NumpyDiagramEvaluator(DynamicsEvaluator):
    """Stateless NumPy evaluator for a compiled diagram.

    Inherits from :class:`DynamicsEvaluator`.  Implements ``f`` and
    :meth:`outputs` for **diagram boundary** ports only (same semantics as
    :class:`NumpyLeafEvaluator`).  :meth:`compute_internal_signals_dict`
    exposes every subsystem output in the internal buffer.  :meth:`h` returns
    the sole external output when exactly one boundary port exists; otherwise
    it raises.  The parametric tier (:meth:`f_p`, :meth:`outputs_p`) takes
    nested params ``{sys_id: {...}}``.

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.core.compile.compiler.build_execution_plan`.
    diagram : DiagramSystem
        The source diagram (used for ``m``, ``_u_nominal``, and ``p``).

    Examples
    --------
    >>> evaluator = compile_diagram(diagram, backend="numpy")
    >>> dx = evaluator.f(x, u, t)
    >>> plant_y = evaluator.compute_internal_signals_dict(x, u, t)["plant:y"]
    """

    def __init__(self, plan: ExecutionPlan, diagram):
        self.plan = plan
        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "numpy"
        self._frozen_params = None  # per-op binding, not diagram-level
        self._u_nominal = np.copy(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)

    # ABC: Standard tier
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
        out = self.outputs(x, u, t)
        if len(out) == 1:
            return next(iter(out.values()))
        raise NotImplementedError(
            "Diagram h() is only defined when exactly one diagram output port "
            "exists; use outputs()."
        )

    def outputs(self, x, u, t=0.0):
        """Boundary outputs only — keys match :attr:`DiagramSystem.outputs`."""
        signals = self._compute_port_signals(x, u, t)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    # ABC: Parametric tier
    def f_p(self, x, u, t, params):
        """Diagram ``f`` with caller-supplied nested params ``{sys_id: {...}}``.

        Missing subsystem ids fall back to each block's live ``self.params``;
        ``bound_params`` (frozen tier) is ignored here.
        """
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, t, params)

        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op_params)
        return dx

    def h_p(self, x, u, t, params):
        out = self.outputs_p(x, u, t, params)
        if len(out) == 1:
            return next(iter(out.values()))
        raise NotImplementedError(
            "Diagram h_p() is only defined when exactly one diagram output "
            "port exists; use outputs_p()."
        )

    def outputs_p(self, x, u, t, params):
        """Boundary outputs with caller-supplied nested params (see :meth:`f_p`)."""
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, t, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    # Diagram-specific methods
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
        """Full internal signal buffer as ``\"sys_id:port_id\"`` → array (diagram-specific)."""
        signals = self._compute_port_signals(x, u, t)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    # Private
    def _compute_port_signals(
        self, x: np.ndarray, u: np.ndarray, t: float
    ) -> np.ndarray:
        """Evaluate all port signals in topological order (frozen params).

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

    def _compute_port_signals_p(
        self, x: np.ndarray, u: np.ndarray, t: float, params
    ) -> np.ndarray:
        """Port-signal buffer with caller-supplied nested params."""
        signals = np.zeros(self.plan.signal_dim)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, t, op_params)
        return signals
