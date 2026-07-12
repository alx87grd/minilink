"""
NumPy evaluation backends for compiled systems.

- :class:`NumpyDynamicEvaluator` — ``DynamicSystem`` leaf.
- :class:`NumpyDiagramEvaluator` — compiled ``DiagramSystem``.
"""

import copy

import numpy as np

from minilink.core.compile.evaluators.dynamics_evaluator import DynamicsEvaluator
from minilink.core.compile.evaluators.output_evaluator import outputs_from_ports
from minilink.core.compile.evaluators.tiers import NoTraceTierMixin
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)
from minilink.core.diagram import validate_diagram_params
from minilink.core.system import DynamicSystem


class NumpyDynamicEvaluator(NoTraceTierMixin, DynamicsEvaluator):
    """Compiled evaluator for a :class:`DynamicSystem` using NumPy."""

    def __init__(self, system: DynamicSystem):
        if not isinstance(system, DynamicSystem):
            raise TypeError("NumpyDynamicEvaluator requires DynamicSystem")
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports())

    def f(self, x, u, t=0.0):
        return self._system.f(x, u, t, self._frozen_params)

    def outputs(self, x, u, t=0.0):
        return outputs_from_ports(self._system, x, u, t, self._frozen_params)

    def f_p(self, x, u, t, params):
        return self._system.f(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        return outputs_from_ports(self._system, x, u, t, params)


def _gather_u(
    gather_sources: tuple[tuple[int, object, int], ...],
    u_dim: int,
    signals: np.ndarray,
    u: np.ndarray,
) -> np.ndarray:
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


class NumpyDiagramEvaluator(NoTraceTierMixin, DynamicsEvaluator):
    """Stateless NumPy evaluator for a compiled diagram."""

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
        self._frozen_params = None
        self._u_nominal = np.copy(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)

    def f(self, x: np.ndarray, u: np.ndarray, t: float = 0.0) -> np.ndarray:
        signals = self._compute_port_signals(x, u, t)
        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op.bound_params)
        return dx

    def outputs(self, x, u, t=0.0):
        signals = self._compute_port_signals(x, u, t)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def f_p(self, x, u, t, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, t, params)
        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op_params)
        return dx

    def outputs_p(self, x, u, t, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, t, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def compute_internal_signals(self, x: np.ndarray, u: np.ndarray, t: float = 0.0):
        return self._compute_port_signals(x, u, t)

    def compute_internal_signals_dict(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0
    ) -> dict:
        signals = self._compute_port_signals(x, u, t)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    def _compute_port_signals(
        self, x: np.ndarray, u: np.ndarray, t: float
    ) -> np.ndarray:
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
        signals = np.zeros(self.plan.signal_dim)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, t, op_params)
        return signals
