"""
NumPy compiled evaluators — dynamics, diagrams, step, and static.

Every method is one body taking ``params=None``; NumPy has nothing to compile.
"""

from __future__ import annotations

import copy

import numpy as np

from minilink.core.compile.evaluators.evaluators import (
    DynamicsEvaluator,
    StaticEvaluator,
    StepEvaluator,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.jacobian import NumpyJacobianMixin
from minilink.core.compile.evaluators.step_rollout import StepRolloutMixin, gather_u
from minilink.core.compile.evaluators.stepping import (
    held_inputs,
    require_step_knobs,
    step_body,
)
from minilink.core.compile.execution_plan import ExecutionPlan
from minilink.core.compile.step_execution_plan import StepExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.system import DynamicSystem, StepSystem, System

###############################################################################
# Public API
###############################################################################


class IntegrationMixin:
    """The step model on :meth:`f`: ``step`` and ``rollout``."""

    def step(self, x, u, t, params=None, *, dt, solver="rk4", substeps=1):
        """``x_{k+1}``: the plant one sample period ``dt`` later, ``u`` held over it.

        ``params=None`` reads the compiled parameters; ``solver`` (``"rk4"``,
        ``"euler"``) and ``substeps`` are Python values.
        """
        require_step_knobs(solver, substeps)
        f = self.rhs_of(params)

        x_next = step_body(f, x, u, u, t, dt, solver, substeps)

        return x_next

    def rollout(
        self,
        x0,
        u_seq,
        t0,
        dt,
        params=None,
        *,
        solver="rk4",
        substeps=1,
        hold="zoh",
        n_steps=None,
    ):
        """``[x_0, …, x_N]``, shape ``(N + 1, n)``: the step model looped over ``u_seq``.

        ``hold="zoh"`` holds each row of ``u_seq`` (shape ``(N, m)``) over its
        period; ``hold="foh"`` reads ``u_seq`` as ``N + 1`` knots, linear between
        them. ``u_seq=None`` holds the nominal input for ``n_steps`` periods.
        """
        require_step_knobs(solver, substeps, hold)

        f = self.rhs_of(params)
        x = np.asarray(x0, dtype=float).reshape(self.n)
        u_seq = self.input_sequence_of(u_seq, n_steps, hold)
        u_start, u_end = held_inputs(u_seq, hold)

        # x_{k+1} = Φ(x_k, u_k, t_k), t_k = t₀ + k dt
        x_samples = [x]
        for k in range(u_start.shape[0]):
            x = step_body(f, x, u_start[k], u_end[k], t0 + k * dt, dt, solver, substeps)
            x_samples.append(x)

        return np.array(x_samples)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def rhs_of(self, params):
        """``f(x, u, t)`` on the compiled parameters, or on ``params``."""
        f = self.f

        return lambda x, u, t: f(x, u, t, params)

    def input_sequence_of(self, u_seq, n_steps, hold):
        """The input sequence as a ``(rows, m)`` array; the nominal input repeated when None."""
        if u_seq is None:
            rows = int(n_steps) + (1 if hold == "foh" else 0)
            return np.tile(np.asarray(self._u_nominal, dtype=float), (rows, 1))

        u_seq = np.asarray(u_seq, dtype=float)
        if u_seq.ndim != 2 or u_seq.shape[1] != self.m:
            raise ValueError(f"u_seq must have shape (N, {self.m}), got {u_seq.shape}")

        return u_seq


class NumpyDynamicEvaluator(NumpyJacobianMixin, DynamicsEvaluator, IntegrationMixin):
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
        self._jac_setup(system, "dynamic", state_fn=self.f, outputs_fn=self.outputs)

    def f(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return self._system.f(x, u, t, params)

    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)


class NumpyDiagramEvaluator(NumpyJacobianMixin, DynamicsEvaluator, IntegrationMixin):
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
        self._jac_setup(diagram, "dynamic", state_fn=None, outputs_fn=None, plan=plan)

    def f(self, x, u, t=0.0, params=None):
        """``dx`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        signals = self.port_signals(x, u, t, params)
        return self.dx_from_signals(x, u, t, params, signals)

    def outputs(self, x, u, t=0.0, params=None):
        self.require_params(params)
        signals = self.port_signals(x, u, t, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def signals(self, x, u, t=0.0, params=None):
        """The internal signal buffer at ``(x, u, t)``: every block output."""
        return self.port_signals(x, u, t, params)

    def compute_internal_signals_dict(self, x, u, t=0.0, params=None) -> dict:
        """Every block output at ``(x, u, t)``, keyed ``"block:port"``."""
        signals = self.port_signals(x, u, t, params)
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

    def port_signals(self, x, u, t, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        signals = np.zeros(self.plan.signal_dim)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, t, op_params)
            if inject is not None and op.out_slice == inject[0]:
                signals[op.out_slice] += inject[1]
        return signals

    def dx_from_signals(self, x, u, t, params, signals):
        """Every block's ``dx`` from the filled signal buffer."""
        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op_params)
        return dx

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)

    def _jac_probe(self, target, wire=None):
        kind, key = target
        signals_of = self.port_signals

        if kind == "state":
            dx_of = self.dx_from_signals

            def probe(x, u, t, params, delta):
                signals = signals_of(x, u, t, params, inject=(wire, delta))
                return dx_of(x, u, t, params, signals)

            return probe

        sl = self.plan.external_output_slices[key] if kind == "port" else key

        def probe(x, u, t, params, delta):
            return signals_of(x, u, t, params, inject=(wire, delta))[sl]

        return probe


class NumpyStaticEvaluator(NumpyJacobianMixin, StaticEvaluator):
    """NumPy evaluator for a static ``System`` leaf (``n == 0``)."""

    def __init__(self, system: System):
        if isinstance(system, DynamicSystem):
            raise TypeError("NumpyStaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")
        self.n = 0
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports())
        self._jac_setup(system, "static", state_fn=None, outputs_fn=self.outputs)

    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)


class NumpyStepEvaluator(NumpyJacobianMixin, StepEvaluator, StepRolloutMixin):
    """Compiled evaluator for a :class:`StepSystem` using NumPy."""

    def __init__(self, system: StepSystem):
        if not isinstance(system, StepSystem):
            raise TypeError("NumpyStepEvaluator requires StepSystem")
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports())
        self._jac_setup(system, "step", state_fn=self.step, outputs_fn=self.outputs)

    def step(self, x, u, k=0, params=None):
        params = self._frozen_params if params is None else params
        return self._system.step(x, u, k, params)

    def outputs(self, x, u, k=0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, k, params)


class NumpyStepDiagramEvaluator(NumpyJacobianMixin, StepEvaluator, StepRolloutMixin):
    """Stateless NumPy evaluator for a compiled step diagram."""

    def __init__(self, plan: StepExecutionPlan, diagram):
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
        self._jac_setup(diagram, "step", state_fn=None, outputs_fn=None, plan=plan)

    def step(self, x, u, k=0, params=None):
        """``x_{k+1}`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        signals = self.port_signals(x, u, k, params)
        return self.next_from_signals(x, u, k, params, signals)

    def outputs(self, x, u, k=0, params=None):
        self.require_params(params)
        signals = self.port_signals(x, u, k, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def compute_internal_signals_dict(self, x, u, k=0, params=None) -> dict:
        """Every block output at ``(x, u, k)``, keyed ``"block:port"``."""
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

    def port_signals(self, x, u, k, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        signals = np.zeros(self.plan.signal_dim)
        x_arr = np.asarray(x, dtype=float).reshape(self.n)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, k, op_params)
            if inject is not None and op.out_slice == inject[0]:
                signals[op.out_slice] += inject[1]
        return signals

    def next_from_signals(self, x, u, k, params, signals):
        """Every block's ``x_{k+1}`` from the filled signal buffer."""
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op_params)
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
