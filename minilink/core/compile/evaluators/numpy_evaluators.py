"""
NumPy compiled evaluators — dynamics, diagrams, step, and static.

Integration methods show textbook RK4 / Euler formulas inline.
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
from minilink.core.compile.evaluators.step_rollout import StepRolloutMixin, gather_u
from minilink.core.compile.evaluators.tiers import NoTraceTierMixin
from minilink.core.compile.execution_plan import ExecutionPlan
from minilink.core.compile.step_execution_plan import StepExecutionPlan
from minilink.core.diagram import validate_diagram_params
from minilink.core.system import DynamicSystem, StepSystem, System

# =============================================================================
# Public API — Integration (textbook)
# =============================================================================


class IntegrationMixin:
    """RK4 / Euler integration on :meth:`f`, :meth:`f_p`, :meth:`f_ivp`, :meth:`f_ivp_p`."""

    # --- RK4 step (ZOH) ---

    def rk4_step(self, x, u, t, dt):
        # x_{k+1} = x + (dt/6)(k1 + 2 k2 + 2 k3 + k4),  u held (ZOH)
        f = self.f
        k1 = f(x, u, t)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_p(self, x, u, t, dt, params):
        f = self.f_p
        k1 = f(x, u, t, params)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, u, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # --- Euler step (ZOH) ---

    def euler_step(self, x, u, t, dt):
        # x_{k+1} = x + dt * f(x, u, t),  u held (ZOH)
        return x + dt * self.f(x, u, t)

    def euler_step_p(self, x, u, t, dt, params):
        return x + dt * self.f_p(x, u, t, params)

    # --- RK4 step IVP ---

    def rk4_step_ivp(self, x, t, dt):
        f = self.f_ivp
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_ivp_p(self, x, t, dt, params):
        f = self.f_ivp_p
        k1 = f(x, t, params)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # --- Euler step IVP ---

    def euler_step_ivp(self, x, t, dt):
        return x + dt * self.f_ivp(x, t)

    def euler_step_ivp_p(self, x, t, dt, params):
        return x + dt * self.f_ivp_p(x, t, params)

    # --- RK4 integrate ZOH ---

    def rk4_integrate_zoh(self, x0, u_sequence, t0, dt):
        """ZOH RK4 rollout: ``u_sequence`` shape ``(N, m)`` → states ``(N+1, n)``."""
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x
        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.rk4_step(x, u_k, t, dt)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    def rk4_integrate_zoh_p(self, x0, u_sequence, t0, dt, params):
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x
        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.rk4_step_p(x, u_k, t, dt, params)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    # --- RK4 integrate linear ---

    def rk4_integrate_linear(self, x0, u_knots, t0, dt):
        """Linear-``u`` RK4 rollout: ``u_knots`` shape ``(K, m)`` → states ``(K, n)``."""
        x = np.asarray(x0).reshape(self.n)
        u = np.asarray(u_knots)
        x_samples = np.zeros((u.shape[0], self.n))
        x_samples[0] = x
        t = t0
        f = self.f
        for k in range(u.shape[0] - 1):
            u0 = u[k]
            u1 = u[k + 1]
            umid = 0.5 * (u0 + u1)
            k1 = f(x, u0, t)
            k2 = f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
            k3 = f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
            k4 = f(x + dt * k3, u1, t + dt)
            x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    def rk4_integrate_linear_p(self, x0, u_knots, t0, dt, params):
        x = np.asarray(x0).reshape(self.n)
        u = np.asarray(u_knots)
        x_samples = np.zeros((u.shape[0], self.n))
        x_samples[0] = x
        t = t0
        f = self.f_p
        for k in range(u.shape[0] - 1):
            u0 = u[k]
            u1 = u[k + 1]
            umid = 0.5 * (u0 + u1)
            k1 = f(x, u0, t, params)
            k2 = f(x + 0.5 * dt * k1, umid, t + 0.5 * dt, params)
            k3 = f(x + 0.5 * dt * k2, umid, t + 0.5 * dt, params)
            k4 = f(x + dt * k3, u1, t + dt, params)
            x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    # --- RK4 integrate IVP ---

    def rk4_integrate_ivp(self, x0, t0, dt, n_steps):
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.rk4_step_ivp(x, t, dt)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)

    def rk4_integrate_ivp_p(self, x0, t0, dt, n_steps, params):
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.rk4_step_ivp_p(x, t, dt, params)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)

    # --- Euler integrate ZOH ---

    def euler_integrate_zoh(self, x0, u_sequence, t0, dt):
        """ZOH Euler rollout: ``u_sequence`` shape ``(N, m)`` → states ``(N+1, n)``."""
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x
        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.euler_step(x, u_k, t, dt)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    def euler_integrate_zoh_p(self, x0, u_sequence, t0, dt, params):
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x
        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.euler_step_p(x, u_k, t, dt, params)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    # --- Euler integrate IVP ---

    def euler_integrate_ivp(self, x0, t0, dt, n_steps):
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.euler_step_ivp(x, t, dt)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)

    def euler_integrate_ivp_p(self, x0, t0, dt, n_steps, params):
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.euler_step_ivp_p(x, t, dt, params)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)

    # --- ZOH sugar (integrate_zoh*) ---

    def integrate_zoh_rollout(self, x0, u_hold, t0, dt_hold, *, dt_inner=None):
        """
        Piecewise-constant ``u_hold`` over ``[t0, t0 + dt_hold]``.

        Returns
        -------
        t_samples : ndarray, shape (n_sub + 1,)
        x_samples : ndarray, shape (n_sub + 1, n)
        """
        u_hold, dt_hold, dt_step, u_sequence = self._zoh_hold_sequence(
            u_hold, dt_hold, dt_inner=dt_inner
        )
        x_samples = self.rk4_integrate_zoh(x0, u_sequence, t0, dt_step)
        t_samples = t0 + np.arange(x_samples.shape[0], dtype=float) * dt_step
        return t_samples, x_samples

    def integrate_zoh(self, x0, u_hold, t0, dt_hold, *, dt_inner=None):
        """Advance state with piecewise-constant ``u_hold``; return final state."""
        _, x_samples = self.integrate_zoh_rollout(
            x0, u_hold, t0, dt_hold, dt_inner=dt_inner
        )
        return x_samples[-1]

    def integrate_zoh_p(self, x0, u_hold, t0, dt_hold, params, *, dt_inner=None):
        """Parametric twin of :meth:`integrate_zoh`."""
        _, _, dt_step, u_sequence = self._zoh_hold_sequence(
            u_hold, dt_hold, dt_inner=dt_inner
        )
        x_samples = self.rk4_integrate_zoh_p(x0, u_sequence, t0, dt_step, params)
        return x_samples[-1]

    def _zoh_hold_sequence(self, u_hold, dt_hold, *, dt_inner=None):
        u_hold = np.asarray(u_hold, dtype=float).reshape(self.m)
        dt_hold = float(dt_hold)
        if dt_hold <= 0.0:
            raise ValueError(f"dt_hold must be positive, got {dt_hold}")

        if dt_inner is None:
            dt_step = dt_hold
            n_sub = 1
        else:
            dt_step = float(dt_inner)
            if dt_step <= 0.0:
                raise ValueError(f"dt_inner must be positive, got {dt_inner}")
            n_sub = max(1, int(round(dt_hold / dt_step)))
            dt_step = dt_hold / n_sub

        u_sequence = np.tile(u_hold, (n_sub, 1))
        return u_hold, dt_hold, dt_step, u_sequence


# =============================================================================
# Public API — NumpyDynamicEvaluator
# =============================================================================


class NumpyDynamicEvaluator(NoTraceTierMixin, DynamicsEvaluator, IntegrationMixin):
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

    def f_p(self, x, u, t, params):
        return self._system.f(x, u, t, params)

    def outputs(self, x, u, t=0.0):
        return outputs_from_ports(self._system, x, u, t, self._frozen_params)

    def outputs_p(self, x, u, t, params):
        return outputs_from_ports(self._system, x, u, t, params)


# =============================================================================
# Public API — NumpyDiagramEvaluator
# =============================================================================


class NumpyDiagramEvaluator(NoTraceTierMixin, DynamicsEvaluator, IntegrationMixin):
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
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op.bound_params)
        return dx

    def f_p(self, x, u, t, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, t, params)
        dx = np.zeros(self.plan.state_dim)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t, op_params)
        return dx

    def outputs(self, x, u, t=0.0):
        signals = self._compute_port_signals(x, u, t)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

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
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
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
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, t, op_params)
        return signals


# =============================================================================
# Public API — NumpyStaticEvaluator
# =============================================================================


class NumpyStaticEvaluator(NoTraceTierMixin, StaticEvaluator):
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

    def outputs(self, x, u, t=0.0):
        return outputs_from_ports(self._system, x, u, t, self._frozen_params)

    def outputs_p(self, x, u, t, params):
        return outputs_from_ports(self._system, x, u, t, params)


# =============================================================================
# Public API — NumpyStepEvaluator
# =============================================================================


class NumpyStepEvaluator(NoTraceTierMixin, StepEvaluator, StepRolloutMixin):
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

    def step(self, x, u, k=0):
        return self._system.step(x, u, k, self._frozen_params)

    def step_p(self, x, u, k, params):
        return self._system.step(x, u, k, params)

    def outputs(self, x, u, k=0):
        return outputs_from_ports(self._system, x, u, k, self._frozen_params)

    def outputs_p(self, x, u, k, params):
        return outputs_from_ports(self._system, x, u, k, params)


# =============================================================================
# Public API — NumpyStepDiagramEvaluator
# =============================================================================


class NumpyStepDiagramEvaluator(NoTraceTierMixin, StepEvaluator, StepRolloutMixin):
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

    def step(self, x, u, k=0):
        signals = self._compute_port_signals(x, u, k)
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op.bound_params)
        return x_new

    def step_p(self, x, u, k, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, k, params)
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op_params)
        return x_new

    def outputs(self, x, u, k=0):
        signals = self._compute_port_signals(x, u, k)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def outputs_p(self, x, u, k, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, k, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def compute_internal_signals_dict(self, x, u, k=0) -> dict:
        signals = self._compute_port_signals(x, u, k)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    def step_block(self, sys_id, x, u, k=0):
        """Advance one step-diagram subsystem (subset-firing hook for Computer)."""
        signals = self._compute_port_signals(x, u, k)
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            if op.sys_id != sys_id:
                continue
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op.bound_params)
        return x_new

    def _compute_port_signals(self, x, u, k) -> np.ndarray:
        signals = np.zeros(self.plan.signal_dim)
        x_arr = np.asarray(x, dtype=float).reshape(self.n)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            signals[op.out_slice] = op.compute_func(
                local_x, local_u, k, op.bound_params
            )
        return signals

    def _compute_port_signals_p(self, x, u, k, params) -> np.ndarray:
        signals = np.zeros(self.plan.signal_dim)
        x_arr = np.asarray(x, dtype=float).reshape(self.n)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, k, op_params)
        return signals
