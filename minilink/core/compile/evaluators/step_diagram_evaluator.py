"""
NumPy and JAX evaluators for compiled step diagrams.
"""

import numpy as np

from minilink.core.compile.evaluators.numpy_evaluator import _gather_u
from minilink.core.compile.evaluators.step_evaluator import StepEvaluator
from minilink.core.compile.evaluators.tiers import (
    NoTraceTierMixin,
    TraceTierMixin,
    register_jit_aliases,
)
from minilink.core.compile.step_execution_plan import StepExecutionPlan
from minilink.core.diagram import validate_diagram_params


class NumpyStepDiagramEvaluator(NoTraceTierMixin, StepEvaluator):
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
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op.bound_params)
        return x_new

    def step_p(self, x, u, k, params):
        validate_diagram_params(params, self._subsystem_ids)
        signals = self._compute_port_signals_p(x, u, k, params)
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
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
        """Advance one step-diagram subsystem (subset-firing hook for :class:`~minilink.simulation.computer.Computer`)."""
        signals = self._compute_port_signals(x, u, k)
        x_new = np.asarray(x, dtype=float).reshape(self.n).copy()
        for op in self.plan.step_ops:
            if op.sys_id != sys_id:
                continue
            local_x = x_new[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            x_new[op.local_x_slice] = op.step_func(local_x, local_u, k, op.bound_params)
        return x_new

    def _compute_port_signals(self, x, u, k) -> np.ndarray:
        signals = np.zeros(self.plan.signal_dim)
        x_arr = np.asarray(x, dtype=float).reshape(self.n)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            signals[op.out_slice] = op.compute_func(
                local_x, local_u, k, op.bound_params
            )
        return signals

    def _compute_port_signals_p(self, x, u, k, params) -> np.ndarray:
        signals = np.zeros(self.plan.signal_dim)
        x_arr = np.asarray(x, dtype=float).reshape(self.n)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = _gather_u(op.gather_sources, op.u_dim, signals, u)
            op_params = None if params is None else params.get(op.sys_id)
            signals[op.out_slice] = op.compute_func(local_x, local_u, k, op_params)
        return signals


class JaxStepDiagramEvaluator(StepEvaluator, TraceTierMixin):
    """JAX evaluator for a compiled step diagram."""

    def __init__(self, plan: StepExecutionPlan, diagram, verbose=False):
        import time

        import jax
        import jax.numpy as jnp

        from minilink.core.compile.evaluators.jax_utils import (
            build_jit_step_rollout,
            check_jax_compatible,
        )

        self.jax = jax
        self.jnp = jnp
        self.plan = plan
        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "jax"
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)
        self._dtype = jnp.float64

        dummy_x = jnp.zeros(self.n)
        dummy_k = 0

        if verbose:
            t0 = time.perf_counter()
            print(
                "[compile] Step 0: Checking JAX compatibility of step diagram...",
                end="",
                flush=True,
            )

        for op in plan.port_ops:
            check_jax_compatible(
                op.compute_func,
                op.label,
                dummy_x[op.local_x_slice],
                jnp.zeros(op.u_dim),
                dummy_k,
                op.bound_params,
                jax,
            )
        for op in plan.step_ops:
            check_jax_compatible(
                op.step_func,
                op.label,
                dummy_x[op.local_x_slice],
                jnp.zeros(op.u_dim),
                dummy_k,
                op.bound_params,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        self._step_trace_fn = self._make_step_fn()
        self._step_trace_p_fn = self._make_step_p_fn()
        self._outputs_trace_fn = self._make_outputs_fn()
        self._outputs_trace_p_fn = self._make_outputs_p_fn()
        self._step_jit_fn = jax.jit(self._step_trace_fn)
        self._step_jit_p_fn = jax.jit(self._step_trace_p_fn)
        self._outputs_jit_fn = jax.jit(self._outputs_trace_fn)
        self._outputs_jit_p_fn = jax.jit(self._outputs_trace_p_fn)
        self._jit_rollout = build_jit_step_rollout(jax, jnp, self._step_jit_fn)

    def _make_step_fn(self):
        from minilink.core.compile.evaluators.jax_utils import gather_u_jax

        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        gather = gather_u_jax

        def _step(x, u, k):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_work = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_work[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op.bound_params)
                )
            x_new = x_work
            for op in plan.step_ops:
                local_x = x_new[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                x_new = x_new.at[op.local_x_slice].set(
                    op.step_func(local_x, local_u, k, op.bound_params)
                )
            return x_new

        return _step

    def _make_step_p_fn(self):
        from minilink.core.compile.evaluators.jax_utils import gather_u_jax

        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        gather = gather_u_jax

        def _step_p(x, u, k, params):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_work = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_work[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                op_params = None if params is None else params.get(op.sys_id)
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op_params)
                )
            x_new = x_work
            for op in plan.step_ops:
                local_x = x_new[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                op_params = None if params is None else params.get(op.sys_id)
                x_new = x_new.at[op.local_x_slice].set(
                    op.step_func(local_x, local_u, k, op_params)
                )
            return x_new

        return _step_p

    def _make_outputs_fn(self):
        from minilink.core.compile.evaluators.jax_utils import gather_u_jax

        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        gather = gather_u_jax

        def _outputs(x, u, k):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_arr = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_arr[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op.bound_params)
                )
            return {
                port_id: signals[sl]
                for port_id, sl in plan.external_output_slices.items()
            }

        return _outputs

    def _make_outputs_p_fn(self):
        from minilink.core.compile.evaluators.jax_utils import gather_u_jax

        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        gather = gather_u_jax

        def _outputs_p(x, u, k, params):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_arr = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_arr[op.local_x_slice]
                local_u = gather(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
                op_params = None if params is None else params.get(op.sys_id)
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op_params)
                )
            return {
                port_id: signals[sl]
                for port_id, sl in plan.external_output_slices.items()
            }

        return _outputs_p

    def step(self, x, u, k=0):
        return np.asarray(
            self._step_jit_fn(
                self.jnp.asarray(x, dtype=self._dtype),
                self.jnp.asarray(u, dtype=self._dtype),
                k,
            ),
            dtype=float,
        )

    def step_trace(self, x, u, k=0):
        """Pre-JIT flat step for JAX composition."""
        return self._step_trace_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
        )

    def step_p(self, x, u, k, params):
        return np.asarray(
            self._step_jit_p_fn(
                self.jnp.asarray(x, dtype=self._dtype),
                self.jnp.asarray(u, dtype=self._dtype),
                k,
                params,
            ),
            dtype=float,
        )

    def step_trace_p(self, x, u, k, params):
        """Pre-JIT parametric step for JAX composition."""
        return self._step_trace_p_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
            params,
        )

    def outputs(self, x, u, k=0):
        out = self._outputs_jit_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
        )
        return {pid: np.asarray(val, dtype=float) for pid, val in out.items()}

    def outputs_trace(self, x, u, k=0):
        """Pre-JIT boundary outputs for JAX composition."""
        return self._outputs_trace_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
        )

    def outputs_p(self, x, u, k, params):
        out = self._outputs_jit_p_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
            params,
        )
        return {pid: np.asarray(val, dtype=float) for pid, val in out.items()}

    def outputs_trace_p(self, x, u, k, params):
        """Pre-JIT parametric boundary outputs for JAX composition."""
        return self._outputs_trace_p_fn(
            self.jnp.asarray(x, dtype=self._dtype),
            self.jnp.asarray(u, dtype=self._dtype),
            k,
            params,
        )

    def rollout(self, x0, *, n_steps, u=None, record_outputs=True):
        if record_outputs and self.p:
            return super().rollout(
                x0, n_steps=n_steps, u=u, record_outputs=record_outputs
            )

        jnp = self.jnp
        x0 = jnp.asarray(x0, dtype=float).reshape(self.n)
        u_samples = self._coerce_rollout_inputs(n_steps, u)
        if n_steps == 0:
            x_path = x0.reshape(1, self.n)
        else:
            u_steps = jnp.asarray(u_samples[:, :n_steps].T)
            x_path = self._jit_rollout(x0, u_steps)
        x_samples = np.asarray(x_path, dtype=float).T
        k = np.arange(n_steps + 1, dtype=float)
        u_np = np.asarray(u_samples, dtype=float)
        from minilink.core.step_rollout import StepRollout

        return StepRollout(k=k, x=x_samples, u=u_np, signals={})


register_jit_aliases(JaxStepDiagramEvaluator, ("step", "outputs"))
