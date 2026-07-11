"""
JAX evaluation backends for compiled systems.

- :class:`JaxDynamicEvaluator` — ``DynamicSystem`` leaf.
- :class:`JaxDiagramEvaluator` — compiled ``DiagramSystem``.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.compile.evaluators.dynamics_evaluator import DynamicsEvaluator
from minilink.core.compile.evaluators.jax_utils import (
    JaxIntegrationMixin,
    build_dynamic_leaf_tiers,
    check_jax_compatible,
    gather_u_jax,
)
from minilink.core.compile.evaluators.tiers import TraceTierMixin, register_jit_aliases
from minilink.core.compile.execution_plan import ExecutionPlan
from minilink.core.diagram import validate_diagram_params
from minilink.core.system import DynamicSystem


class JaxDynamicEvaluator(DynamicsEvaluator, JaxIntegrationMixin, TraceTierMixin):
    """Compiled evaluator for a :class:`DynamicSystem` using JAX."""

    def __init__(self, system: DynamicSystem, verbose=False):
        if not isinstance(system, DynamicSystem):
            raise TypeError("JaxDynamicEvaluator requires DynamicSystem")

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
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        f_raw = system.f
        frozen_p = self._frozen_params
        u_nom = self._u_nominal
        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_jax_compatible(
            f_raw, system.name, dummy_x, dummy_u, dummy_t, frozen_p, jax
        )
        for port_id, port in system.outputs.items():
            check_jax_compatible(
                port.compute,
                f"{system.name}:{port_id}",
                dummy_x,
                dummy_u,
                dummy_t,
                frozen_p,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print(
                f"[compile] Step 1: JIT-compiling to XLA on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        tiers = build_dynamic_leaf_tiers(jax, system, frozen_p, u_nom)
        self._f_trace_fn = tiers["_f_trace_fn"]
        self._f_trace_p_fn = tiers["_f_trace_p_fn"]
        self._f_jit_fn = tiers["_f_jit_fn"]
        self._f_jit_p_fn = tiers["_f_jit_p_fn"]
        self._f_ivp_trace_fn = tiers["_f_ivp_trace_fn"]
        self._f_ivp_jit_fn = tiers["_f_ivp_jit_fn"]
        self._jac_f_params_jit_fn = tiers["_jac_f_params_jit_fn"]
        self._jac_ivp_jit_fn = tiers["_jac_ivp_jit_fn"]
        self._outputs_trace_fn = tiers["_outputs_trace_fn"]
        self._outputs_trace_p_fn = tiers["_outputs_trace_p_fn"]
        self._outputs_jit_fn = tiers["_outputs_jit_fn"]
        self._outputs_jit_p_fn = tiers["_outputs_jit_p_fn"]

        self._setup_integration_tiers(jax, jnp)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print("[compile] Step 2: Warm-starting JIT cache...", end="", flush=True)

        try:
            self._f_jit_fn(dummy_x, dummy_u, dummy_t)
            self._f_jit_p_fn(dummy_x, dummy_u, dummy_t, frozen_p)
            self._f_ivp_jit_fn(dummy_x, dummy_t)
            self._outputs_jit_fn(dummy_x, dummy_u, dummy_t)
            self._outputs_jit_p_fn(dummy_x, dummy_u, dummy_t, frozen_p)
        except Exception as e:
            raise RuntimeError(
                f"\n\nBlock '{system.name}' failed during JAX warm-start.\n"
                f"Rewrite in purely functional style or use backend='numpy'.\n"
                f"Original JAX error: {e}"
            ) from e

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    def f(self, x, u, t=0.0):
        return self._f_jit_fn(x, u, t)

    def f_trace(self, x, u, t=0.0):
        """Pre-JIT flat callable for JAX composition."""
        return self._f_trace_fn(x, u, t)

    def f_scipy(self, x, u, t=0.0):
        x = self.jnp.asarray(x)
        u = self.jnp.asarray(u)
        return np.asarray(self._f_jit_fn(x, u, t))

    def outputs(self, x, u, t=0.0):
        return self._outputs_jit_fn(x, u, t)

    def outputs_trace(self, x, u, t=0.0):
        """Pre-JIT boundary outputs for JAX composition."""
        return self._outputs_trace_fn(x, u, t)

    def f_p(self, x, u, t, params):
        return self._f_jit_p_fn(x, u, t, params)

    def f_trace_p(self, x, u, t, params):
        """Pre-JIT parametric dynamics for JAX composition."""
        return self._f_trace_p_fn(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        return self._outputs_jit_p_fn(x, u, t, params)

    def outputs_trace_p(self, x, u, t, params):
        """Pre-JIT parametric boundary outputs for JAX composition."""
        return self._outputs_trace_p_fn(x, u, t, params)

    def jacobian_f_params(self, x, u, t, params):
        if params is None:
            raise ValueError("jacobian_f_params requires an explicit params pytree")
        return self._jac_f_params_jit_fn(x, u, t, params)


register_jit_aliases(JaxDynamicEvaluator, ("f", "outputs"))


class JaxDiagramEvaluator(DynamicsEvaluator, JaxIntegrationMixin, TraceTierMixin):
    """JAX-compatible evaluator for a compiled diagram."""

    def __init__(self, plan: ExecutionPlan, diagram, verbose=False):
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
        self.jax = jax
        self.jnp = jnp

        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "jax"
        self._frozen_params = None
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)

        if verbose:
            t0 = time.perf_counter()
            n_blocks = len(diagram.subsystems)
            print(
                f"[compile] Step 0: Checking JAX compatibility of {n_blocks} blocks...",
                end="",
                flush=True,
            )

        self._check_jax_compatibility(diagram, jax, jnp)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print(
                f"[compile] Step 3: JIT-compiling to XLA on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        self._f_trace_fn = self._f_eager
        self._f_trace_p_fn = self._f_p_eager
        self._outputs_trace_fn = self._external_outputs_eager
        self._outputs_trace_p_fn = self._outputs_p_eager
        u_nom = self._u_nominal
        self._f_jit_fn = jax.jit(self._f_trace_fn)
        self._f_jit_p_fn = jax.jit(self._f_trace_p_fn)
        self._f_ivp_trace_fn = lambda x, t: self._f_trace_fn(x, u_nom, t)
        self._f_ivp_jit_fn = jax.jit(self._f_ivp_trace_fn)
        self._jac_ivp_jit_fn = jax.jit(jax.jacfwd(self._f_ivp_jit_fn, argnums=0))

        self._setup_integration_tiers(jax, jnp)

        self._outputs_jit_fn = jax.jit(self._outputs_trace_fn)
        self._jit_internal_signals = jax.jit(
            lambda x, u, t: self._internal_signals_eager(x, u, t)
        )
        self._outputs_jit_p_fn = jax.jit(self._outputs_trace_p_fn)
        self._jac_f_params_jit_fn = jax.jit(jax.jacfwd(self._f_trace_p_fn, argnums=3))

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print("[compile] Step 4: Warm-starting JIT cache...", end="", flush=True)

        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        try:
            self._f_jit_fn(dummy_x, dummy_u, dummy_t)
            self._f_ivp_jit_fn(dummy_x, dummy_t)
            self._outputs_jit_fn(dummy_x, dummy_u, dummy_t)
            self._jit_internal_signals(dummy_x, dummy_u, dummy_t)
        except Exception as e:
            raise RuntimeError(
                f"\n\nDiagram failed during JAX warm-start.\nOriginal JAX error: {e}"
            ) from e

        params_nominal = {
            sys_id: sub.params for sys_id, sub in diagram.subsystems.items()
        }
        try:
            self._f_jit_p_fn(dummy_x, dummy_u, dummy_t, params_nominal)
            self._outputs_jit_p_fn(dummy_x, dummy_u, dummy_t, params_nominal)
        except Exception:
            pass

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    def _external_outputs_eager(self, x, u, t):
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals(x, u, t, dtype)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def _internal_signals_eager(self, x, u, t):
        dtype = self._infer_dtype(x, u)
        return self._compute_port_signals(x, u, t, dtype)

    def _check_jax_compatibility(self, diagram, jax, jnp):
        for sys_id, sys in diagram.subsystems.items():
            dummy_x = jnp.zeros(sys.n)
            dummy_u = jnp.zeros(sys.m)
            dummy_t = 0.0
            params = getattr(sys, "params", {})

            if isinstance(sys, DynamicSystem):
                check_jax_compatible(
                    sys.f,
                    f"{sys_id} ({sys.name})",
                    dummy_x,
                    dummy_u,
                    dummy_t,
                    params,
                    jax,
                )

            for port_id, port in sys.outputs.items():
                check_jax_compatible(
                    port.compute,
                    f"{sys_id}:{port_id} ({sys.name})",
                    dummy_x,
                    dummy_u,
                    dummy_t,
                    params,
                    jax,
                )

    def _infer_dtype(self, x, u):
        jnp = self._jnp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        return dtype if dtype is not None else jnp.float32

    def f(self, x, u, t=0.0):
        return self._f_jit_fn(x, u, t)

    def f_trace(self, x, u, t=0.0):
        """Pre-JIT flat callable for JAX composition."""
        return self._f_trace_fn(x, u, t)

    def f_scipy(self, x, u, t=0.0):
        x = self._jnp.asarray(x)
        u = self._jnp.asarray(u)
        return np.asarray(self._f_jit_fn(x, u, t))

    def _f_eager(self, x, u, t=0.0):
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals(x, u, t, dtype)
        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            dx_piece = op.f_func(local_x, local_u, t, op.bound_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def _f_p_eager(self, x, u, t, params):
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals_p(x, u, t, dtype, params)
        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = None if params is None else params.get(op.sys_id)
            dx_piece = op.f_func(local_x, local_u, t, op_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def _outputs_p_eager(self, x, u, t, params):
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals_p(x, u, t, dtype, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def outputs(self, x, u, t=0.0):
        return self._outputs_jit_fn(x, u, t)

    def outputs_trace(self, x, u, t=0.0):
        """Pre-JIT boundary outputs for JAX composition."""
        return self._outputs_trace_fn(x, u, t)

    def f_p(self, x, u, t, params):
        validate_diagram_params(params, self._subsystem_ids)
        return self._f_jit_p_fn(x, u, t, params)

    def f_trace_p(self, x, u, t, params):
        """Pre-JIT parametric dynamics for JAX composition."""
        validate_diagram_params(params, self._subsystem_ids)
        return self._f_trace_p_fn(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        validate_diagram_params(params, self._subsystem_ids)
        return self._outputs_jit_p_fn(x, u, t, params)

    def outputs_trace_p(self, x, u, t, params):
        """Pre-JIT parametric boundary outputs for JAX composition."""
        validate_diagram_params(params, self._subsystem_ids)
        return self._outputs_trace_p_fn(x, u, t, params)

    def jacobian_f_params(self, x, u, t, params):
        if params is None:
            raise ValueError(
                "jacobian_f_params requires an explicit nested params pytree"
            )
        validate_diagram_params(params, self._subsystem_ids)
        return self._jac_f_params_jit_fn(x, u, t, params)

    def compute_internal_signals(self, x, u, t=0.0):
        return self._jit_internal_signals(x, u, t)

    def compute_internal_signals_dict(self, x, u, t=0.0):
        signals = self.compute_internal_signals(x, u, t)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    def _compute_port_signals(self, x, u, t, dtype):
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            y_out = op.compute_func(local_x, local_u, t, op.bound_params)
            signals = signals.at[op.out_slice].set(y_out)
        return signals

    def _compute_port_signals_p(self, x, u, t, dtype, params):
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = None if params is None else params.get(op.sys_id)
            y_out = op.compute_func(local_x, local_u, t, op_params)
            signals = signals.at[op.out_slice].set(y_out)
        return signals


register_jit_aliases(JaxDiagramEvaluator, ("f", "outputs"))
