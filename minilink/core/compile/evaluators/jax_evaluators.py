"""
JAX compiled evaluators — dynamics, diagrams, step, and static.

Integration methods show textbook RK4 / Euler formulas inline.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.compile.evaluators.evaluators import (
    DynamicsEvaluator,
    StaticEvaluator,
    StepEvaluator,
)
from minilink.core.compile.evaluators.step_rollout import StepRolloutMixin
from minilink.core.compile.evaluators.tiers import TraceTierMixin, register_jit_aliases
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)
from minilink.core.compile.step_execution_plan import StepExecutionPlan
from minilink.core.diagram import validate_diagram_params
from minilink.core.step_rollout import StepRollout
from minilink.core.system import DynamicSystem, StepSystem, System

# =============================================================================
# Public API — JaxIntegrationMixin (textbook)
# =============================================================================


class JaxIntegrationMixin:
    """RK4 / Euler integration on trace and JIT ``f`` tiers."""

    def _setup_integration_tiers(self, jax, jnp) -> None:
        """Bind trace-tier rollout closures; fast rollouts JIT lazily on first call."""
        f_trace = self._f_trace_fn
        f_trace_p = self._f_trace_p_fn
        f_ivp_trace = self._f_ivp_trace_fn
        f_ivp_trace_p = self._f_ivp_trace_p_fn

        def _rk4_integrate_ivp_trace_fn(x0_, t0_, dt_, n_steps_):
            def body(carry, _):
                x, t = carry
                k1 = f_ivp_trace(x, t)
                k2 = f_ivp_trace(x + 0.5 * dt_ * k1, t + 0.5 * dt_)
                k3 = f_ivp_trace(x + 0.5 * dt_ * k2, t + 0.5 * dt_)
                k4 = f_ivp_trace(x + dt_ * k3, t + dt_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _rk4_integrate_ivp_trace_p_fn(x0_, t0_, dt_, n_steps_, params_):
            def body(carry, _):
                x, t = carry
                k1 = f_ivp_trace_p(x, t, params_)
                k2 = f_ivp_trace_p(x + 0.5 * dt_ * k1, t + 0.5 * dt_, params_)
                k3 = f_ivp_trace_p(x + 0.5 * dt_ * k2, t + 0.5 * dt_, params_)
                k4 = f_ivp_trace_p(x + dt_ * k3, t + dt_, params_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _rk4_integrate_linear_trace_fn(x0_, u_knots_, t0_, dt_):
            def body(carry, u_pair):
                x, t = carry
                u0, u1 = u_pair
                umid = 0.5 * (u0 + u1)
                k1 = f_trace(x, u0, t)
                k2 = f_trace(x + 0.5 * dt_ * k1, umid, t + 0.5 * dt_)
                k3 = f_trace(x + 0.5 * dt_ * k2, umid, t + 0.5 * dt_)
                k4 = f_trace(x + dt_ * k3, u1, t + dt_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(
                body,
                (x0_, t0_),
                (u_knots_[:-1], u_knots_[1:]),
            )
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _rk4_integrate_linear_trace_p_fn(x0_, u_knots_, t0_, dt_, params_):
            def body(carry, u_pair):
                x, t = carry
                u0, u1 = u_pair
                umid = 0.5 * (u0 + u1)
                k1 = f_trace_p(x, u0, t, params_)
                k2 = f_trace_p(x + 0.5 * dt_ * k1, umid, t + 0.5 * dt_, params_)
                k3 = f_trace_p(x + 0.5 * dt_ * k2, umid, t + 0.5 * dt_, params_)
                k4 = f_trace_p(x + dt_ * k3, u1, t + dt_, params_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(
                body,
                (x0_, t0_),
                (u_knots_[:-1], u_knots_[1:]),
            )
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _rk4_integrate_zoh_trace_fn(x0_, u_sequence_, t0_, dt_):
            def body(carry, u_k):
                x, t = carry
                k1 = f_trace(x, u_k, t)
                k2 = f_trace(x + 0.5 * dt_ * k1, u_k, t + 0.5 * dt_)
                k3 = f_trace(x + 0.5 * dt_ * k2, u_k, t + 0.5 * dt_)
                k4 = f_trace(x + dt_ * k3, u_k, t + dt_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _rk4_integrate_zoh_trace_p_fn(x0_, u_sequence_, t0_, dt_, params_):
            def body(carry, u_k):
                x, t = carry
                k1 = f_trace_p(x, u_k, t, params_)
                k2 = f_trace_p(x + 0.5 * dt_ * k1, u_k, t + 0.5 * dt_, params_)
                k3 = f_trace_p(x + 0.5 * dt_ * k2, u_k, t + 0.5 * dt_, params_)
                k4 = f_trace_p(x + dt_ * k3, u_k, t + dt_, params_)
                x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _euler_integrate_zoh_trace_fn(x0_, u_sequence_, t0_, dt_):
            def body(carry, u_k):
                x, t = carry
                x_next = x + dt_ * f_trace(x, u_k, t)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _euler_integrate_zoh_trace_p_fn(x0_, u_sequence_, t0_, dt_, params_):
            def body(carry, u_k):
                x, t = carry
                x_next = x + dt_ * f_trace_p(x, u_k, t, params_)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _euler_integrate_ivp_trace_fn(x0_, t0_, dt_, n_steps_):
            def body(carry, _):
                x, t = carry
                x_next = x + dt_ * f_ivp_trace(x, t)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        def _euler_integrate_ivp_trace_p_fn(x0_, t0_, dt_, n_steps_, params_):
            def body(carry, _):
                x, t = carry
                x_next = x + dt_ * f_ivp_trace_p(x, t, params_)
                return (x_next, t + dt_), x_next

            (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
            return jnp.concatenate((x0_[None, :], xs), axis=0)

        self._rk4_integrate_ivp_trace_fn = _rk4_integrate_ivp_trace_fn
        self._rk4_integrate_ivp_trace_p_fn = _rk4_integrate_ivp_trace_p_fn
        self._rk4_integrate_linear_trace_fn = _rk4_integrate_linear_trace_fn
        self._rk4_integrate_linear_trace_p_fn = _rk4_integrate_linear_trace_p_fn
        self._rk4_integrate_zoh_trace_fn = _rk4_integrate_zoh_trace_fn
        self._rk4_integrate_zoh_trace_p_fn = _rk4_integrate_zoh_trace_p_fn
        self._euler_integrate_zoh_trace_fn = _euler_integrate_zoh_trace_fn
        self._euler_integrate_zoh_trace_p_fn = _euler_integrate_zoh_trace_p_fn
        self._euler_integrate_ivp_trace_fn = _euler_integrate_ivp_trace_fn
        self._euler_integrate_ivp_trace_p_fn = _euler_integrate_ivp_trace_p_fn

        # Lazy rollout JIT slots (bound on first call; no compile-time warm-start)
        self._jit_rk4_integrate_zoh = None
        self._jit_rk4_integrate_zoh_p = None
        self._jit_rk4_integrate_linear = None
        self._jit_rk4_integrate_linear_p = None
        self._jit_rk4_integrate_ivp = None
        self._jit_rk4_integrate_ivp_p = None
        self._jit_euler_integrate_zoh = None
        self._jit_euler_integrate_zoh_p = None
        self._jit_euler_integrate_ivp = None
        self._jit_euler_integrate_ivp_p = None

    # --- RK4 step (ZOH) ---

    def rk4_step(self, x, u, t, dt):
        f = self._f_jit_fn
        k1 = f(x, u, t)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_trace(self, x, u, t, dt):
        """Pre-JIT RK4 step for JAX composition."""
        f = self._f_trace_fn
        k1 = f(x, u, t)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_p(self, x, u, t, dt, params):
        f = self._f_jit_p_fn
        k1 = f(x, u, t, params)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, u, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_trace_p(self, x, u, t, dt, params):
        """Pre-JIT parametric RK4 step for JAX composition."""
        f = self._f_trace_p_fn
        k1 = f(x, u, t, params)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, u, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # --- Euler step (ZOH) ---

    def euler_step(self, x, u, t, dt):
        return x + dt * self._f_jit_fn(x, u, t)

    def euler_step_trace(self, x, u, t, dt):
        """Pre-JIT Euler step for JAX composition."""
        return x + dt * self._f_trace_fn(x, u, t)

    def euler_step_p(self, x, u, t, dt, params):
        return x + dt * self._f_jit_p_fn(x, u, t, params)

    def euler_step_trace_p(self, x, u, t, dt, params):
        """Pre-JIT parametric Euler step for JAX composition."""
        return x + dt * self._f_trace_p_fn(x, u, t, params)

    # --- RK4 step IVP ---

    def rk4_step_ivp(self, x, t, dt):
        f = self._f_ivp_jit_fn
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_ivp_trace(self, x, t, dt):
        """Pre-JIT IVP RK4 step for JAX composition."""
        f = self._f_ivp_trace_fn
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_ivp_p(self, x, t, dt, params):
        f = self._f_ivp_jit_p_fn
        k1 = f(x, t, params)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_step_ivp_trace_p(self, x, t, dt, params):
        """Pre-JIT parametric IVP RK4 step for JAX composition."""
        f = self._f_ivp_trace_p_fn
        k1 = f(x, t, params)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt, params)
        k4 = f(x + dt * k3, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # --- Euler step IVP ---

    def euler_step_ivp(self, x, t, dt):
        return x + dt * self._f_ivp_jit_fn(x, t)

    def euler_step_ivp_trace(self, x, t, dt):
        """Pre-JIT IVP Euler step for JAX composition."""
        return x + dt * self._f_ivp_trace_fn(x, t)

    def euler_step_ivp_p(self, x, t, dt, params):
        return x + dt * self._f_ivp_jit_p_fn(x, t, params)

    def euler_step_ivp_trace_p(self, x, t, dt, params):
        """Pre-JIT parametric IVP Euler step for JAX composition."""
        return x + dt * self._f_ivp_trace_p_fn(x, t, params)

    # --- RK4 integrate ZOH ---

    def rk4_integrate_zoh(self, x0, u_sequence, t0, dt):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_zoh
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_zoh_trace_fn)
            self._jit_rk4_integrate_zoh = fn
        return fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_zoh_trace(self, x0, u_sequence, t0, dt):
        """Pre-JIT ZOH-input rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_zoh_trace_fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_zoh_p(self, x0, u_sequence, t0, dt, params):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_zoh_p
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_zoh_trace_p_fn)
            self._jit_rk4_integrate_zoh_p = fn
        return fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def rk4_integrate_zoh_trace_p(self, x0, u_sequence, t0, dt, params):
        """Pre-JIT parametric ZOH-input rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_zoh_trace_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    # --- RK4 integrate linear ---

    def rk4_integrate_linear(self, x0, u_knots, t0, dt):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_linear
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_linear_trace_fn)
            self._jit_rk4_integrate_linear = fn
        return fn(
            jnp.asarray(x0), jnp.asarray(u_knots), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_linear_trace(self, x0, u_knots, t0, dt):
        """Pre-JIT linear-``u`` rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_linear_trace_fn(
            jnp.asarray(x0), jnp.asarray(u_knots), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_linear_p(self, x0, u_knots, t0, dt, params):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_linear_p
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_linear_trace_p_fn)
            self._jit_rk4_integrate_linear_p = fn
        return fn(
            jnp.asarray(x0),
            jnp.asarray(u_knots),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def rk4_integrate_linear_trace_p(self, x0, u_knots, t0, dt, params):
        """Pre-JIT parametric linear-``u`` rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_linear_trace_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_knots),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    # --- RK4 integrate IVP ---

    def rk4_integrate_ivp(self, x0, t0, dt, n_steps):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_ivp
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_ivp_trace_fn, static_argnums=3)
            self._jit_rk4_integrate_ivp = fn
        return fn(jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps)

    def rk4_integrate_ivp_trace(self, x0, t0, dt, n_steps):
        """Pre-JIT IVP rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_ivp_trace_fn(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps
        )

    def rk4_integrate_ivp_p(self, x0, t0, dt, n_steps, params):
        jnp = self.jnp
        fn = self._jit_rk4_integrate_ivp_p
        if fn is None:
            fn = self.jax.jit(self._rk4_integrate_ivp_trace_p_fn, static_argnums=3)
            self._jit_rk4_integrate_ivp_p = fn
        return fn(jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps, params)

    def rk4_integrate_ivp_trace_p(self, x0, t0, dt, n_steps, params):
        """Pre-JIT parametric IVP rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_ivp_trace_p_fn(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps, params
        )

    # --- Euler integrate ZOH ---

    def euler_integrate_zoh(self, x0, u_sequence, t0, dt):
        jnp = self.jnp
        fn = self._jit_euler_integrate_zoh
        if fn is None:
            fn = self.jax.jit(self._euler_integrate_zoh_trace_fn)
            self._jit_euler_integrate_zoh = fn
        return fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def euler_integrate_zoh_trace(self, x0, u_sequence, t0, dt):
        """Pre-JIT ZOH Euler rollout for JAX composition."""
        jnp = self.jnp
        return self._euler_integrate_zoh_trace_fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def euler_integrate_zoh_p(self, x0, u_sequence, t0, dt, params):
        jnp = self.jnp
        fn = self._jit_euler_integrate_zoh_p
        if fn is None:
            fn = self.jax.jit(self._euler_integrate_zoh_trace_p_fn)
            self._jit_euler_integrate_zoh_p = fn
        return fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def euler_integrate_zoh_trace_p(self, x0, u_sequence, t0, dt, params):
        """Pre-JIT parametric ZOH Euler rollout for JAX composition."""
        jnp = self.jnp
        return self._euler_integrate_zoh_trace_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    # --- Euler integrate IVP ---

    def euler_integrate_ivp(self, x0, t0, dt, n_steps):
        jnp = self.jnp
        fn = self._jit_euler_integrate_ivp
        if fn is None:
            fn = self.jax.jit(self._euler_integrate_ivp_trace_fn, static_argnums=3)
            self._jit_euler_integrate_ivp = fn
        return fn(jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps)

    def euler_integrate_ivp_trace(self, x0, t0, dt, n_steps):
        """Pre-JIT IVP Euler rollout for JAX composition."""
        jnp = self.jnp
        return self._euler_integrate_ivp_trace_fn(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps
        )

    def euler_integrate_ivp_p(self, x0, t0, dt, n_steps, params):
        jnp = self.jnp
        fn = self._jit_euler_integrate_ivp_p
        if fn is None:
            fn = self.jax.jit(self._euler_integrate_ivp_trace_p_fn, static_argnums=3)
            self._jit_euler_integrate_ivp_p = fn
        return fn(jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps, params)

    def euler_integrate_ivp_trace_p(self, x0, t0, dt, n_steps, params):
        """Pre-JIT parametric IVP Euler rollout for JAX composition."""
        jnp = self.jnp
        return self._euler_integrate_ivp_trace_p_fn(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps, params
        )

    # --- ZOH sugar ---

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

    # --- SciPy / IVP bridges ---

    def f_ivp(self, x, t=0.0):
        return self._f_ivp_jit_fn(x, t)

    def f_ivp_p(self, x, t, params):
        return self._f_ivp_jit_p_fn(x, t, params)

    def f_ivp_scipy(self, x, t=0.0):
        x = self.jnp.asarray(x)
        return np.asarray(self._f_ivp_jit_fn(x, t))

    def as_scipy_jac(self):
        return lambda t, x: np.asarray(self._jac_ivp_jit_fn(self.jnp.asarray(x), t))


register_jit_aliases(
    JaxIntegrationMixin,
    ("rk4_step", "rk4_integrate_zoh", "rk4_integrate_linear"),
)


# =============================================================================
# Public API — JaxDynamicEvaluator
# =============================================================================


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
            system.f, system.name, dummy_x, dummy_u, dummy_t, frozen_p, jax
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
        self._f_ivp_trace_p_fn = tiers["_f_ivp_trace_p_fn"]
        self._f_ivp_jit_p_fn = tiers["_f_ivp_jit_p_fn"]
        self._jac_f_params_jit_fn = tiers["_jac_f_params_jit_fn"]
        self._jac_ivp_jit_fn = tiers["_jac_ivp_jit_fn"]
        self._outputs_trace_fn = tiers["_outputs_trace_fn"]
        self._outputs_trace_p_fn = tiers["_outputs_trace_p_fn"]
        self._outputs_jit_fn = tiers["_outputs_jit_fn"]
        self._outputs_jit_p_fn = tiers["_outputs_jit_p_fn"]

        self._setup_integration_tiers(jax, jnp)

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


# =============================================================================
# Public API — JaxDiagramEvaluator
# =============================================================================


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

        self._f_jit_fn = jax.jit(self._f_trace_fn)
        u_nom = self._u_nominal
        self._f_ivp_trace_fn = lambda x, t: self._f_trace_fn(x, u_nom, t)
        self._f_ivp_jit_fn = jax.jit(self._f_ivp_trace_fn)
        self._jac_ivp_jit_fn = jax.jit(jax.jacfwd(self._f_ivp_jit_fn, argnums=0))

        self._outputs_jit_fn = jax.jit(self._outputs_trace_fn)
        self._internal_signals_jit_fn = jax.jit(self._internal_signals_trace_fn)

        self._f_jit_p_fn = jax.jit(self._f_trace_p_fn)
        self._outputs_jit_p_fn = jax.jit(self._outputs_trace_p_fn)
        self._jac_f_params_jit_fn = jax.jit(jax.jacfwd(self._f_trace_p_fn, argnums=3))

        def _f_ivp_trace_p_fn(x, t, p):
            return self._f_trace_p_fn(x, u_nom, t, p)

        self._f_ivp_trace_p_fn = _f_ivp_trace_p_fn
        self._f_ivp_jit_p_fn = jax.jit(_f_ivp_trace_p_fn)

        self._setup_integration_tiers(jax, jnp)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

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

    def _f_trace_fn(self, x, u, t=0.0):
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

    def _f_trace_p_fn(self, x, u, t, params):
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

    def _outputs_trace_p_fn(self, x, u, t, params):
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
        return self._internal_signals_jit_fn(x, u, t)

    def _outputs_trace_fn(self, x, u, t=0.0):
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals(x, u, t, dtype)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def _internal_signals_trace_fn(self, x, u, t=0.0):
        dtype = self._infer_dtype(x, u)
        return self._compute_port_signals(x, u, t, dtype)

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


# =============================================================================
# Public API — JaxStaticEvaluator
# =============================================================================


class JaxStaticEvaluator(StaticEvaluator, TraceTierMixin):
    """JAX evaluator for a static ``System`` leaf."""

    def __init__(self, system: System, verbose=False):
        if isinstance(system, DynamicSystem):
            raise TypeError("JaxStaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")

        import jax
        import jax.numpy as jnp

        self.jax = jax
        self.jnp = jnp
        self.n = 0
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        frozen_p = self._frozen_params
        dummy_x = jnp.zeros(0)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
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
                f"[compile] Step 1: JIT-compiling static outputs on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        tiers = build_static_output_tiers(jax, system, frozen_p)
        self._outputs_trace_fn = tiers["_outputs_trace_fn"]
        self._outputs_trace_p_fn = tiers["_outputs_trace_p_fn"]
        self._outputs_jit_fn = tiers["_outputs_jit_fn"]
        self._outputs_jit_p_fn = tiers["_outputs_jit_p_fn"]

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    def outputs(self, x, u, t=0.0):
        return self._outputs_jit_fn(x, u, t)

    def outputs_trace(self, x, u, t=0.0):
        """Pre-JIT boundary outputs for JAX composition."""
        return self._outputs_trace_fn(x, u, t)

    def outputs_p(self, x, u, t, params):
        return self._outputs_jit_p_fn(x, u, t, params)

    def outputs_trace_p(self, x, u, t, params):
        """Pre-JIT parametric boundary outputs for JAX composition."""
        return self._outputs_trace_p_fn(x, u, t, params)


register_jit_aliases(JaxStaticEvaluator, ("outputs",))


# =============================================================================
# Public API — JaxStepEvaluator
# =============================================================================


class JaxStepEvaluator(StepEvaluator, StepRolloutMixin, TraceTierMixin):
    """Compiled evaluator for a :class:`StepSystem` using JAX."""

    def __init__(self, system: StepSystem, verbose=False):
        if not isinstance(system, StepSystem):
            raise TypeError("JaxStepEvaluator requires StepSystem")

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

        frozen_p = self._frozen_params
        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_k = 0

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_jax_compatible(
            system.step, system.name, dummy_x, dummy_u, dummy_k, frozen_p, jax
        )
        for port_id, port in system.outputs.items():
            check_jax_compatible(
                port.compute,
                f"{system.name}:{port_id}",
                dummy_x,
                dummy_u,
                dummy_k,
                frozen_p,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        tiers = build_step_leaf_tiers(jax, system, frozen_p)
        self._step_trace_fn = tiers["_step_trace_fn"]
        self._step_trace_p_fn = tiers["_step_trace_p_fn"]
        self._step_jit_fn = tiers["_step_jit_fn"]
        self._step_jit_p_fn = tiers["_step_jit_p_fn"]
        self._outputs_trace_fn = tiers["_outputs_trace_fn"]
        self._outputs_trace_p_fn = tiers["_outputs_trace_p_fn"]
        self._outputs_jit_fn = tiers["_outputs_jit_fn"]
        self._outputs_jit_p_fn = tiers["_outputs_jit_p_fn"]
        self._rollout_jit_fn = build_jit_step_rollout(jax, jnp, self._step_jit_fn)

    def step(self, x, u, k=0):
        return self._step_jit_fn(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def step_trace(self, x, u, k=0):
        """Pre-JIT flat step for JAX composition."""
        return self._step_trace_fn(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def step_p(self, x, u, k, params):
        return self._step_jit_p_fn(self.jnp.asarray(x), self.jnp.asarray(u), k, params)

    def step_trace_p(self, x, u, k, params):
        """Pre-JIT parametric step for JAX composition."""
        return self._step_trace_p_fn(
            self.jnp.asarray(x), self.jnp.asarray(u), k, params
        )

    def outputs(self, x, u, k=0):
        return self._outputs_jit_fn(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def outputs_trace(self, x, u, k=0):
        """Pre-JIT boundary outputs for JAX composition."""
        return self._outputs_trace_fn(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def outputs_p(self, x, u, k, params):
        return self._outputs_jit_p_fn(
            self.jnp.asarray(x), self.jnp.asarray(u), k, params
        )

    def outputs_trace_p(self, x, u, k, params):
        """Pre-JIT parametric boundary outputs for JAX composition."""
        return self._outputs_trace_p_fn(
            self.jnp.asarray(x), self.jnp.asarray(u), k, params
        )

    def rollout(self, x0, *, n_steps, u=None):
        jnp = self.jnp
        x0 = jnp.asarray(x0, dtype=float).reshape(self.n)
        u_samples = self._coerce_rollout_inputs(n_steps, u)
        if n_steps == 0:
            x_path = x0.reshape(1, self.n)
        else:
            u_steps = jnp.asarray(u_samples[:, :n_steps].T)
            x_path = self._rollout_jit_fn(x0, u_steps)
        x_samples = np.asarray(x_path, dtype=float).T
        k = np.arange(n_steps + 1, dtype=float)
        u_np = np.asarray(u_samples, dtype=float)
        return StepRollout(k=k, x=x_samples, u=u_np)


register_jit_aliases(JaxStepEvaluator, ("step", "outputs"))


# =============================================================================
# Public API — JaxStepDiagramEvaluator
# =============================================================================


class JaxStepDiagramEvaluator(StepEvaluator, StepRolloutMixin, TraceTierMixin):
    """JAX evaluator for a compiled step diagram."""

    def __init__(self, plan: StepExecutionPlan, diagram, verbose=False):
        import jax
        import jax.numpy as jnp

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
        self._frozen_params = None
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

        self._step_trace_fn = self._make_step_trace_fn()
        self._step_trace_p_fn = self._make_step_trace_p_fn()
        self._outputs_trace_fn = self._make_outputs_trace_fn()
        self._outputs_trace_p_fn = self._make_outputs_trace_p_fn()
        self._step_jit_fn = jax.jit(self._step_trace_fn)
        self._step_jit_p_fn = jax.jit(self._step_trace_p_fn)
        self._outputs_jit_fn = jax.jit(self._outputs_trace_fn)
        self._outputs_jit_p_fn = jax.jit(self._outputs_trace_p_fn)
        self._rollout_jit_fn = build_jit_step_rollout(jax, jnp, self._step_jit_fn)

    def _make_step_trace_fn(self):
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype

        def _step(x, u, k):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_work = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_work[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op.bound_params)
                )
            x_new = x_work
            for op in plan.step_ops:
                local_x = x_new[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
                x_new = x_new.at[op.local_x_slice].set(
                    op.step_func(local_x, local_u, k, op.bound_params)
                )
            return x_new

        return _step

    def _make_step_trace_p_fn(self):
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype

        def _step_p(x, u, k, params):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_work = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_work[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
                op_params = None if params is None else params.get(op.sys_id)
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op_params)
                )
            x_new = x_work
            for op in plan.step_ops:
                local_x = x_new[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
                op_params = None if params is None else params.get(op.sys_id)
                x_new = x_new.at[op.local_x_slice].set(
                    op.step_func(local_x, local_u, k, op_params)
                )
            return x_new

        return _step_p

    def _make_outputs_trace_fn(self):
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype

        def _outputs(x, u, k):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_arr = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_arr[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
                signals = signals.at[op.out_slice].set(
                    op.compute_func(local_x, local_u, k, op.bound_params)
                )
            return {
                port_id: signals[sl]
                for port_id, sl in plan.external_output_slices.items()
            }

        return _outputs

    def _make_outputs_trace_p_fn(self):
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype

        def _outputs_p(x, u, k, params):
            signals = jnp.zeros(plan.signal_dim, dtype=dtype)
            x_arr = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
            for op in plan.port_ops:
                local_x = x_arr[op.local_x_slice]
                local_u = gather_u_jax(
                    op.gather_sources, op.u_dim, signals, u, jnp, dtype
                )
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

    def rollout(self, x0, *, n_steps, u=None):
        jnp = self.jnp
        x0 = jnp.asarray(x0, dtype=float).reshape(self.n)
        u_samples = self._coerce_rollout_inputs(n_steps, u)
        if n_steps == 0:
            x_path = x0.reshape(1, self.n)
        else:
            u_steps = jnp.asarray(u_samples[:, :n_steps].T)
            x_path = self._rollout_jit_fn(x0, u_steps)
        x_samples = np.asarray(x_path, dtype=float).T
        k = np.arange(n_steps + 1, dtype=float)
        u_np = np.asarray(u_samples, dtype=float)
        return StepRollout(k=k, x=x_samples, u=u_np)


register_jit_aliases(JaxStepDiagramEvaluator, ("step", "outputs"))


# =============================================================================
# Internal machinery
# =============================================================================


def check_jax_compatible(func, label, dummy_x, dummy_u, dummy_t, params, jax):
    """Test if *func* is JAX-traceable via ``jax.make_jaxpr``."""
    from jax.errors import ConcretizationTypeError

    try:
        jax.make_jaxpr(lambda x, u, t: func(x, u, t, params))(dummy_x, dummy_u, dummy_t)
    except (ConcretizationTypeError, TypeError, Exception) as e:
        raise RuntimeError(
            f"\n\nBlock '{label}' is not JAX-traceable.\n"
            f"Its f()/compute() likely performs in-place array mutation "
            f"(e.g., x[0] = ...)\n"
            f"or uses Python control flow on traced values "
            f"(e.g., if t < threshold).\n"
            f"Rewrite in purely functional style or use backend='numpy'.\n"
            f"Original JAX error: {e}"
        ) from e


def gather_u_jax(gather_sources, u_dim, signals, u, jnp, dtype):
    """Assemble the local input vector using JAX operations."""
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


def build_dynamic_leaf_tiers(jax, system, frozen_p, u_nom):
    """Build fast (JIT) and trace (pre-JIT) tiers for a :class:`DynamicSystem` leaf."""
    f_raw = system.f
    output_items = tuple((pid, port.compute) for pid, port in system.outputs.items())

    def _f_trace_fn(x, u, t):
        return f_raw(x, u, t, frozen_p)

    def _f_trace_p_fn(x, u, t, p):
        return f_raw(x, u, t, p)

    _f_jit_fn = jax.jit(_f_trace_fn)
    _f_jit_p_fn = jax.jit(_f_trace_p_fn)

    def _f_ivp_trace_fn(x, t):
        return _f_trace_fn(x, u_nom, t)

    _f_ivp_jit_fn = jax.jit(_f_ivp_trace_fn)

    def _f_ivp_trace_p_fn(x, t, p):
        return _f_trace_p_fn(x, u_nom, t, p)

    _f_ivp_jit_p_fn = jax.jit(_f_ivp_trace_p_fn)
    _jac_f_params_jit_fn = jax.jit(jax.jacfwd(_f_trace_p_fn, argnums=3))
    _jac_ivp_jit_fn = jax.jit(jax.jacfwd(_f_ivp_jit_fn, argnums=0))

    def _outputs_trace_fn(x, u, t):
        return {pid: fn(x, u, t, frozen_p) for pid, fn in output_items}

    def _outputs_trace_p_fn(x, u, t, p):
        return {pid: fn(x, u, t, p) for pid, fn in output_items}

    if output_items:
        _outputs_jit_fn = jax.jit(_outputs_trace_fn)
        _outputs_jit_p_fn = jax.jit(_outputs_trace_p_fn)
    else:

        def _outputs_trace_fn(x, u, t):
            return {}

        def _outputs_trace_p_fn(x, u, t, p):
            return {}

        _outputs_jit_fn = jax.jit(_outputs_trace_fn)
        _outputs_jit_p_fn = jax.jit(_outputs_trace_p_fn)

    return {
        "_f_trace_fn": _f_trace_fn,
        "_f_trace_p_fn": _f_trace_p_fn,
        "_f_jit_fn": _f_jit_fn,
        "_f_jit_p_fn": _f_jit_p_fn,
        "_f_ivp_trace_fn": _f_ivp_trace_fn,
        "_f_ivp_jit_fn": _f_ivp_jit_fn,
        "_f_ivp_trace_p_fn": _f_ivp_trace_p_fn,
        "_f_ivp_jit_p_fn": _f_ivp_jit_p_fn,
        "_jac_f_params_jit_fn": _jac_f_params_jit_fn,
        "_jac_ivp_jit_fn": _jac_ivp_jit_fn,
        "_outputs_trace_fn": _outputs_trace_fn,
        "_outputs_trace_p_fn": _outputs_trace_p_fn,
        "_outputs_jit_fn": _outputs_jit_fn,
        "_outputs_jit_p_fn": _outputs_jit_p_fn,
    }


def build_static_output_tiers(jax, system, frozen_p):
    """Build fast (JIT) and trace tiers for a static ``System`` (no ``f``)."""
    output_items = tuple((pid, port.compute) for pid, port in system.outputs.items())

    def _outputs_trace_fn(x, u, t):
        return {pid: fn(x, u, t, frozen_p) for pid, fn in output_items}

    def _outputs_trace_p_fn(x, u, t, p):
        return {pid: fn(x, u, t, p) for pid, fn in output_items}

    if output_items:
        return {
            "_outputs_trace_fn": _outputs_trace_fn,
            "_outputs_trace_p_fn": _outputs_trace_p_fn,
            "_outputs_jit_fn": jax.jit(_outputs_trace_fn),
            "_outputs_jit_p_fn": jax.jit(_outputs_trace_p_fn),
        }

    def _empty_trace(x, u, t):
        return {}

    def _empty_trace_p(x, u, t, p):
        return {}

    return {
        "_outputs_trace_fn": _empty_trace,
        "_outputs_trace_p_fn": _empty_trace_p,
        "_outputs_jit_fn": jax.jit(_empty_trace),
        "_outputs_jit_p_fn": jax.jit(_empty_trace_p),
    }


def build_step_leaf_tiers(jax, system, frozen_p):
    """Build fast (JIT) and trace tiers for a :class:`StepSystem` leaf."""
    step_raw = system.step
    output_items = tuple((pid, port.compute) for pid, port in system.outputs.items())

    def _step_trace_fn(x, u, k):
        return step_raw(x, u, k, frozen_p)

    def _step_trace_p_fn(x, u, k, p):
        return step_raw(x, u, k, p)

    _step_jit_fn = jax.jit(_step_trace_fn)
    _step_jit_p_fn = jax.jit(_step_trace_p_fn)

    def _outputs_trace_fn(x, u, k):
        return {pid: fn(x, u, k, frozen_p) for pid, fn in output_items}

    def _outputs_trace_p_fn(x, u, k, p):
        return {pid: fn(x, u, k, p) for pid, fn in output_items}

    if output_items:
        _outputs_jit_fn = jax.jit(_outputs_trace_fn)
        _outputs_jit_p_fn = jax.jit(_outputs_trace_p_fn)
    else:

        def _outputs_trace_fn(x, u, k):
            return {}

        def _outputs_trace_p_fn(x, u, k, p):
            return {}

        _outputs_jit_fn = jax.jit(_outputs_trace_fn)
        _outputs_jit_p_fn = jax.jit(_outputs_trace_p_fn)

    return {
        "_step_trace_fn": _step_trace_fn,
        "_step_trace_p_fn": _step_trace_p_fn,
        "_step_jit_fn": _step_jit_fn,
        "_step_jit_p_fn": _step_jit_p_fn,
        "_outputs_trace_fn": _outputs_trace_fn,
        "_outputs_trace_p_fn": _outputs_trace_p_fn,
        "_outputs_jit_fn": _outputs_jit_fn,
        "_outputs_jit_p_fn": _outputs_jit_p_fn,
    }


def build_jit_step_rollout(jax, jnp, jit_step):
    """JIT a scan-based rollout for a compiled step function."""

    def _rollout(x0, u_steps):
        ks = jnp.arange(u_steps.shape[0])

        def body(x, inputs):
            k, u_k = inputs
            x_next = jit_step(x, u_k, k)
            return x_next, x_next

        _, xs = jax.lax.scan(body, x0, (ks, u_steps))
        return jnp.concatenate((x0[None, :], xs), axis=0)

    return jax.jit(_rollout)
