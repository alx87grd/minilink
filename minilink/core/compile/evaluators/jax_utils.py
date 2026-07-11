"""Shared JAX compile helpers for dynamics evaluators."""

from __future__ import annotations

import numpy as np

from minilink.core.compile.evaluators.tiers import register_jit_aliases
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
)


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


def _rk4_step_from_f(f):
    """One RK4 step ``(x, u, t, dt) -> x_next`` using dynamics callable ``f(x, u, t)``."""

    def rk4_step(x, u, t, dt):
        k1 = f(x, u, t)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return rk4_step


def _rk4_step_ivp_from_f(f):
    """One RK4 IVP step ``(x, t, dt) -> x_next`` using ``f(x, t)``."""

    def rk4_step_ivp(x, t, dt):
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return rk4_step_ivp


def build_trace_rk4_integrate_ivp(jax, jnp, f_ivp):
    """Pre-JIT IVP rollout — single fused scan over ``f_ivp``."""

    def _rk4_integrate_ivp(x0_, t0_, dt_, n_steps_):
        def body(carry, _):
            x, t = carry
            k1 = f_ivp(x, t)
            k2 = f_ivp(x + 0.5 * dt_ * k1, t + 0.5 * dt_)
            k3 = f_ivp(x + 0.5 * dt_ * k2, t + 0.5 * dt_)
            k4 = f_ivp(x + dt_ * k3, t + dt_)
            x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return _rk4_integrate_ivp


def build_jit_rk4_integrate_ivp(jax, jnp, f_ivp):
    return jax.jit(build_trace_rk4_integrate_ivp(jax, jnp, f_ivp), static_argnums=3)


def build_trace_rk4_integrate_forced(jax, jnp, f):
    """Pre-JIT forced rollout — single fused scan over ``f``."""

    def _rk4_integrate_forced(x0_, u_knots_, t0_, dt_):
        def body(carry, u_pair):
            x, t = carry
            u0, u1 = u_pair
            umid = 0.5 * (u0 + u1)
            k1 = f(x, u0, t)
            k2 = f(x + 0.5 * dt_ * k1, umid, t + 0.5 * dt_)
            k3 = f(x + 0.5 * dt_ * k2, umid, t + 0.5 * dt_)
            k4 = f(x + dt_ * k3, u1, t + dt_)
            x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(
            body,
            (x0_, t0_),
            (u_knots_[:-1], u_knots_[1:]),
        )
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return _rk4_integrate_forced


def build_jit_rk4_integrate_forced(jax, jnp, f):
    return jax.jit(build_trace_rk4_integrate_forced(jax, jnp, f))


def build_trace_rk4_integrate_forced_p(jax, jnp, f_p):
    """Pre-JIT parametric forced rollout."""

    def _rk4_integrate_forced_p(x0_, u_knots_, t0_, dt_, params_):
        def body(carry, u_pair):
            x, t = carry
            u0, u1 = u_pair
            umid = 0.5 * (u0 + u1)
            k1 = f_p(x, u0, t, params_)
            k2 = f_p(x + 0.5 * dt_ * k1, umid, t + 0.5 * dt_, params_)
            k3 = f_p(x + 0.5 * dt_ * k2, umid, t + 0.5 * dt_, params_)
            k4 = f_p(x + dt_ * k3, u1, t + dt_, params_)
            x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(
            body,
            (x0_, t0_),
            (u_knots_[:-1], u_knots_[1:]),
        )
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return _rk4_integrate_forced_p


def build_jit_rk4_integrate_forced_p(jax, jnp, f_p):
    return jax.jit(build_trace_rk4_integrate_forced_p(jax, jnp, f_p))


def build_trace_integrate(jax, jnp, f):
    """Pre-JIT ZOH input rollout over ``u_sequence`` with shape ``(N, m)``."""
    rk4_step = _rk4_step_from_f(f)

    def _integrate(x0_, u_sequence_, t0_, dt_):
        def body(carry, u_k):
            x, t = carry
            x_next = rk4_step(x, u_k, t, dt_)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return _integrate


def build_jit_integrate(jax, jnp, f):
    return jax.jit(build_trace_integrate(jax, jnp, f))


def build_trace_integrate_p(jax, jnp, f_p):
    """Pre-JIT parametric ZOH input rollout."""

    def rk4_step_p(x, u, t, dt, params):
        k1 = f_p(x, u, t, params)
        k2 = f_p(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
        k3 = f_p(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
        k4 = f_p(x + dt * k3, u, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _integrate_p(x0_, u_sequence_, t0_, dt_, params_):
        def body(carry, u_k):
            x, t = carry
            x_next = rk4_step_p(x, u_k, t, dt_, params_)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(body, (x0_, t0_), u_sequence_)
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return _integrate_p


def build_jit_integrate_p(jax, jnp, f_p):
    return jax.jit(build_trace_integrate_p(jax, jnp, f_p))


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


def build_jit_static_outputs(jax, system, frozen_p):
    """JIT output-tier callables for a static ``System`` (no ``f``)."""
    tiers = build_static_output_tiers(jax, system, frozen_p)
    return tiers["_outputs_jit_fn"], tiers["_outputs_jit_p_fn"]


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
        "_jac_f_params_jit_fn": _jac_f_params_jit_fn,
        "_jac_ivp_jit_fn": _jac_ivp_jit_fn,
        "_outputs_trace_fn": _outputs_trace_fn,
        "_outputs_trace_p_fn": _outputs_trace_p_fn,
        "_outputs_jit_fn": _outputs_jit_fn,
        "_outputs_jit_p_fn": _outputs_jit_p_fn,
    }


def build_jit_dynamic_leaf(jax, system, frozen_p, u_nom):
    """JIT ``f`` and output tiers for a :class:`DynamicSystem` leaf."""
    tiers = build_dynamic_leaf_tiers(jax, system, frozen_p, u_nom)
    return {
        "f": tiers["_f_jit_fn"],
        "f_p": tiers["_f_jit_p_fn"],
        "f_ivp": tiers["_f_ivp_jit_fn"],
        "jac_f_params": tiers["_jac_f_params_jit_fn"],
        "jac_ivp": tiers["_jac_ivp_jit_fn"],
        "outputs": tiers["_outputs_jit_fn"],
        "outputs_p": tiers["_outputs_jit_p_fn"],
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


def build_jit_step_leaf(jax, system, frozen_p):
    """JIT ``step`` and output tiers for a :class:`StepSystem` leaf."""
    tiers = build_step_leaf_tiers(jax, system, frozen_p)
    return {
        "step": tiers["_step_jit_fn"],
        "step_p": tiers["_step_jit_p_fn"],
        "outputs": tiers["_outputs_jit_fn"],
        "outputs_p": tiers["_outputs_jit_p_fn"],
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


class JaxIntegrationMixin:
    """Shared IVP integration overrides for JAX dynamic evaluators."""

    def _setup_integration_tiers(self, jax, jnp) -> None:
        """Bind fast (JIT) and trace integration callables after ``f`` tiers exist."""
        self._rk4_integrate_ivp_trace_fn = build_trace_rk4_integrate_ivp(
            jax, jnp, self._f_ivp_trace_fn
        )
        self._rk4_integrate_forced_trace_fn = build_trace_rk4_integrate_forced(
            jax, jnp, self._f_trace_fn
        )
        self._rk4_integrate_forced_trace_p_fn = build_trace_rk4_integrate_forced_p(
            jax, jnp, self._f_trace_p_fn
        )
        self._integrate_trace_fn = build_trace_integrate(jax, jnp, self._f_trace_fn)
        self._integrate_trace_p_fn = build_trace_integrate_p(
            jax, jnp, self._f_trace_p_fn
        )
        self._jit_rk4_integrate_ivp = build_jit_rk4_integrate_ivp(
            jax, jnp, self._f_ivp_jit_fn
        )
        self._jit_rk4_integrate_forced = build_jit_rk4_integrate_forced(
            jax, jnp, self._f_jit_fn
        )
        self._jit_rk4_integrate_forced_p = build_jit_rk4_integrate_forced_p(
            jax, jnp, self._f_jit_p_fn
        )
        self._integrate_jit_fn = build_jit_integrate(jax, jnp, self._f_jit_fn)
        self._integrate_jit_p_fn = build_jit_integrate_p(jax, jnp, self._f_jit_p_fn)

    def f_ivp(self, x, t=0.0):
        return self._f_ivp_jit_fn(x, t)

    def f_ivp_scipy(self, x, t=0.0):
        x = self.jnp.asarray(x)
        return np.asarray(self._f_ivp_jit_fn(x, t))

    def as_scipy_jac(self):
        return lambda t, x: np.asarray(self._jac_ivp_jit_fn(self.jnp.asarray(x), t))

    def rk4_step(self, x, u, t, dt):
        return _rk4_step_from_f(self._f_jit_fn)(x, u, t, dt)

    def rk4_step_trace(self, x, u, t, dt):
        """Pre-JIT RK4 step for JAX composition."""
        return _rk4_step_from_f(self._f_trace_fn)(x, u, t, dt)

    def rk4_step_p(self, x, u, t, dt, params):
        return _rk4_step_from_f(
            lambda x_, u_, t_: self._f_jit_p_fn(x_, u_, t_, params)
        )(x, u, t, dt)

    def rk4_step_trace_p(self, x, u, t, dt, params):
        """Pre-JIT parametric RK4 step for JAX composition."""
        return _rk4_step_from_f(
            lambda x_, u_, t_: self._f_trace_p_fn(x_, u_, t_, params)
        )(x, u, t, dt)

    def euler_step(self, x, u, t, dt):
        return x + dt * self._f_jit_fn(x, u, t)

    def euler_step_trace(self, x, u, t, dt):
        """Pre-JIT Euler step for JAX composition."""
        return x + dt * self._f_trace_fn(x, u, t)

    def rk4_step_ivp(self, x, t, dt):
        return _rk4_step_ivp_from_f(self._f_ivp_jit_fn)(x, t, dt)

    def rk4_step_ivp_trace(self, x, t, dt):
        """Pre-JIT IVP RK4 step for JAX composition."""
        return _rk4_step_ivp_from_f(self._f_ivp_trace_fn)(x, t, dt)

    def euler_step_ivp(self, x, t, dt):
        return x + dt * self._f_ivp_jit_fn(x, t)

    def euler_step_ivp_trace(self, x, t, dt):
        """Pre-JIT IVP Euler step for JAX composition."""
        return x + dt * self._f_ivp_trace_fn(x, t)

    def rk4_integrate_ivp(self, x0, t0, dt, n_steps):
        jnp = self.jnp
        return self._jit_rk4_integrate_ivp(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps
        )

    def rk4_integrate_ivp_trace(self, x0, t0, dt, n_steps):
        """Pre-JIT IVP rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_ivp_trace_fn(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps
        )

    def rk4_integrate_forced(self, x0, u_knots, t0, dt):
        jnp = self.jnp
        return self._jit_rk4_integrate_forced(
            jnp.asarray(x0), jnp.asarray(u_knots), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_forced_trace(self, x0, u_knots, t0, dt):
        """Pre-JIT forced rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_forced_trace_fn(
            jnp.asarray(x0), jnp.asarray(u_knots), jnp.asarray(t0), jnp.asarray(dt)
        )

    def rk4_integrate_forced_p(self, x0, u_knots, t0, dt, params):
        jnp = self.jnp
        return self._jit_rk4_integrate_forced_p(
            jnp.asarray(x0),
            jnp.asarray(u_knots),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def rk4_integrate_forced_trace_p(self, x0, u_knots, t0, dt, params):
        """Pre-JIT parametric forced rollout for JAX composition."""
        jnp = self.jnp
        return self._rk4_integrate_forced_trace_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_knots),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def integrate(self, x0, u_sequence, t0, dt):
        jnp = self.jnp
        return self._integrate_jit_fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def integrate_trace(self, x0, u_sequence, t0, dt):
        """Pre-JIT ZOH-input rollout for JAX composition."""
        jnp = self.jnp
        return self._integrate_trace_fn(
            jnp.asarray(x0), jnp.asarray(u_sequence), jnp.asarray(t0), jnp.asarray(dt)
        )

    def integrate_p(self, x0, u_sequence, t0, dt, params):
        jnp = self.jnp
        return self._integrate_jit_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )

    def integrate_trace_p(self, x0, u_sequence, t0, dt, params):
        """Pre-JIT parametric ZOH-input rollout for JAX composition."""
        jnp = self.jnp
        return self._integrate_trace_p_fn(
            jnp.asarray(x0),
            jnp.asarray(u_sequence),
            jnp.asarray(t0),
            jnp.asarray(dt),
            params,
        )


register_jit_aliases(
    JaxIntegrationMixin,
    ("rk4_step", "integrate", "rk4_integrate_forced"),
)
