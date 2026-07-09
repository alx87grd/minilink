"""Shared JAX compile helpers for dynamics evaluators."""

from __future__ import annotations

import numpy as np

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


def build_jit_rk4_integrate_ivp(jax, jnp, f_ivp):
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

    return jax.jit(_rk4_integrate_ivp, static_argnums=3)


def build_jit_rk4_integrate_forced(jax, jnp, f):
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

    return jax.jit(_rk4_integrate_forced)


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


def build_jit_static_outputs(jax, system, frozen_p):
    """JIT output-tier callables for a static ``System`` (no ``f``)."""
    output_items = tuple((pid, port.compute) for pid, port in system.outputs.items())

    def _outputs_frozen(x, u, t):
        return {pid: fn(x, u, t, frozen_p) for pid, fn in output_items}

    def _outputs_param(x, u, t, p):
        return {pid: fn(x, u, t, p) for pid, fn in output_items}

    if output_items:
        return jax.jit(_outputs_frozen), jax.jit(_outputs_param)
    empty = jax.jit(lambda x, u, t: {})
    empty_p = jax.jit(lambda x, u, t, p: {})
    return empty, empty_p


def build_jit_dynamic_leaf(jax, system, frozen_p, u_nom):
    """JIT ``f`` and output tiers for a :class:`DynamicSystem` leaf."""
    f_raw = system.f
    output_items = tuple((pid, port.compute) for pid, port in system.outputs.items())

    jit_f = jax.jit(lambda x, u, t: f_raw(x, u, t, frozen_p))
    jit_f_p = jax.jit(lambda x, u, t, p: f_raw(x, u, t, p))
    jit_f_ivp = jax.jit(lambda x, t: f_raw(x, u_nom, t, frozen_p))
    jit_jac_f_params = jax.jit(
        jax.jacfwd(lambda x, u, t, p: f_raw(x, u, t, p), argnums=3)
    )
    jit_jac_ivp = jax.jit(jax.jacfwd(jit_f_ivp, argnums=0))

    def _outputs_frozen(x, u, t):
        return {pid: fn(x, u, t, frozen_p) for pid, fn in output_items}

    def _outputs_param(x, u, t, p):
        return {pid: fn(x, u, t, p) for pid, fn in output_items}

    if output_items:
        jit_outputs = jax.jit(_outputs_frozen)
        jit_outputs_p = jax.jit(_outputs_param)
    else:
        jit_outputs = jax.jit(lambda x, u, t: {})
        jit_outputs_p = jax.jit(lambda x, u, t, p: {})

    return {
        "f": jit_f,
        "f_p": jit_f_p,
        "f_ivp": jit_f_ivp,
        "jac_f_params": jit_jac_f_params,
        "jac_ivp": jit_jac_ivp,
        "outputs": jit_outputs,
        "outputs_p": jit_outputs_p,
    }


class JaxIntegrationMixin:
    """Shared IVP integration overrides for JAX dynamic evaluators."""

    def f_ivp(self, x, t=0.0):
        return self._jit_f_ivp(x, t)

    def f_ivp_scipy(self, x, t=0.0):
        x = self.jnp.asarray(x)
        return np.asarray(self._jit_f_ivp(x, t))

    def as_scipy_jac(self):
        return lambda t, x: np.asarray(self._jit_jac_ivp(self.jnp.asarray(x), t))

    def rk4_step_ivp(self, x, t, dt):
        f = self._jit_f_ivp
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_integrate_ivp(self, x0, t0, dt, n_steps):
        jnp = self.jnp
        return self._jit_rk4_integrate_ivp(
            jnp.asarray(x0), jnp.asarray(t0), jnp.asarray(dt), n_steps
        )

    def rk4_integrate_forced(self, x0, u_knots, t0, dt):
        jnp = self.jnp
        return self._jit_rk4_integrate_forced(
            jnp.asarray(x0), jnp.asarray(u_knots), jnp.asarray(t0), jnp.asarray(dt)
        )
