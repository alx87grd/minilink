"""Frozen step vs params-as-argument vs bound params, in the situations that matter.

Run: ``PYTHONPATH=. python benchmarks/params_binding.py`` (needs JAX). The numbers it
prints are the evidence of docs/plans/sampled-path.md §3.5 (the step model, D10 of
docs/plans/end-state-api.md): one step called from Python, one scan, a vmapped batch, a
parameter family, and a gradient through the rollout, for the pendulum and the cart-pole.
"""

import time
from functools import partial

import jax
import jax.numpy as jnp

jax.config.update("jax_enable_x64", True)
sys.path.insert(0, ".")
from minilink import CartPole, Pendulum  # noqa: E402


def best(fn, repeats=7, inner=200):
    fn()  # warm up / compile
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        for _ in range(inner):
            out = fn()
        jax.block_until_ready(out)
        times.append((time.perf_counter() - t0) / inner)
    return min(times)


def bench(plant):
    ev = plant.compile(backend="jax")
    params = {k: jnp.asarray(v) for k, v in plant.params.items()}
    x0 = jnp.asarray(plant.x0, dtype=float)
    u0 = jnp.zeros(plant.m)
    dt = 0.01
    N, B = 1000, 1000

    step_frozen = jax.jit(lambda x, u, t: ev.rk4_step_trace(x, u, t, dt))
    step_p = jax.jit(lambda x, u, t, p: ev.rk4_step_trace_p(x, u, t, dt, p))
    step_bound = jax.jit(
        partial(lambda x, u, t, p: ev.rk4_step_trace_p(x, u, t, dt, p), p=params)
    )
    # params as one flat vector (what a pytree flatten costs vs what a dict costs)
    keys = list(params)
    flat = jnp.stack([params[k] for k in keys])

    def step_flat_fn(x, u, t, pv):
        return ev.rk4_step_trace_p(x, u, t, dt, dict(zip(keys, pv)))

    step_flat = jax.jit(step_flat_fn)

    print(f"\n== {type(plant).__name__}: n={plant.n}, {len(params)} params ==")
    print("-- one step called from Python (realtime loop, gym bridge, RRT extender)")
    for name, fn in [
        ("frozen jit                ", lambda: step_frozen(x0, u0, 0.0)),
        ("params dict each call     ", lambda: step_p(x0, u0, 0.0, params)),
        ("params bound (partial+jit)", lambda: step_bound(x0, u0, 0.0)),
        ("params as one flat vector ", lambda: step_flat(x0, u0, 0.0, flat)),
    ]:
        print(f"   {name}: {best(fn) * 1e6:7.1f} us")
    # where the dict cost goes: dispatch of a trivial jitted fn with / without the dict
    trivial = jax.jit(lambda x: x + 1.0)
    trivial_p = jax.jit(lambda x, p: x + p[keys[0]] * 0.0)
    print(
        f"   [dispatch only] trivial jit      : {best(lambda: trivial(x0)) * 1e6:7.1f} us"
    )
    print(
        f"   [dispatch only] trivial jit+dict : {best(lambda: trivial_p(x0, params)) * 1e6:7.1f} us"
    )

    # -- one rollout of N steps inside a scan
    def rollout_frozen(x0):
        def body(x, t):
            return ev.rk4_step_trace(x, u0, t, dt), x

        return jax.lax.scan(body, x0, jnp.arange(N) * dt)[1]

    def rollout_p(x0, p):
        def body(x, t):
            return ev.rk4_step_trace_p(x, u0, t, dt, p), x

        return jax.lax.scan(body, x0, jnp.arange(N) * dt)[1]

    r_frozen = jax.jit(rollout_frozen)
    r_p = jax.jit(rollout_p)
    r_bound = jax.jit(partial(rollout_p, p=params))
    print(f"-- one rollout, {N} RK4 steps in a scan")
    for name, fn in [
        ("frozen        ", lambda: r_frozen(x0)),
        ("params        ", lambda: r_p(x0, params)),
        ("params bound  ", lambda: r_bound(x0)),
    ]:
        print(f"   {name}: {best(fn, inner=20) * 1e3:7.2f} ms")

    # -- a batch of B rollouts (vmap), same params, and a params family (one m per rollout)
    x0s = jnp.tile(x0, (B, 1)) + 0.01 * jnp.arange(B)[:, None]
    rb_frozen = jax.jit(jax.vmap(rollout_frozen))
    rb_p = jax.jit(jax.vmap(rollout_p, in_axes=(0, None)))
    family = dict(params)
    first = keys[0]
    family[first] = params[first] * (1.0 + 0.001 * jnp.arange(B))
    rb_family = jax.jit(
        jax.vmap(rollout_p, in_axes=(0, {k: (0 if k == first else None) for k in keys}))
    )
    print(f"-- {B} rollouts x {N} steps, vmap")
    for name, fn in [
        ("frozen                  ", lambda: rb_frozen(x0s)),
        ("params (shared)         ", lambda: rb_p(x0s, params)),
        (f"params family ({first} varies)", lambda: rb_family(x0s, family)),
    ]:
        print(f"   {name}: {best(fn, inner=3) * 1e3:7.1f} ms")

    # -- a gradient of a rollout cost w.r.t. a plant parameter (only the params form can)
    def J(p):
        xs = rollout_p(x0, p)
        return jnp.sum(xs**2) * dt

    gJ = jax.jit(jax.grad(J))
    print("-- jax.grad of a rollout cost w.r.t. every param (params form only)")
    print(
        f"   grad                    : {best(lambda: gJ(params), inner=10) * 1e3:7.2f} ms"
    )
    print(
        f"   value                   : {best(lambda: jax.jit(J)(params), inner=10) * 1e3:7.2f} ms"
    )


for plant in (Pendulum(), CartPole()):
    bench(plant)
