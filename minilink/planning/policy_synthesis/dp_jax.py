"""The JAX engine of value iteration: the lookup-table backward step, jitted on device."""

import time

import numpy as np

from minilink.core.backends import ensure_jax_x64
from minilink.planning.policy_synthesis.discretizer import PAIR_CHUNK_SIZE
from minilink.planning.policy_synthesis.dp import DynamicProgrammingResult
from minilink.planning.policy_synthesis.progress import (
    SweepLog,
    maybe_print_build_progress,
    print_build_complete,
)

# Public API


def value_iteration(planner, max_sweeps, stop_on_tol=True) -> DynamicProgrammingResult:
    """
    The planner's value iteration, run on device.

    The same backward sweeps as
    :meth:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner.value_iteration`
    over the lookup tables, with the whole loop compiled as one ``lax`` loop.
    With ``verbose`` or ``record_history`` the sweeps run one at a time from
    Python instead, so each one can be reported or recorded.

    Warning: the device tables are built once, at ``final_time``, and reused
    for every sweep, so the dynamics ``f``, the running cost ``g`` and a callable
    ``out_of_bound_cost`` must not depend on time. Use the ``"numpy"`` backend on
    a grid with ``precompute=False`` for a time-varying problem.
    """
    jax = ensure_jax_x64()
    opt = planner.options

    # Device tables and the cost-to-go at the final time
    G, coords, in_bounds = device_tables(planner)
    J0 = jax.numpy.asarray(terminal_cost(planner, opt.final_time))

    if opt.verbose or opt.record_history:
        return sweep_by_sweep(
            planner, J0, G, coords, in_bounds, max_sweeps, stop_on_tol
        )

    # All the backward sweeps in one compiled call
    run = compiled_loop(planner, stop_on_tol, max_sweeps)
    J, pi, k, delta = run(J0, G, coords, in_bounds)
    jax.block_until_ready(J)

    return DynamicProgrammingResult(
        grid=planner.grid,
        J=np.array(J),
        pi=np.array(pi, dtype=int),
        iterations=int(k),
        delta=float(delta),
        history=None,
    )


def terminal_cost(planner, t) -> np.ndarray:
    """Terminal cost at every grid node (JAX vmap over N nodes)."""
    jax = ensure_jax_x64()
    jnp = jax.numpy
    grid = planner.grid
    h = planner.problem.cost.h
    cost_params = planner.problem.params.cost
    N = grid.nodes_n
    verbose = planner.options.verbose

    states = jnp.asarray(grid.states)

    if verbose:
        print(f"Computing h(x,t) terminal cost.. {N:,} nodes", flush=True)

    # Final cost of one node
    def node(s):
        return h(states[s], t, cost_params)

    try:
        J = build_jax_node_chunks(
            node,
            N,
            jax,
            jnp,
            interval=PAIR_CHUNK_SIZE,
            verbose=verbose,
            prefix="Computing h(x,t) terminal cost",
        )
    except Exception as exc:
        raise ValueError(
            "JAX terminal-cost build requires a JAX-traceable cost.h"
        ) from exc

    return J


def running_cost_table(planner, t) -> np.ndarray:
    """Running cost of every (node, action) pair over one step, with the out-of-bound penalty (JAX vmap)."""
    jax = ensure_jax_x64()
    jnp = jax.numpy
    grid = planner.grid
    g = planner.problem.cost.g
    cost_params = planner.problem.params.cost
    dt = grid.dt
    N, A = grid.nodes_n, grid.actions_n
    verbose = planner.options.verbose

    states = jnp.asarray(grid.states)
    inputs = jnp.asarray(grid.inputs)

    if verbose:
        print(f"Computing g(x,u,t) look-up table.. {N * A:,} pairs", flush=True)

    # Running cost of one pair over one step
    def pair(s, a):
        return g(states[s], inputs[a], t, cost_params) * dt

    try:
        G = build_jax_sa_chunks(
            pair,
            N,
            A,
            jax,
            jnp,
            interval=PAIR_CHUNK_SIZE,
            verbose=verbose,
            prefix="Computing g(x,u,t) look-up table",
        )
    except Exception as exc:
        raise ValueError(
            "JAX G-table build requires a JAX-traceable cost.g "
            "(QuadraticCost and TimeCost are supported)"
        ) from exc

    # Out of bound cost on the pairs that leave the admissible set: the scalar, or
    # the price of each pair's successor
    x_next, action_ok, x_next_ok = grid.transition(t)
    inadmissible = ~(action_ok & x_next_ok)
    price = planner.options.out_of_bound_cost
    if callable(price):
        exits = jnp.asarray(x_next[inadmissible])

        # Price of leaving at one exit state
        def exit_state(k):
            return price(exits[k], t)

        try:
            G[inadmissible] = build_jax_node_chunks(
                exit_state,
                len(exits),
                jax,
                jnp,
                interval=PAIR_CHUNK_SIZE,
                verbose=verbose,
                prefix="Computing out_of_bound_cost(x,t) of the exits",
            )
        except Exception as exc:
            raise ValueError(
                "JAX backend requires a JAX-traceable, scalar-valued "
                "out_of_bound_cost(x, t); use a scalar price or the 'numpy' backend"
            ) from exc
    else:
        G[inadmissible] = price

    return G


# Internal machinery


def device_tables(planner):
    """Device copies of the running-cost table and of the successors as fractional grid indices."""
    cached = planner._jax_cache.get("tables")
    if cached is not None:
        return cached

    jnp = ensure_jax_x64().numpy
    grid = planner.grid
    tf = planner.options.final_time
    n = grid.n

    if planner.options.interpolation not in ("linear", "nearest"):
        raise ValueError("JAX backend supports 'linear' or 'nearest' interpolation")

    # Successors of every pair, and the running-cost table on device
    x_next, _, _ = grid.transition(tf)
    G = jnp.asarray(running_cost_table(planner, tf))

    # Successors as fractional grid indices, shape (n, N*A)
    x_lb = jnp.asarray(grid.x_lb)
    x_step = jnp.asarray(grid.x_step)
    coords = ((jnp.asarray(x_next.reshape(-1, n)) - x_lb) / x_step).T

    # Arrivals off the grid are hard-zeroed, as RegularGridInterpolator(fill_value=0)
    # does, instead of map_coordinates' half-cell ramp to cval
    shape = jnp.asarray([int(s) for s in grid.x_grid_shape])
    in_bounds = jnp.all((coords >= 0.0) & (coords <= shape[:, None] - 1), axis=0)

    planner._jax_cache["tables"] = (G, coords, in_bounds)

    return G, coords, in_bounds


def jitted_step(planner):
    """Return (and cache) the jitted single Bellman backup."""
    cached = planner._jax_cache.get("step")
    if cached is not None:
        return cached

    jax = ensure_jax_x64()
    jnp = jax.numpy
    from jax.scipy.ndimage import map_coordinates

    shape = tuple(int(s) for s in planner.grid.x_grid_shape)
    N, A = planner.grid.nodes_n, planner.grid.actions_n
    alpha = float(planner.options.alpha)
    order = 0 if planner.options.interpolation == "nearest" else 1

    def step(J, G, coords, in_bounds):
        # Estimated cost-to-go of all the arrival states
        J_arrival = map_coordinates(
            J.reshape(shape), coords, order=order, mode="constant", cval=0.0
        )
        J_arrival = jnp.where(in_bounds, J_arrival, 0.0)

        # Matrix version of all the Q values, best action at every node
        Q = G + alpha * J_arrival.reshape(N, A)

        return Q.min(axis=1), Q.argmin(axis=1).astype(jnp.int32)

    planner._jax_cache["step"] = jax.jit(step)

    return planner._jax_cache["step"]


def compiled_loop(planner, stop_on_tol, max_sweeps):
    """Return (and cache) the jitted whole value-iteration loop."""
    key = (bool(stop_on_tol), int(max_sweeps))
    cached = planner._jax_cache.get(key)
    if cached is not None:
        return cached

    jax = ensure_jax_x64()
    jnp = jax.numpy
    step = jitted_step(planner)
    N = planner.grid.nodes_n
    tol = float(planner.options.tol)

    def run(J0, G, coords, in_bounds):
        def keep_going(carry):
            _, _, k, delta = carry
            return (k < max_sweeps) & ((delta > tol) | (not stop_on_tol))

        def sweep(carry):
            J_next, _, k, _ = carry
            J, pi = step(J_next, G, coords, in_bounds)
            delta = jnp.max(jnp.abs(J - J_next))
            return J, pi, k + 1, delta

        init = (J0, jnp.zeros(N, dtype=jnp.int32), jnp.array(0), jnp.array(np.inf))

        return jax.lax.while_loop(keep_going, sweep, init)

    planner._jax_cache[key] = jax.jit(run)

    return planner._jax_cache[key]


def sweep_by_sweep(planner, J0, G, coords, in_bounds, max_sweeps, stop_on_tol):
    """The backward sweeps one jitted step at a time, so each can be reported or recorded."""
    jax = ensure_jax_x64()
    jnp = jax.numpy
    opt = planner.options
    tf = opt.final_time
    dt = planner.grid.dt
    tol = float(opt.tol)
    step = jitted_step(planner)

    log = SweepLog(opt, max_sweeps, stop_on_tol)

    J = J0
    pi = jnp.zeros(planner.grid.nodes_n, dtype=jnp.int32)
    log.start(tf, J, pi)

    k = 0
    delta = np.inf

    while k < max_sweeps and (not stop_on_tol or delta > tol):
        k = k + 1
        t = tf - k * dt

        J_next = J
        J, pi = step(J_next, G, coords, in_bounds)
        jax.block_until_ready(J)

        delta = float(jnp.max(jnp.abs(J - J_next)))

        log.sweep(k, t, J, J_next, pi)

    log.done(k, delta)

    return DynamicProgrammingResult(
        grid=planner.grid,
        J=np.array(J),
        pi=np.array(pi, dtype=int),
        iterations=k,
        delta=delta,
        history=log.history,
    )


def build_jax_sa_chunks(pair_fn, N, A, jax, jnp, *, interval, verbose, prefix):
    """Fill an ``(N, A)`` table by JAX ``vmap`` over flat state–action indices."""
    total = N * A
    out = np.empty((N, A), dtype=float)

    @jax.jit
    def eval_chunk(sa):
        s = sa // A
        a = sa % A
        return jax.vmap(pair_fn)(s, a)

    build_start = time.time()
    progress = {"enabled": verbose}
    for begin in range(0, total, interval):
        end = min(begin + interval, total)
        count = end - begin
        if count == interval:
            sa = jnp.arange(begin, end)
            chunk = eval_chunk(sa)
        else:
            sa = jnp.arange(begin, end)
            s = sa // A
            a = sa % A
            chunk = jax.vmap(pair_fn)(s, a)

        chunk = np.asarray(chunk)
        idx = np.arange(begin, end)
        out[idx // A, idx % A] = chunk
        maybe_print_build_progress(
            end, total, build_start, prefix=prefix, unit="pairs", state=progress
        )

    if verbose:
        print_build_complete(prefix, time.time() - build_start, f"({total:,} pairs)")

    return out


def build_jax_node_chunks(node_fn, N, jax, jnp, *, interval, verbose, prefix):
    """Fill a length-``N`` vector by JAX ``vmap`` over node indices."""
    out = np.empty(N, dtype=float)

    @jax.jit
    def eval_chunk(idx):
        return jax.vmap(node_fn)(idx)

    build_start = time.time()
    progress = {"enabled": verbose}
    for begin in range(0, N, interval):
        end = min(begin + interval, N)
        count = end - begin
        if count == interval:
            idx = jnp.arange(begin, end)
            chunk = eval_chunk(idx)
        else:
            idx = jnp.arange(begin, end)
            chunk = jax.vmap(node_fn)(idx)

        out[begin:end] = np.asarray(chunk)
        maybe_print_build_progress(
            end, N, build_start, prefix=prefix, unit="nodes", state=progress
        )

    if verbose:
        print_build_complete(prefix, time.time() - build_start, f"({N:,} nodes)")

    return out
