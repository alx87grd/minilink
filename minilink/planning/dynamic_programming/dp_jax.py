"""The JAX engine of value iteration: the lookup-table backward step, jitted on device."""

import time

import numpy as np

from minilink.core.backends import ensure_jax_x64
from minilink.planning.dynamic_programming.discretizer import PAIR_CHUNK_SIZE
from minilink.planning.dynamic_programming.dp import (
    DynamicProgrammingResult,
    stage_schedule,
)
from minilink.planning.dynamic_programming.progress import (
    maybe_print_build_progress,
    print_build_complete,
)

###############################################################################
# Public API
###############################################################################


def value_iteration(
    planner, max_sweeps, stop_on_tol=True, keep_stages=False
) -> DynamicProgrammingResult:
    """
    The planner's value iteration, run on device.

    The same backward sweeps as
    :meth:`~minilink.planning.dynamic_programming.dp.DynamicProgrammingPlanner.value_iteration`
    over the lookup tables, with the whole loop compiled as one ``lax`` loop
    (``keep_stages``: one ``lax.scan`` over the ``max_sweeps`` sweeps, every
    stage stacked). A reported or recorded solve runs the planner's own loop
    instead, one jitted step at a time (:func:`device_step`).

    Warning: a solve builds the device tables once, at ``final_time``, and reuses
    them for every sweep, so the dynamics ``f``, the running cost ``g`` and a
    callable ``out_of_bound_cost`` must not depend on time. Use the ``"numpy"`` backend on
    a grid with ``precompute=False`` for a time-varying problem.
    """
    jax = ensure_jax_x64()

    # Device tables and the cost-to-go at the final time, built for this solve
    G, coords, in_bounds = device_tables(planner)
    J0 = jax.numpy.asarray(terminal_cost(planner, planner.final_time))

    # A finite horizon: its N sweeps in one compiled scan, every stage kept
    if keep_stages:
        return finite_sweeps(planner, J0, G, coords, in_bounds, max_sweeps)

    # All the backward sweeps in one compiled call, the numbers its arguments
    gamma = float(planner.gamma)
    tol = float(planner.tol)
    run = compiled_loop(planner, stop_on_tol, max_sweeps)
    J, pi, k, delta = run(J0, G, coords, in_bounds, gamma, tol)
    jax.block_until_ready(J)

    return DynamicProgrammingResult(
        grid=planner.grid,
        J=np.array(J),
        pi=np.array(pi, dtype=int),
        iterations=int(k),
        delta=float(delta),
        history=None,
    )


def device_step(planner, verbose=False):
    """
    ``(step, J, pi)``: the jitted backup bound to this solve's device tables, ``J, pi = step(J_next)``, and the sweeps' start.

    The device tables and the cost-to-go at the final time are built here
    (``verbose``: their builds' progress), the policy starting at action 0;
    the planner's own loop then drives the step one sweep at a time.
    """
    jax = ensure_jax_x64()
    jnp = jax.numpy

    # Device tables and the cost-to-go at the final time, built for this solve
    G, coords, in_bounds = device_tables(planner, verbose)
    J0 = jnp.asarray(terminal_cost(planner, planner.final_time, verbose))
    pi0 = jnp.zeros(planner.grid.nodes_n, dtype=jnp.int32)

    gamma = float(planner.gamma)
    step = jitted_step(planner)

    # One Bellman backup on device from the cost-to-go one step ahead
    def bound_step(J_next):
        return step(J_next, G, coords, in_bounds, gamma)

    return bound_step, J0, pi0


def terminal_cost(planner, t, verbose=False) -> np.ndarray:
    """Terminal cost at every grid node (JAX vmap over N nodes; ``verbose``: the build's progress)."""
    jax = ensure_jax_x64()
    jnp = jax.numpy

    grid = planner.grid
    model = grid.model
    N = grid.nodes_n

    states = jnp.asarray(grid.states)

    if verbose:
        print(f"Computing h(x,t) terminal cost.. {N:,} nodes", flush=True)

    # Final cost of one node
    def node(s):
        return model.terminal(states[s], t)

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


def running_cost_table(planner, t, verbose=False) -> np.ndarray:
    """Running cost of every (node, action) pair over one step, with the out-of-bound penalty (JAX vmap; ``verbose``: the build's progress)."""
    jax = ensure_jax_x64()
    jnp = jax.numpy

    grid = planner.grid
    model = grid.model
    params = model.params
    N = grid.nodes_n
    A = grid.actions_n
    failure_cost = planner.out_of_bound_cost

    states = jnp.asarray(grid.states)
    inputs = jnp.asarray(grid.inputs)

    if verbose:
        print(f"Computing g(x,u,t) look-up table.. {N * A:,} pairs", flush=True)

    # The stage cost of one pair over one step: the model's c (g dt under Euler)
    def pair(s, a):
        return model.stage_cost_at(states[s], inputs[a], t, params)

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
    # the cost of each pair's successor
    x_next, action_ok, x_next_ok = grid.transition(t)
    inadmissible = ~(action_ok & x_next_ok)
    if callable(failure_cost):
        exits = jnp.asarray(x_next[inadmissible])

        # Price of leaving at one exit state
        def exit_state(k):
            return failure_cost(exits[k], t)

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
                "out_of_bound_cost(x, t); use a scalar cost or the 'numpy' backend"
            ) from exc
    else:
        G[inadmissible] = failure_cost

    return G


###############################################################################
# Internal machinery
###############################################################################


def device_tables(planner, verbose=False):
    """Device copies of the running-cost table and of the successors as fractional grid indices, at the final time."""
    jnp = ensure_jax_x64().numpy

    grid = planner.grid
    tf = planner.final_time
    interpolation = planner.interpolation
    n = grid.n

    require_device_interpolation(interpolation)

    # Successors of every pair, and the running-cost table on device
    x_next, _, _ = grid.transition(tf)
    G = jnp.asarray(running_cost_table(planner, tf, verbose))

    # Successors as fractional grid indices, (x_next − x_lb) / Δx, shape (n, N*A)
    x_lb = jnp.asarray(grid.x_lb)
    x_step = jnp.asarray(grid.x_step)
    coords = ((jnp.asarray(x_next.reshape(-1, n)) - x_lb) / x_step).T

    # Arrivals off the grid are hard-zeroed, as RegularGridInterpolator(fill_value=0)
    # does, instead of map_coordinates' half-cell ramp to cval
    shape = jnp.asarray([int(s) for s in grid.x_grid_shape])
    in_bounds = jnp.all((coords >= 0.0) & (coords <= shape[:, None] - 1), axis=0)

    return G, coords, in_bounds


def require_device_interpolation(interpolation):
    """Refuse an interpolation the device step cannot run (spline methods)."""
    if interpolation not in ("linear", "nearest"):
        raise ValueError("JAX backend supports 'linear' or 'nearest' interpolation")


def finite_sweeps(planner, J0, G, coords, in_bounds, N) -> DynamicProgrammingResult:
    """A finite horizon's ``N`` backward sweeps in one compiled scan, the stages stacked forward in time."""
    jax = ensure_jax_x64()

    tf = planner.final_time
    dt = planner.grid.dt
    gamma = float(planner.gamma)

    # J_{N−1} … J_0 and π_{N−1} … π_0, the sweeps' order
    run = compiled_scan(planner, N)
    J_swept, pi_swept = run(J0, G, coords, in_bounds, gamma)
    jax.block_until_ready(J_swept)
    J_swept = np.array(J_swept)
    pi_swept = np.array(pi_swept, dtype=int)

    # The last sweep's largest change of the cost-to-go
    J_before = np.array(J0) if N == 1 else J_swept[-2]
    delta = float(np.max(np.abs(J_swept[-1] - J_before)))

    # The stages forward in time: J_0 … J_N (J_N = h) and π_0 … π_{N−1}
    schedule = stage_schedule(tf, dt, [np.array(J0), *J_swept], list(pi_swept))

    return DynamicProgrammingResult(
        grid=planner.grid,
        J=np.array(J_swept[-1]),
        pi=np.array(pi_swept[-1], dtype=int),
        iterations=N,
        delta=delta,
        history=None,
        schedule=schedule,
    )


def compiled_scan(planner, N):
    """The jitted scan of a finite horizon's ``N`` sweeps, compiled once per ``N``; the discount is its argument."""
    key = ("finite", int(N))
    compiled = planner.compiled.get(key)
    if compiled is not None:
        return compiled

    jax = ensure_jax_x64()
    step = jitted_step(planner)

    def run(J0, G, coords, in_bounds, gamma):
        # One sweep: the Bellman backup from the cost-to-go one step ahead, its
        # tables kept
        def sweep(J_next, _):
            J, pi = step(J_next, G, coords, in_bounds, gamma)
            return J, (J, pi)

        _, (J_swept, pi_swept) = jax.lax.scan(sweep, J0, None, length=N)

        return J_swept, pi_swept

    planner.compiled[key] = jax.jit(run)

    return planner.compiled[key]


def compiled_loop(planner, stop_on_tol, max_sweeps):
    """The jitted whole value-iteration loop, compiled once per loop shape ``(stop_on_tol, max_sweeps)``; the discount and the tolerance are its arguments."""
    key = (bool(stop_on_tol), int(max_sweeps))
    compiled = planner.compiled.get(key)
    if compiled is not None:
        return compiled

    jax = ensure_jax_x64()
    jnp = jax.numpy

    N = planner.grid.nodes_n
    step = jitted_step(planner)

    def run(J0, G, coords, in_bounds, gamma, tol):
        # At most max_sweeps sweeps, and with stop_on_tol only until delta ≤ tol
        def keep_going(carry):
            _, _, k, delta = carry
            return (k < max_sweeps) & ((delta > tol) | (not stop_on_tol))

        def sweep(carry):
            J_next, _, k, _ = carry

            # One step back in time
            k = k + 1

            # Bellman backup from the cost-to-go one step ahead
            J, pi = step(J_next, G, coords, in_bounds, gamma)

            # Largest change of the cost-to-go: the convergence measure
            delta = jnp.max(jnp.abs(J - J_next))

            return J, pi, k, delta

        # The cost-to-go at the final time, no sweep yet, an infinite change
        init = (J0, jnp.zeros(N, dtype=jnp.int32), jnp.array(0), jnp.array(np.inf))

        return jax.lax.while_loop(keep_going, sweep, init)

    planner.compiled[key] = jax.jit(run)

    return planner.compiled[key]


def jitted_step(planner):
    """The jitted single Bellman backup, compiled once per planner; the discount is its argument."""
    compiled = planner.compiled.get("step")
    if compiled is not None:
        return compiled

    jax = ensure_jax_x64()
    jnp = jax.numpy
    map_coordinates = jax.scipy.ndimage.map_coordinates

    shape = tuple(int(s) for s in planner.grid.x_grid_shape)
    N = planner.grid.nodes_n
    A = planner.grid.actions_n
    order = 0 if planner.interpolation == "nearest" else 1

    def step(J_next, G, coords, in_bounds, gamma):
        # The cost-to-go one step ahead at every pair's successor (zero off the grid)
        J_arrival = map_coordinates(
            J_next.reshape(shape), coords, order=order, mode="constant", cval=0.0
        )
        J_arrival = jnp.where(in_bounds, J_arrival, 0.0).reshape(N, A)

        # Q(x, u) = c(x, u) + γ J_next(x_next)
        Q = G + gamma * J_arrival

        # J(x) = min_u Q(x, u), π(x) = argmin_u Q(x, u)
        J = Q.min(axis=1)
        pi = Q.argmin(axis=1)

        return J, pi.astype(jnp.int32)

    planner.compiled["step"] = jax.jit(step)

    return planner.compiled["step"]


def build_jax_sa_chunks(pair_fn, N, A, jax, jnp, *, interval, verbose, prefix):
    """Fill an ``(N, A)`` table by JAX ``vmap`` over flat state–action indices."""
    total = N * A
    out = np.empty((N, A), dtype=float)

    # A full chunk, compiled once for its shape
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

        # The full chunks run compiled; the shorter last one runs eagerly
        sa = jnp.arange(begin, end)
        if count == interval:
            chunk = eval_chunk(sa)
        else:
            s = sa // A
            a = sa % A
            chunk = jax.vmap(pair_fn)(s, a)

        # Flat index sa = s A + a, back to its (s, a) cell
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

    # A full chunk, compiled once for its shape
    @jax.jit
    def eval_chunk(idx):
        return jax.vmap(node_fn)(idx)

    build_start = time.time()
    progress = {"enabled": verbose}
    for begin in range(0, N, interval):
        end = min(begin + interval, N)
        count = end - begin

        # The full chunks run compiled; the shorter last one runs eagerly
        idx = jnp.arange(begin, end)
        if count == interval:
            chunk = eval_chunk(idx)
        else:
            chunk = jax.vmap(node_fn)(idx)

        out[begin:end] = np.asarray(chunk)
        maybe_print_build_progress(
            end, N, build_start, prefix=prefix, unit="nodes", state=progress
        )

    if verbose:
        print_build_complete(prefix, time.time() - build_start, f"({N:,} nodes)")

    return out
