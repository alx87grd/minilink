"""
Dynamic-programming value iteration on a discretized state space.

:class:`DynamicProgrammingPlanner` solves the Bellman equation backward over a
:class:`~minilink.planning.policy_synthesis.discretizer.StateSpaceGrid`,
producing a cost-to-go field ``J`` and a greedy policy ``pi`` (action ids). It
mirrors the trajectory-optimization planner: the planner owns the workflow, the
grid owns the discretization. One method covers both the infinite-horizon
value-iteration-to-tolerance and the finite-horizon fixed-step solves.

The cost is the textbook running/terminal pair ``g(x, u, t)`` / ``h(x, t)`` from
:class:`~minilink.core.costs.CostFunction`; out-of-domain transitions are
charged a finite ``out_of_bound_cost`` (the role of pyro's ``cf.INF``).

Three interchangeable backward-step engines share this workflow:

- ``"loop"`` — a per-node Python double loop (pyro's reference ``DynamicProgramming``),
  recomputing dynamics on the fly; low memory, slow.
- ``"numpy"`` — vectorized over a precomputed successor/cost lookup table (pyro's
  ``DynamicProgrammingWithLookUpTable``); the default.
- ``"jax"`` — the same vectorized backup mapped with ``vmap``/``map_coordinates``
  and run as a single jitted ``lax.while_loop`` on device; fastest at scale.
"""

from dataclasses import dataclass

import time

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, configure_jax
from minilink.planning.planner import Planner
from minilink.planning.policy_synthesis.discretizer import (
    PAIR_CHUNK_SIZE,
    StateSpaceGrid,
    build_jax_node_chunks,
    build_jax_sa_chunks,
    maybe_print_build_progress,
    print_build_complete,
    print_build_progress,
)
from minilink.planning.problems import PlanningProblem

#: Per-node Python reference engine (pyro's base ``DynamicProgramming``).
BACKEND_LOOP = "loop"

# Public API


@dataclass
class DynamicProgrammingOptions:
    """
    Workflow options for :class:`DynamicProgrammingPlanner`.

    Parameters
    ----------
    backend : {"numpy", "loop", "jax"}
        Backward-step engine. ``"numpy"`` (default) is the vectorized lookup
        table, ``"loop"`` the per-node Python reference, ``"jax"`` the jitted
        device version (linear/nearest interpolation only; traceable cost/sets).
    alpha : float
        Discount (exponential forgetting) factor; use ``alpha < 1`` for
        guaranteed contraction in infinite-horizon value iteration.
    tol : float
        Stopping tolerance on the largest cost-to-go change per sweep.
    max_iterations : int
        Maximum number of backward sweeps.
    interpolation : {"linear", "nearest", "cubic", "quintic"}
        Cost-to-go interpolation. ``"linear"`` (default) and ``"nearest"`` are
        robust; spline methods (``"cubic"``, the ``"spline"`` alias, ``"quintic"``)
        are smoother but can ring across the infeasibility penalty.
    out_of_bound_cost : float
        Finite penalty charged to inadmissible inputs or out-of-domain
        successors.
    final_time : float
        Terminal time ``tf``; sweeps step backward as ``t = tf - k dt``.
    record_history : bool
        Keep ``(t, J, pi)`` per sweep for animation.
    verbose : bool
        Print build progress (50k-item counts with elapsed time and ETA) for
        lookup-table precompute and backward Bellman sweeps. Grid mesh and
        ``x_next`` progress use :attr:`StateSpaceGrid.verbose`; ``G``, ``J0``,
        and sweeps use this flag. Progress lines update in place about every
        2% until each step completes. Set both to ``False`` for silent runs.
    """

    backend: str = BACKEND_NUMPY
    alpha: float = 1.0
    tol: float = 0.1
    max_iterations: int = 1000
    interpolation: str = "linear"
    out_of_bound_cost: float = 1.0e6
    final_time: float = 0.0
    record_history: bool = False
    verbose: bool = False


@dataclass
class DynamicProgrammingResult:
    """
    Cost-to-go field and greedy policy from value iteration.

    Parameters
    ----------
    grid : StateSpaceGrid
        Grid the result is defined on.
    J : np.ndarray
        Cost-to-go by node id, shape ``(nodes_n,)``.
    pi : np.ndarray
        Greedy policy as action ids by node id, shape ``(nodes_n,)``.
    iterations : int
        Number of backward sweeps performed.
    delta : float
        Largest cost-to-go change in the final sweep.
    history : list, optional
        ``(t, J, pi)`` tuples per sweep when recorded.
    """

    grid: StateSpaceGrid
    J: np.ndarray
    pi: np.ndarray
    iterations: int
    delta: float
    history: list | None = None

    def controller(self, *, interpolation: str = "linear"):
        """Return a :class:`LookupTableController` from the greedy policy."""
        from minilink.planning.policy_synthesis.lookup_policy import (
            LookupTableController,
        )

        return LookupTableController(self.grid, self.pi, interpolation=interpolation)

    def value_at(self, x) -> float:
        """Interpolate the cost-to-go at an arbitrary state ``x``."""
        return float(self.grid.interpolate(self.J, np.atleast_2d(x))[0])

    def save(self, path: str) -> None:
        """Save ``J`` and ``pi`` to ``path`` (single ``.npz``)."""
        np.savez(path, J=self.J, pi=self.pi)

    @classmethod
    def load(cls, path: str, grid: StateSpaceGrid) -> "DynamicProgrammingResult":
        """Load a result saved by :meth:`save`, bound to ``grid``."""
        data = np.load(path)
        return cls(
            grid=grid,
            J=data["J"],
            pi=data["pi"],
            iterations=0,
            delta=float("nan"),
        )


class DynamicProgrammingPlanner(Planner):
    """
    Value-iteration planner over a discretized state space.

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid
        Discretization of ``problem``'s state and input spaces.
    options : DynamicProgrammingOptions, optional
        Workflow options.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        grid: StateSpaceGrid,
        options: DynamicProgrammingOptions | None = None,
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.grid = grid
        self.options = DynamicProgrammingOptions() if options is None else options
        if self.options.backend not in (BACKEND_LOOP, BACKEND_NUMPY, BACKEND_JAX):
            raise ValueError(f"Unknown backend {self.options.backend!r}")
        self._G = None  # running-cost table, cached when the grid is precomputed
        self._jax_cache = {}  # compiled JAX runners keyed by (stop_on_tol, n)
        if self.options.backend == BACKEND_JAX:
            self.grid.ensure_jax_transition(self.options.final_time)

    def compute_solution(self) -> DynamicProgrammingResult:
        """Run value iteration until convergence (or ``max_iterations``)."""
        return self._solve(max_iterations=self.options.max_iterations, stop_on_tol=True)

    def solve_steps(self, n: int) -> DynamicProgrammingResult:
        """Run exactly ``n`` backward sweeps (finite-horizon / time-varying)."""
        return self._solve(max_iterations=int(n), stop_on_tol=False)

    def clean_infeasible_set(self, tol: float = 1.0) -> DynamicProgrammingResult:
        """
        Flag states whose cost-to-go has saturated at ``out_of_bound_cost``.

        Their value is pinned to the penalty and their policy to the action
        nearest the system's nominal input, mirroring pyro's cleanup pass.
        """
        result = self.require_result()
        default_action = self.grid.nearest_action(
            self.problem.sys.get_u_from_input_ports()
        )
        infeasible = result.J > (self.options.out_of_bound_cost - tol)
        result.J[infeasible] = self.options.out_of_bound_cost
        result.pi[infeasible] = default_action
        return result

    # Internal machinery

    def _solve(self, *, max_iterations, stop_on_tol):
        if self.options.backend == BACKEND_JAX:
            return self._solve_jax(max_iterations, stop_on_tol)

        grid = self.grid
        opt = self.options
        tf = opt.final_time
        step = self._loop_step if opt.backend == BACKEND_LOOP else self._vectorized_step

        if opt.verbose:
            self._print_solve_banner(max_iterations, stop_on_tol)

        J = self._terminal_cost(tf)
        pi = np.zeros(grid.nodes_n, dtype=int)

        history = [(tf, J.copy(), pi.copy())] if opt.record_history else None

        delta = np.inf
        k = 0
        start_time = time.time()
        sweep_progress = {"enabled": opt.verbose}
        while k < max_iterations and (not stop_on_tol or delta > opt.tol):
            k += 1
            t = tf - k * grid.dt
            J_next = J
            J, pi = step(J_next, t)
            delta = float(np.max(np.abs(J - J_next)))
            if opt.verbose:
                if stop_on_tol:
                    self._print_sweep_line(k, t, J, J_next, start_time)
                else:
                    maybe_print_build_progress(
                        k,
                        max_iterations,
                        start_time,
                        prefix="Computing backward DP",
                        unit="sweeps",
                        state=sweep_progress,
                    )
            if history is not None:
                history.append((t, J.copy(), pi.copy()))

        if opt.verbose:
            elapsed = time.time() - start_time
            if stop_on_tol:
                print()
                print("Bellman equation solved!")
            else:
                print_build_complete(
                    "Computing backward DP",
                    elapsed,
                    f"({k} sweeps, delta={delta:.3f})",
                )

        result = DynamicProgrammingResult(
            grid=grid, J=J, pi=pi, iterations=k, delta=delta, history=history
        )
        return self._store_result(result)

    def _vectorized_step(self, J, t):
        """Vectorized Bellman backup over the precomputed lookup table (NumPy)."""
        grid = self.grid
        n, N, A = grid.n, grid.nodes_n, grid.actions_n
        alpha = self.options.alpha

        x_next, action_ok, x_next_ok = grid.transition(t)
        G = self._running_cost(action_ok, x_next_ok, t)

        J_next = grid.interpolate(J, x_next.reshape(-1, n), self.options.interpolation)
        Q = G + alpha * J_next.reshape(N, A)

        return Q.min(axis=1), Q.argmin(axis=1)

    def _loop_step(self, J, t):
        """One Bellman backup as an explicit loop over nodes and actions.

        The readable reference engine (pyro's ``DynamicProgramming``): for every
        state node and every control action, take a forward-Euler step, look up
        the arrival cost-to-go, and keep the cheapest action. ``numpy`` and
        ``jax`` compute the same thing vectorized; this version reads like the
        textbook algorithm.
        """
        grid = self.grid
        sys = self.problem.sys
        cost = self.problem.cost
        X = self.problem.X
        U = self.problem.U
        params = self.problem.params
        dt = grid.dt
        alpha = self.options.alpha
        INF = self.options.out_of_bound_cost

        J_interpol = grid.build_interpolator(J, self.options.interpolation)
        J_new = np.zeros(grid.nodes_n)
        pi = np.zeros(grid.nodes_n, dtype=int)

        # For all state nodes
        for s in range(grid.nodes_n):
            x = grid.states[s]
            Q = np.zeros(grid.actions_n)

            # For all control actions
            for a in range(grid.actions_n):
                u = grid.inputs[a]

                # If action is in allowable set
                if U.contains(u, x, t, params.sets):
                    # Forward dynamics
                    x_next = sys.f(x, u, t, params.system) * dt + x

                    # if the next state is not out-of-bound
                    if X.contains(x_next, t, params.sets):
                        # Estimated (interpolation) cost to go of arrival x_next state
                        J_next = J_interpol(x_next)[0]

                        # Cost-to-go of a given action
                        Q[a] = cost.g(x, u, t, params.cost) * dt + alpha * J_next

                    else:
                        # Out of bound terminal cost
                        Q[a] = INF

                else:
                    # Invalid control input at this state
                    Q[a] = INF

            J_new[s] = Q.min()
            pi[s] = Q.argmin()

        return J_new, pi

    def _terminal_cost(self, t):
        if self.options.backend == BACKEND_JAX:
            return self._terminal_cost_jax(t)

        grid = self.grid
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        N = grid.nodes_n
        verbose = self.options.verbose
        start = time.time()

        if verbose:
            print(f"Computing h(x,t) terminal cost.. {N:,} nodes", flush=True)

        J0 = np.empty(N, dtype=float)
        progress = {"enabled": verbose}
        for s in range(N):
            J0[s] = float(h(grid.states[s], t, cost_params))
            maybe_print_build_progress(
                s + 1,
                N,
                start,
                prefix="Computing h(x,t) terminal cost",
                unit="nodes",
                state=progress,
            )

        if verbose:
            print_build_complete(
                "Computing h(x,t) terminal cost",
                time.time() - start,
                f"({N:,} nodes)",
            )

        return J0

    def _terminal_cost_jax(self, t):
        """Terminal cost-to-go at every grid node (JAX vmap over N nodes)."""
        jax = configure_jax(enable_x64=True)
        jnp = jax.numpy
        grid = self.grid
        N = grid.nodes_n
        states = jnp.asarray(grid.states)
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        verbose = self.options.verbose

        if verbose:
            print(f"Computing h(x,t) terminal cost.. {N:,} nodes", flush=True)

        def node(s):
            return h(states[s], t, cost_params)

        try:
            J0 = build_jax_node_chunks(
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

        return J0

    def _running_cost(self, action_ok, x_next_ok, t):
        if self.grid.precomputed and self._G is not None:
            return self._G

        if self.options.backend == BACKEND_JAX:
            return self._running_cost_jax(action_ok, x_next_ok, t)

        grid = self.grid
        g = self.problem.cost.g
        cost_params = self.problem.params.cost
        dt = grid.dt
        N, A = grid.nodes_n, grid.actions_n
        verbose = self.options.verbose
        total_pairs = N * A
        start = time.time()

        if verbose:
            print(
                f"Computing g(x,u,t) look-up table.. {total_pairs:,} pairs", flush=True
            )

        G = np.empty((N, A), dtype=float)
        pairs_done = 0
        progress = {"enabled": verbose}
        for a in range(A):
            u = grid.inputs[a]
            for s in range(N):
                G[s, a] = float(g(grid.states[s], u, t, cost_params)) * dt
                pairs_done += 1
                maybe_print_build_progress(
                    pairs_done,
                    total_pairs,
                    start,
                    prefix="Computing g(x,u,t) look-up table",
                    unit="pairs",
                    state=progress,
                )

        G[(~action_ok) | (~x_next_ok)] = self.options.out_of_bound_cost

        if verbose:
            print_build_complete(
                "Computing g(x,u,t) look-up table",
                time.time() - start,
                f"({total_pairs:,} pairs)",
            )

        if self.grid.precomputed:
            self._G = G
        return G

    def _running_cost_jax(self, action_ok, x_next_ok, t):
        """Running-cost lookup table G[s,a] = g(x,u,t)*dt (JAX vmap over N×A pairs)."""
        jax = configure_jax(enable_x64=True)
        jnp = jax.numpy
        grid = self.grid
        g = self.problem.cost.g
        cost_params = self.problem.params.cost
        dt = grid.dt
        N, A = grid.nodes_n, grid.actions_n
        states = jnp.asarray(grid.states)
        inputs = jnp.asarray(grid.inputs)
        penalty = float(self.options.out_of_bound_cost)
        verbose = self.options.verbose
        total_pairs = N * A

        if verbose:
            print(
                f"Computing g(x,u,t) look-up table.. {total_pairs:,} pairs",
                flush=True,
            )

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

        G[(~action_ok) | (~x_next_ok)] = penalty

        if self.grid.precomputed:
            self._G = G
        return G

    def _print_solve_banner(self, max_iterations, stop_on_tol):
        """Print a pyro-style solve header."""
        if stop_on_tol:
            print(
                f"\nComputing backward DP iterations until dJ<{self.options.tol:.2f}:"
            )
            print("---------------------------------------------------------")
        else:
            print(f"\nComputing {max_iterations} backward DP iterations:")
            print("-----------------------------------------")

    def _print_sweep_line(self, k, t, J, J_next, start_time):
        """Print one convergence line (pyro ``finalize_backward_step`` format)."""
        delta = J - J_next
        elapsed = time.time() - start_time
        print(
            f"{k:4d} t:{t:7.2f} Elapsed:{elapsed:7.2f} "
            f"max:{J.max():7.2f} dmax:{delta.max():7.2f} dmin:{delta.min():7.2f}"
        )

    def _solve_jax(self, max_iterations, stop_on_tol):
        """Jitted on-device value iteration over JAX-built lookup tables.

        Grid transition, terminal cost J0, and running cost G are built with JAX
        ``vmap`` when ``backend="jax"``. The convergence loop runs as a single
        jitted ``lax.while_loop`` with ``map_coordinates`` interpolation. The
        compiled runner is cached so repeated solves pay compilation only once.
        """
        jax = configure_jax(enable_x64=True)  # match NumPy float64 precision
        jnp = jax.numpy
        grid = self.grid
        opt = self.options
        tf = opt.final_time
        n = grid.n

        if opt.interpolation not in ("linear", "nearest"):
            raise ValueError("JAX backend supports 'linear' or 'nearest' interpolation")

        # JAX-built lookup tables, then move to device for the jitted sweep.
        x_next, action_ok, x_next_ok = grid.transition(tf)
        G = jnp.asarray(self._running_cost(action_ok, x_next_ok, tf))
        J0 = jnp.asarray(self._terminal_cost(tf))
        coords = (
            (jnp.asarray(x_next.reshape(-1, n)) - jnp.asarray(grid.x_lb))
            / jnp.asarray(grid.x_step)
        ).T  # fractional grid indices, shape (n, N*A)
        # Hard-zero out-of-grid arrivals to match RegularGridInterpolator(fill_value=0)
        # instead of map_coordinates' half-cell ramp to cval.
        shape = tuple(int(s) for s in grid.x_grid_shape)
        in_bounds = jnp.all(
            (coords >= 0.0) & (coords <= (jnp.asarray(shape)[:, None] - 1)), axis=0
        )

        if opt.record_history or opt.verbose:
            return self._solve_jax_iterative(
                jax, jnp, J0, G, coords, in_bounds, max_iterations, stop_on_tol
            )

        run = self._jax_runner(jax, jnp, stop_on_tol, max_iterations)
        J, pi, k, delta = run(J0, G, coords, in_bounds)
        jax.block_until_ready(J)
        result = DynamicProgrammingResult(
            grid=grid,
            J=np.array(J),  # writable copy (np.asarray of a jax buffer is read-only)
            pi=np.array(pi, dtype=int),
            iterations=int(k),
            delta=float(delta),
            history=None,
        )
        return self._store_result(result)

    def _jax_step(self, jax, jnp):
        """Return (and cache) the jitted single Bellman backup."""
        cached = self._jax_cache.get("step")
        if cached is not None:
            return cached

        from jax.scipy.ndimage import map_coordinates

        shape = tuple(int(s) for s in self.grid.x_grid_shape)
        N, A = self.grid.nodes_n, self.grid.actions_n
        alpha = float(self.options.alpha)
        order = 0 if self.options.interpolation == "nearest" else 1

        def step(J, G, coords, in_bounds):
            arrival = map_coordinates(
                J.reshape(shape), coords, order=order, mode="constant", cval=0.0
            )
            arrival = jnp.where(in_bounds, arrival, 0.0)
            Q = G + alpha * arrival.reshape(N, A)
            return Q.min(axis=1), Q.argmin(axis=1).astype(jnp.int32)

        compiled = jax.jit(step)
        self._jax_cache["step"] = compiled
        return compiled

    def _jax_runner(self, jax, jnp, stop_on_tol, max_iterations):
        """Return (and cache) the jitted whole value-iteration loop."""
        key = (bool(stop_on_tol), int(max_iterations))
        cached = self._jax_cache.get(key)
        if cached is not None:
            return cached

        step = self._jax_step(jax, jnp)
        N = self.grid.nodes_n
        tol = float(self.options.tol)
        zeros_pi = jnp.zeros(N, dtype=jnp.int32)

        if stop_on_tol:

            def run(J0, G, coords, in_bounds):
                def cond(carry):
                    _, _, k, delta = carry
                    return (delta > tol) & (k < max_iterations)

                def body(carry):
                    J, _, k, _ = carry
                    J_new, pi = step(J, G, coords, in_bounds)
                    return J_new, pi, k + 1, jnp.max(jnp.abs(J_new - J))

                init = (J0, zeros_pi, jnp.array(0), jnp.array(np.inf))
                return jax.lax.while_loop(cond, body, init)
        else:

            def run(J0, G, coords, in_bounds):
                def body(_, carry):
                    J, _, _ = carry
                    J_new, pi = step(J, G, coords, in_bounds)
                    return J_new, pi, jnp.max(jnp.abs(J_new - J))

                init = (J0, zeros_pi, jnp.array(np.inf))
                J, pi, delta = jax.lax.fori_loop(0, max_iterations, body, init)
                return J, pi, jnp.array(max_iterations), delta

        compiled = jax.jit(run)
        self._jax_cache[key] = compiled
        return compiled

    def _solve_jax_iterative(
        self, jax, jnp, J0, G, coords, in_bounds, max_iterations, stop_on_tol
    ):
        """JAX value iteration one sweep at a time (progress and/or history)."""
        grid = self.grid
        opt = self.options
        tf, tol = opt.final_time, float(opt.tol)
        step = self._jax_step(jax, jnp)

        if opt.verbose:
            self._print_solve_banner(max_iterations, stop_on_tol)

        J, pi = J0, jnp.zeros(grid.nodes_n, dtype=jnp.int32)
        delta, k = np.inf, 0
        start_time = time.time()
        sweep_progress = {"enabled": opt.verbose}
        history = [(tf, np.asarray(J), np.asarray(pi))] if opt.record_history else None

        while k < max_iterations and (not stop_on_tol or delta > tol):
            k += 1
            J_next = J
            J, pi = step(J, G, coords, in_bounds)
            jax.block_until_ready(J)
            delta = float(jnp.max(jnp.abs(J - J_next)))
            t = tf - k * grid.dt
            if opt.verbose:
                if stop_on_tol:
                    self._print_sweep_line(
                        k, t, np.asarray(J), np.asarray(J_next), start_time
                    )
                else:
                    maybe_print_build_progress(
                        k,
                        max_iterations,
                        start_time,
                        prefix="Computing backward DP",
                        unit="sweeps",
                        state=sweep_progress,
                    )
            if history is not None:
                history.append((t, np.asarray(J), np.asarray(pi)))

        if opt.verbose:
            elapsed = time.time() - start_time
            if stop_on_tol:
                print()
                print("Bellman equation solved!")
            else:
                print_build_complete(
                    "Computing backward DP",
                    elapsed,
                    f"({k} sweeps, delta={delta:.3f})",
                )

        result = DynamicProgrammingResult(
            grid=grid,
            J=np.array(J),  # writable copy
            pi=np.array(pi, dtype=int),
            iterations=k,
            delta=delta,
            history=history,
        )
        return self._store_result(result)
