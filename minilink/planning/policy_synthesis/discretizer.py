"""
State-space discretization for dynamic-programming policy synthesis.

A :class:`StateSpaceGrid` turns the continuous plant of a
:class:`~minilink.planning.problems.PlanningProblem` into the finite objects a
value-iteration solver needs: a regular grid of states and admissible inputs,
a forward-Euler successor map ``x_next = x + f(x, u, t) dt``, boolean validity
masks sourced from the problem sets (``X.contains`` / ``U.contains``), and an
interpolation of any node-indexed scalar field back onto the state space.

The grid is dimension-generic (any state size ``n`` and input size ``m``) and
cost-agnostic — the running and terminal costs live with the solver, not here.
This module is library-developer facing; the value-iteration math reads from
:mod:`minilink.planning.policy_synthesis.dp`.
"""

import time

import numpy as np
from scipy.interpolate import RegularGridInterpolator

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY
from minilink.core.sets import BoxInputSet, BoxSet
from minilink.planning.problems import PlanningProblem

PAIR_PROGRESS_INTERVAL = 50_000


def print_build_progress(done, total, start, *, prefix, unit="pairs"):
    """In-place terminal progress line with elapsed time and ETA."""
    elapsed = time.time() - start
    eta = elapsed / done * (total - done) if done < total else 0.0
    print(
        f"\r{prefix}.. {done:,}/{total:,} {unit} ({100.0 * done / total:.1f}%)"
        f"  {elapsed:.1f}s elapsed  ETA ~{eta:.0f}s",
        end="",
        flush=True,
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
        if verbose:
            print_build_progress(end, total, build_start, prefix=prefix)

    if verbose:
        elapsed = time.time() - build_start
        print()
        print(f"{prefix}.. completed in {elapsed:4.2f} sec  ({total:,} pairs)")

    return out


def build_jax_node_chunks(node_fn, N, jax, jnp, *, interval, verbose, prefix):
    """Fill a length-``N`` vector by JAX ``vmap`` over node indices."""
    out = np.empty(N, dtype=float)

    @jax.jit
    def eval_chunk(idx):
        return jax.vmap(node_fn)(idx)

    build_start = time.time()
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
        if verbose:
            print_build_progress(
                end, N, build_start, prefix=prefix, unit="nodes"
            )

    if verbose:
        elapsed = time.time() - build_start
        print()
        print(f"{prefix}.. completed in {elapsed:4.2f} sec  ({N:,} nodes)")

    return out

# Public API


class StateSpaceGrid:
    """
    Regular grid over the state and input spaces of a planning problem.

    Parameters
    ----------
    problem : PlanningProblem
        Source of the system, the state set ``X`` (grid extent and state
        validity), and the input set ``U`` (grid extent and input validity).
    x_grid_shape : sequence of int
        Number of levels per state axis, length ``n``.
    u_grid_shape : sequence of int
        Number of levels per input axis, length ``m``.
    dt : float
        Forward-Euler time step used by the successor map.
    precompute : bool, optional
        When ``True`` (default) the successor and validity tables are built once
        at construction (``t = 0``) — fast sweeps, but only valid for
        time-invariant dynamics. When ``False`` they are recomputed per call to
        :meth:`transition`, which costs less memory and supports time-varying
        dynamics.
    precompute_backend : {"numpy", "jax"}, optional
        Engine for building the successor lookup table. ``"numpy"`` (default) uses
        nested Python loops (pyro's reference). ``"jax"`` vectorizes the forward
        dynamics with ``vmap`` and is much faster on large grids when the plant
        is JAX-traceable and the state/input sets are boxes.
    verbose : bool, optional
        Print mesh size and lookup-table build progress (pyro-style feedback for
        large grids): counts every 50k items, elapsed time, and ETA.

    Attributes
    ----------
    states : np.ndarray
        Grid states, shape ``(nodes_n, n)``.
    inputs : np.ndarray
        Grid inputs, shape ``(actions_n, m)``.
    x_next, action_ok, x_next_ok : np.ndarray
        Successor and validity tables (present only when ``precompute`` is set).
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        x_grid_shape,
        u_grid_shape,
        dt: float,
        precompute: bool = True,
        precompute_backend: str = BACKEND_NUMPY,
        verbose: bool = False,
    ) -> None:
        self.problem = problem
        self.sys = problem.sys
        self.n = int(self.sys.n)
        self.m = int(self.sys.m)
        self.dt = float(dt)

        self.x_grid_shape = self._coerce_shape(x_grid_shape, self.n, "x_grid_shape")
        self.u_grid_shape = self._coerce_shape(u_grid_shape, self.m, "u_grid_shape")

        x_lb, x_ub = self._state_bounds(problem)
        u_lb, u_ub = self._input_bounds(problem)
        self.x_lb, self.x_ub = x_lb, x_ub
        self.u_lb, self.u_ub = u_lb, u_ub

        self.x_levels = [
            np.linspace(x_lb[i], x_ub[i], self.x_grid_shape[i]) for i in range(self.n)
        ]
        self.u_levels = [
            np.linspace(u_lb[i], u_ub[i], self.u_grid_shape[i]) for i in range(self.m)
        ]
        self.x_step = self._step_sizes(x_lb, x_ub, self.x_grid_shape)
        self.u_step = self._step_sizes(u_lb, u_ub, self.u_grid_shape)

        self.states = self._grid_points(self.x_levels)
        self.inputs = self._grid_points(self.u_levels)
        self.nodes_n = self.states.shape[0]
        self.actions_n = self.inputs.shape[0]

        self.precompute_backend = precompute_backend
        self.verbose = bool(verbose)
        self._jax_transition = False
        if precompute_backend not in (BACKEND_NUMPY, BACKEND_JAX):
            raise ValueError(f"Unknown precompute_backend {precompute_backend!r}")

        self.precomputed = bool(precompute)
        if self.precomputed:
            if self.verbose:
                self._print_mesh_summary()
            self.x_next, self.action_ok, self.x_next_ok = self._build_transition(0.0)
            if precompute_backend == BACKEND_JAX:
                self._jax_transition = True

    def ensure_jax_transition(self, t: float = 0.0) -> None:
        """Build or rebuild transition tables with the JAX backend.

        Called by :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
        when ``backend="jax"``. Use ``precompute=False`` on the grid in JAX demos
        so the slow NumPy build is skipped at construction time.
        """
        if self._jax_transition and self.precomputed:
            return

        if self.verbose and not self.precomputed:
            self._print_mesh_summary()

        try:
            self.x_next, self.action_ok, self.x_next_ok = self._compute_transition_jax(t)
        except Exception as exc:
            raise ValueError(
                "JAX grid precompute requires a JAX-traceable plant f and BoxSet X"
            ) from exc

        self.precomputed = True
        self._jax_transition = True

    # Dynamics and validity

    def successors(self, t: float = 0.0) -> np.ndarray:
        """Return the successor states ``x_next`` of shape ``(nodes_n, actions_n, n)``."""
        if self.precomputed:
            return self.x_next
        return self._build_transition(t)[0]

    def transition(self, t: float = 0.0):
        """
        Return ``(x_next, action_ok, x_next_ok)`` at time ``t``.

        Cached tables are returned when ``precompute`` is set; otherwise the
        successor map and validity masks are recomputed for time ``t``.
        """
        if self.precomputed:
            return self.x_next, self.action_ok, self.x_next_ok
        return self._build_transition(t)

    def _build_transition(self, t):
        """Build successor and validity tables with the selected precompute backend."""
        if self.precompute_backend == BACKEND_JAX:
            return self._compute_transition_jax(t)
        return self._compute_transition_numpy(t)

    def _compute_transition_numpy(self, t):
        """Build the forward-Euler successors and the input/state validity masks."""
        N, A, n = self.nodes_n, self.actions_n, self.n
        f = self.sys.f
        X, U = self.problem.X, self.problem.U
        sys_params = self.problem.params.system
        set_params = self.problem.params.sets
        dt = self.dt
        verbose = self.verbose

        if verbose:
            start = time.time()
            total_pairs = N * A
            print("Computing x_next array.. ", end="", flush=True)

        x_next = np.empty((N, A, n), dtype=float)
        action_ok = np.empty((N, A), dtype=bool)
        x_next_ok = np.empty((N, A), dtype=bool)

        pairs_done = 0
        for a in range(A):
            u = self.inputs[a]
            for s in range(N):
                x = self.states[s]

                # forward Euler step of the continuous dynamics
                xnext = x + f(x, u, t, sys_params) * dt

                x_next[s, a] = xnext
                action_ok[s, a] = U.contains(u, x, t, set_params)
                x_next_ok[s, a] = X.contains(xnext, t, set_params)

                pairs_done += 1
                if verbose and pairs_done % PAIR_PROGRESS_INTERVAL == 0:
                    print_build_progress(
                        pairs_done, total_pairs, start, prefix="Computing x_next array"
                    )

        if verbose:
            elapsed = time.time() - start
            print()
            print(
                f"Computing x_next array.. completed in {elapsed:4.2f} sec"
                f"  ({total_pairs:,} pairs)"
            )

        return x_next, action_ok, x_next_ok

    def _compute_transition_jax(self, t):
        """Vectorized successor map and box validity masks on device."""
        from minilink.core.backends import configure_jax

        if not isinstance(self.problem.X, BoxSet):
            raise ValueError("JAX precompute requires a BoxSet state constraint X")

        jax = configure_jax(enable_x64=True)
        jnp = jax.numpy

        N, A, n = self.nodes_n, self.actions_n, self.n
        dt = self.dt
        sys_params = self.problem.params.system
        states = jnp.asarray(self.states)
        inputs = jnp.asarray(self.inputs)
        x_lb = jnp.asarray(self.problem.X.lower)
        x_ub = jnp.asarray(self.problem.X.upper)
        box_inputs = isinstance(self.problem.U, BoxInputSet)

        if self.verbose:
            print(
                f"Computing x_next array (JAX vmap).. {N * A:,} pairs",
                flush=True,
            )

        def pair(s, a):
            x = states[s]
            u = inputs[a]

            # forward Euler step of the continuous dynamics
            xnext = x + self.sys.f(x, u, t, sys_params) * dt
            x_next_ok = jnp.all((xnext >= x_lb) & (xnext <= x_ub))
            return xnext, x_next_ok

        x_next, x_next_ok = self._build_xnext_jax_chunks(
            pair, N, A, n, jax, jnp, interval=PAIR_PROGRESS_INTERVAL
        )
        if box_inputs:
            action_ok = np.ones((N, A), dtype=bool)
        else:
            action_ok, x_next_ok = self._validity_masks(x_next, t)

        return x_next, action_ok, x_next_ok

    def _build_xnext_jax_chunks(self, pair, N, A, n, jax, jnp, *, interval):
        """Fill ``x_next`` in fixed-size JAX chunks with optional progress lines."""
        total_pairs = N * A
        x_next = np.empty((N, A, n), dtype=float)
        x_next_ok = np.empty((N, A), dtype=bool)
        verbose = self.verbose

        @jax.jit
        def eval_chunk(sa):
            s = sa // A
            a = sa % A
            xnext, ok = jax.vmap(pair)(s, a)
            return xnext, ok

        build_start = time.time()
        for begin in range(0, total_pairs, interval):
            end = min(begin + interval, total_pairs)
            count = end - begin
            if count == interval:
                sa = jnp.arange(begin, end)
                xnext_chunk, ok_chunk = eval_chunk(sa)
            else:
                sa = jnp.arange(begin, end)
                s = sa // A
                a = sa % A
                xnext_chunk, ok_chunk = jax.vmap(pair)(s, a)

            xnext_chunk = np.asarray(xnext_chunk)
            ok_chunk = np.asarray(ok_chunk)
            idx = np.arange(begin, end)
            s_idx = idx // A
            a_idx = idx % A
            x_next[s_idx, a_idx] = xnext_chunk
            x_next_ok[s_idx, a_idx] = ok_chunk
            if verbose:
                print_build_progress(
                    end,
                    total_pairs,
                    build_start,
                    prefix="Computing x_next array (JAX vmap)",
                )

        if verbose:
            elapsed = time.time() - build_start
            print()
            print(
                f"Computing x_next array (JAX vmap).. completed in {elapsed:4.2f} sec"
                f"  ({total_pairs:,} pairs)"
            )

        return x_next, x_next_ok

    def _validity_masks(self, x_next, t):
        """Input and successor validity masks for a precomputed ``x_next`` table."""
        N, A = self.nodes_n, self.actions_n
        X, U = self.problem.X, self.problem.U
        set_params = self.problem.params.sets

        if isinstance(U, BoxInputSet) and isinstance(X, BoxSet):
            action_ok = np.ones((N, A), dtype=bool)
            x_next_ok = np.all(x_next >= X.lower, axis=2) & np.all(x_next <= X.upper, axis=2)
            return action_ok, x_next_ok

        action_ok = np.empty((N, A), dtype=bool)
        x_next_ok = np.empty((N, A), dtype=bool)
        for a in range(A):
            u = self.inputs[a]
            for s in range(N):
                x = self.states[s]
                action_ok[s, a] = U.contains(u, x, t, set_params)
                x_next_ok[s, a] = X.contains(x_next[s, a], t, set_params)
        return action_ok, x_next_ok

    def _print_mesh_summary(self):
        """Print grid size summary (mirrors pyro ``GridDynamicSystem.compute``)."""
        name = getattr(self.sys, "name", "system")
        print(f"\nGenerating mesh for: {name}")
        print("---------------------------------------------------")
        print(f"State space dimensions: {self.n}  Input space dimension: {self.m}")
        print(
            f"Number of nodes: {self.nodes_n}  Number of actions: {self.actions_n}"
        )
        print(f"Number of node-action pairs: {self.nodes_n * self.actions_n}")
        print("---------------------------------------------------")

    # Interpolation and reshaping

    def build_interpolator(self, values, method: str = "linear"):
        """
        Return a regular-grid interpolator callable for a node-indexed field.

        ``method`` is one of ``{"linear", "nearest", "cubic", "quintic"}``;
        ``"spline"`` is accepted as an alias for ``"cubic"``. Out-of-bounds
        queries return 0. Spline methods are smoother but ring across the
        infeasibility penalty, so they suit fields without large cliffs.
        """
        method = "cubic" if method == "spline" else method
        grid = values.reshape(self.x_grid_shape)
        return RegularGridInterpolator(
            tuple(self.x_levels),
            grid,
            method=method,
            bounds_error=False,
            fill_value=0.0,
        )

    def interpolate(self, values, queries, method: str = "linear") -> np.ndarray:
        """Interpolate a node-indexed field ``values`` at states ``queries`` (M, n)."""
        interp = self.build_interpolator(values, method)
        return interp(np.asarray(queries, dtype=float))

    def grid_from_array(self, values) -> np.ndarray:
        """Reshape a node-indexed array ``(nodes_n,)`` into the state grid shape."""
        return np.asarray(values).reshape(self.x_grid_shape)

    def input_from_policy(self, pi) -> np.ndarray:
        """Map a policy of action ids ``(nodes_n,)`` to inputs ``(nodes_n, m)``."""
        return self.inputs[np.asarray(pi, dtype=int)]

    # Grid lookups

    def nearest_node(self, x) -> int:
        """Return the node id of the grid state closest to ``x``."""
        index = self._nearest_index(x, self.x_lb, self.x_step, self.x_grid_shape)
        return int(np.ravel_multi_index(index, self.x_grid_shape))

    def nearest_action(self, u) -> int:
        """Return the action id of the grid input closest to ``u``."""
        index = self._nearest_index(u, self.u_lb, self.u_step, self.u_grid_shape)
        return int(np.ravel_multi_index(index, self.u_grid_shape))

    def slice_2d(self, grid_nd, axis_x: int = 0, axis_y: int = 1, anchor=None):
        """
        Take a 2-D ``(axis_x, axis_y)`` slice of an ``n``-D grid array.

        Other axes are pinned to the grid node nearest ``anchor`` (defaulting to
        the problem goal when available, otherwise the grid centre).
        """
        grid_nd = np.asarray(grid_nd)
        if self.n == 2:
            return grid_nd

        anchor = self._default_anchor() if anchor is None else np.asarray(anchor)
        index = list(
            self._nearest_index(anchor, self.x_lb, self.x_step, self.x_grid_shape)
        )
        nx, ny = self.x_grid_shape[axis_x], self.x_grid_shape[axis_y]
        out = np.empty((nx, ny), dtype=float)
        for i in range(nx):
            for j in range(ny):
                index[axis_x] = i
                index[axis_y] = j
                out[i, j] = grid_nd[tuple(index)]
        return out

    # Persistence

    def save(self, path: str) -> None:
        """Save the successor and validity tables to ``path`` (single ``.npz``)."""
        x_next, action_ok, x_next_ok = self.transition(0.0)
        np.savez(path, x_next=x_next, action_ok=action_ok, x_next_ok=x_next_ok)

    def load(self, path: str) -> None:
        """Load successor and validity tables saved by :meth:`save`."""
        data = np.load(path)
        self.x_next = data["x_next"]
        self.action_ok = data["action_ok"]
        self.x_next_ok = data["x_next_ok"]
        self.precomputed = True

    # Internal machinery

    @staticmethod
    def _grid_points(levels):
        """Cartesian product of per-axis levels as rows, C-order over the shape."""
        mesh = np.meshgrid(*levels, indexing="ij")
        return np.stack([axis.ravel() for axis in mesh], axis=1)

    @staticmethod
    def _step_sizes(lb, ub, shape):
        counts = np.asarray(shape, dtype=int)
        spans = np.asarray(ub, dtype=float) - np.asarray(lb, dtype=float)
        return np.where(counts > 1, spans / np.maximum(counts - 1, 1), 1.0)

    @staticmethod
    def _nearest_index(point, lb, step, shape):
        point = np.asarray(point, dtype=float).reshape(-1)
        raw = np.rint((point - lb) / step).astype(int)
        return tuple(np.clip(raw, 0, np.asarray(shape) - 1))

    def _default_anchor(self):
        goal = self.problem.x_goal
        if goal is not None:
            return np.asarray(goal, dtype=float)
        return 0.5 * (self.x_lb + self.x_ub)

    @staticmethod
    def _coerce_shape(shape, dim, label):
        shape = tuple(int(s) for s in shape)
        if len(shape) != dim:
            raise ValueError(f"{label} must have length {dim}")
        if any(s < 2 for s in shape):
            raise ValueError(f"{label} entries must be at least 2")
        return shape

    def _state_bounds(self, problem):
        box = (
            problem.X
            if isinstance(problem.X, BoxSet)
            else BoxSet.from_system_state(self.sys)
        )
        return self._require_finite_box(box.lower, box.upper, "state")

    def _input_bounds(self, problem):
        box = problem.U.box
        return self._require_finite_box(box.lower, box.upper, "input")

    @staticmethod
    def _require_finite_box(lower, upper, label):
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if not (np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))):
            raise ValueError(
                f"StateSpaceGrid needs finite {label} bounds; "
                f"set them on the system or pass an explicit box set."
            )
        return lower, upper


if __name__ == "__main__":
    from minilink.core.system import DynamicSystem

    class DoubleIntegrator(DynamicSystem):
        def __init__(self):
            super().__init__(n=2, input_dim=1, output_dim=2)
            self.state.lower_bound = np.array([-2.0, -2.0])
            self.state.upper_bound = np.array([2.0, 2.0])
            self.inputs["u"].lower_bound = np.array([-1.0])
            self.inputs["u"].upper_bound = np.array([1.0])

        def f(self, x, u, t=0, params=None):
            return np.array([x[1], u[0]])

    problem = PlanningProblem(DoubleIntegrator(), x_goal=np.zeros(2))
    grid = StateSpaceGrid(problem, x_grid_shape=(11, 11), u_grid_shape=(3,), dt=0.1)
    print("nodes:", grid.nodes_n, "actions:", grid.actions_n)
    print("x_next shape:", grid.successors().shape)
