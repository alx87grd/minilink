"""Value iteration: the Bellman equation solved backward on a discretized state space."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY
from minilink.planning.evaluation import nominal_trajectory
from minilink.planning.planner import Planner
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.lookup_policy import LookupTableController
from minilink.planning.policy_synthesis.progress import SweepLog, progress
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution

#: Per-node Python reference engine (pyro's base ``DynamicProgramming``).
BACKEND_LOOP = "loop"

_UNSET = object()  # marks a keyword the caller did not pass

# Public API


class DynamicProgrammingPlanner(Planner):
    """
    Value-iteration planner over a discretized state space.

    Sweeps the Bellman equation backward in time on a
    :class:`~minilink.planning.policy_synthesis.discretizer.StateSpaceGrid`,
    from the terminal cost ``h`` at the horizon, and keeps the minimizing
    action of every node as the greedy policy ``pi``::

        DynamicProgrammingPlanner(problem, x_grid=(201, 201), u_grid=(21,), dt=0.05)

    builds the grid itself; pass ``grid=`` for a custom one. The backend picks
    how the backward step runs: ``"loop"`` is the textbook double loop,
    ``"numpy"`` (default) the same step over lookup tables, and ``"jax"`` the
    table step jitted on device (:mod:`~minilink.planning.policy_synthesis.dp_jax`).

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid, optional
        Discretization of ``problem``'s state and input spaces. Give
        ``x_grid``, ``u_grid`` and ``dt`` instead and the planner builds it.
    options : DynamicProgrammingOptions, optional
        Workflow options. The flat keyword arguments below overlay matching
        fields: ``backend``, ``alpha``, ``tol``, ``max_iterations``,
        ``interpolation``, ``out_of_bound_cost``, ``final_time``,
        ``record_history``, ``verbose``, ``clean_infeasible``.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        grid: StateSpaceGrid | None = None,
        x_grid=None,
        u_grid=None,
        dt=None,
        options: DynamicProgrammingOptions | None = None,
        backend=_UNSET,
        alpha=_UNSET,
        tol=_UNSET,
        max_iterations=_UNSET,
        interpolation=_UNSET,
        out_of_bound_cost=_UNSET,
        final_time=_UNSET,
        record_history=_UNSET,
        verbose=_UNSET,
        clean_infeasible=_UNSET,
    ) -> None:
        super().__init__(problem)
        self.require_cost()

        # Discretized state and input spaces
        self.grid = grid_of(problem, grid, x_grid, u_grid, dt)

        # Workflow options: the flat keywords over the bag, then the problem's defaults
        self.options = options_of(
            problem,
            self.grid,
            options,
            backend=backend,
            alpha=alpha,
            tol=tol,
            max_iterations=max_iterations,
            interpolation=interpolation,
            out_of_bound_cost=out_of_bound_cost,
            final_time=final_time,
            record_history=record_history,
            verbose=verbose,
            clean_infeasible=clean_infeasible,
        )

        # Memory
        self.result: DynamicProgrammingResult | None = None  # latest solve's tables
        self._G = None  # running-cost table, built once when the grid is precomputed
        self._jax_cache = {}  # the JAX engine's device tables and compiled loops

        # The JAX engine builds the grid's transition tables with JAX
        if self.options.backend == BACKEND_JAX:
            self.grid.ensure_jax_transition(self.options.final_time)

    def solve(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Infinite horizon: sweep backward until the cost-to-go stops changing.

        Stops once the largest change of ``J`` falls under ``tol``, or after
        ``max_iterations`` sweeps. The solution's policy is the greedy lookup
        table and its ``cost_to_go`` the interpolated ``J``. ``evaluate=True``
        also rolls the law out from the problem's start and scores it over the
        problem's draws.
        """
        # Backward sweeps until the cost-to-go stops changing
        result = self.value_iteration(self.options.max_iterations, stop_on_tol=True)

        # The cost-to-go and policy tables as a planning solution
        return self.dp_result_to_planning_solution(
            result, fixed_horizon=False, evaluate=evaluate, n_trials=n_trials
        )

    def solve_steps(self, n: int, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Finite horizon: exactly ``n`` backward sweeps from the terminal cost."""
        # Exactly n backward sweeps from the terminal cost
        result = self.value_iteration(int(n), stop_on_tol=False)

        # The cost-to-go and policy tables as a planning solution
        return self.dp_result_to_planning_solution(
            result, fixed_horizon=True, evaluate=evaluate, n_trials=n_trials
        )

    def solve_policy(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """The planner-family name of :meth:`solve`, answered by every policy planner."""
        return self.solve(evaluate=evaluate, n_trials=n_trials)

    # Value iteration

    def value_iteration(
        self, max_sweeps: int, *, stop_on_tol: bool = True
    ) -> DynamicProgrammingResult:
        """
        Sweep the Bellman equation backward from the terminal cost.

        Each sweep steps time back by ``dt`` and replaces the cost-to-go with
        the best one-step backup. Stops after ``max_sweeps`` or, with
        ``stop_on_tol``, once the largest change falls under ``tol``. Returns
        the raw tables; :meth:`solve` and :meth:`solve_steps` wrap them in a
        solution.
        """
        opt = self.options
        tf = opt.final_time
        dt = self.grid.dt
        tol = opt.tol

        # The JAX engine runs these same sweeps, jitted on device
        if opt.backend == BACKEND_JAX:
            from minilink.planning.policy_synthesis import dp_jax

            return dp_jax.value_iteration(self, max_sweeps, stop_on_tol)

        # The backward step: the textbook loop, or the same step over lookup tables
        if opt.backend == BACKEND_LOOP:
            backward_step = self.backward_step_loop
        else:
            backward_step = self.backward_step_table

        log = SweepLog(opt, max_sweeps, stop_on_tol)

        # Cost-to-go at the final time
        J = self.terminal_cost(tf)
        pi = np.zeros(self.grid.nodes_n, dtype=int)
        log.start(tf, J, pi)

        # Backward sweeps: at most max_sweeps of them, and with stop_on_tol
        # only until the largest change of the cost-to-go falls under tol
        k = 0
        delta = np.inf

        while k < max_sweeps and (not stop_on_tol or delta > tol):
            # One step back in time (from the default tf = 0, t runs negative,
            # which only matters when f or g depends on time)
            k = k + 1
            t = tf - k * dt

            # Bellman backup from the cost-to-go one step ahead:
            # J(x) = min_u [ g(x, u, t) dt + alpha J_next(x + f(x, u, t) dt) ]
            J_next = J
            J, pi = backward_step(J_next, t)

            # Largest change of the cost-to-go: the convergence measure
            delta = float(np.max(np.abs(J - J_next)))

            log.sweep(k, t, J, J_next, pi)

        log.done(k, delta)

        return DynamicProgrammingResult(
            grid=self.grid,
            J=np.array(J),
            pi=np.array(pi, dtype=int),
            iterations=k,
            delta=delta,
            history=log.history,
        )

    def backward_step_loop(self, J_next, t):
        """
        One backward Bellman step as an explicit loop over nodes and actions.

        The readable reference (pyro's ``DynamicProgramming``): for every
        state node and every control action, take a forward-Euler step, look
        up the arrival cost-to-go, and keep the cheapest action.
        """
        grid = self.grid
        sys = self.problem.sys
        cost = self.problem.cost
        X = self.problem.X
        U = self.problem.U
        params = self.problem.params
        dt = grid.dt
        alpha = self.options.alpha
        INF = self.options.out_of_bound_cost  # a large finite penalty (pyro's cf.INF)

        # Interpolation of the cost-to-go one step ahead
        J_interpol = grid.build_interpolator(J_next, self.options.interpolation)

        # New cost-to-go and policy to be computed
        J = np.zeros(grid.nodes_n)
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

                    # If the next state is in X and on the grid (the table's domain)
                    if X.contains(x_next, t, params.sets) and grid.X.contains(x_next):
                        # Estimated (interpolation) cost-to-go of the arrival state
                        # (the interpolator takes a batch of states: [0] is this one)
                        J_arrival = J_interpol(x_next)[0]

                        # Cost-to-go of a given action
                        Q[a] = cost.g(x, u, t, params.cost) * dt + alpha * J_arrival

                    else:
                        # The next state leaves the admissible states: the penalty
                        Q[a] = INF

                else:
                    # Invalid control input at this state
                    Q[a] = INF

            # Best action at this node
            J[s] = Q.min()
            pi[s] = Q.argmin()

        return J, pi

    def backward_step_table(self, J_next, t):
        """The same backward step, vectorized over the successor and running-cost tables."""
        grid = self.grid
        alpha = self.options.alpha
        method = self.options.interpolation
        n, N, A = grid.n, grid.nodes_n, grid.actions_n  # state dim, nodes, actions

        # Euler successors x_next = x + f(x, u, t) dt of every (node, action) pair,
        # shape (N, A, n), and which pairs are admissible
        x_next, action_ok, x_next_ok = grid.transition(t)
        admissible = action_ok & x_next_ok

        # Running-cost table, with the out-of-bound cost on inadmissible pairs
        G = self.running_cost_table(t, admissible)

        # Estimated (interpolation) cost-to-go of all the arrival states:
        # the (N, A, n) successors flattened to N*A states, then back to (N, A)
        J_arrival = grid.interpolate(J_next, x_next.reshape(-1, n), method)
        J_arrival = J_arrival.reshape(N, A)

        # Q(x, u) = g(x, u, t) dt + alpha J_next(x_next), for every pair at once
        Q = G + alpha * J_arrival

        # Best action at every node
        J = Q.min(axis=1)
        pi = Q.argmin(axis=1)

        return J, pi

    def terminal_cost(self, t) -> np.ndarray:
        """The terminal cost ``h`` at time ``t`` of every grid node, by node id."""
        grid = self.grid
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        N = grid.nodes_n

        J = np.empty(N, dtype=float)
        nodes = progress(
            range(N), "Computing h(x,t) terminal cost", self.options.verbose
        )

        # For all state nodes
        for s in nodes:
            x = grid.states[s]

            # Final cost of the state
            J[s] = float(h(x, t, cost_params))

        return J

    def running_cost_table(self, t, admissible=None) -> np.ndarray:
        """
        Running cost of every (node, action) pair over one step, shape ``(nodes_n, actions_n)``.

        Pairs that are not ``admissible`` (action outside the input set, or a
        successor outside the state set or the grid) cost ``out_of_bound_cost``.
        The mask is read from the grid's transition tables when not given.

        Warning: on a precomputed grid the table is built once, at the first
        sweep's time, and reused for every sweep. A running cost ``g`` that
        depends on ``t`` is then frozen at that time. Use a grid with
        ``precompute=False`` for a time-varying cost.
        """
        # Built once on a precomputed grid: this assumes g does not depend on t
        if self._G is not None:
            return self._G

        grid = self.grid
        g = self.problem.cost.g
        cost_params = self.problem.params.cost
        dt = grid.dt
        N, A = grid.nodes_n, grid.actions_n
        INF = self.options.out_of_bound_cost  # a large finite penalty (pyro's cf.INF)

        if admissible is None:
            _, action_ok, x_next_ok = grid.transition(t)
            admissible = action_ok & x_next_ok

        G = np.empty((N, A), dtype=float)
        nodes = progress(
            range(N),
            "Computing g(x,u,t) look-up table",
            self.options.verbose,
            unit="pairs",
            per_item=A,
        )

        # For all state nodes
        for s in nodes:
            x = grid.states[s]

            # For all control actions
            for a in range(A):
                u = grid.inputs[a]

                # Running cost over one step
                G[s, a] = float(g(x, u, t, cost_params)) * dt

        # Out of bound cost on the pairs that leave the admissible set
        G[~admissible] = INF

        if grid.precomputed:
            self._G = G

        return G

    # Controller, cleanup and plots

    def get_controller(self, **kwargs):
        """The solution's greedy lookup law; ``interpolation=`` builds a variant of it."""
        solution = self.require_solution()
        if not kwargs:
            return solution.policy

        return LookupTableController(self.result.grid, self.result.pi, **kwargs)

    def value_at(self, x) -> float:
        """Interpolate the cost-to-go at the latest solve."""
        self.require_solution()
        return self.result.value_at(x)

    def nominal_trajectory(self, tf=None):
        """The greedy law from the problem's start on the grid's control period."""
        return nominal_trajectory(
            self.problem, self.get_controller(), dt=self.grid.dt, tf=tf
        )

    def clean_infeasible_set(self, tol: float = 1.0) -> DynamicProgrammingResult:
        """
        Flag states whose cost-to-go has saturated at ``out_of_bound_cost``.

        Their value is pinned to the penalty and their policy to the action
        nearest the system's nominal input, mirroring pyro's cleanup pass.
        """
        result = self.result
        INF = self.options.out_of_bound_cost

        # The action nearest the system's nominal input
        u_nominal = self.problem.sys.get_u_from_input_ports()
        default_action = self.grid.nearest_action(u_nominal)

        # Nodes from which leaving the admissible set is unavoidable
        infeasible = result.J > (INF - tol)

        result.J[infeasible] = INF
        result.pi[infeasible] = default_action

        return result

    def plot_cost2go(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        self.require_solution()
        return plotting.plot_cost2go(self.result, **kwargs)

    def plot_policy(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        self.require_solution()
        return plotting.plot_policy(self.result, **kwargs)

    def animate_cost2go(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        self.require_solution()
        return plotting.animate_cost2go(self.result, **kwargs)

    def animate_policy(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        self.require_solution()
        return plotting.animate_policy(self.result, **kwargs)

    # Internal machinery

    def dp_result_to_planning_solution(
        self, result: DynamicProgrammingResult, *, fixed_horizon, evaluate, n_trials
    ) -> PlanningSolution:
        """
        Turn the value-iteration tables into the planner's :class:`PlanningSolution`.

        Keeps the tables on ``planner.result``, cleans the infeasible cells,
        builds the greedy lookup law, records how the sweeps ended, optionally
        rolls the law out, and stores the solution.
        """
        tol = float(self.options.tol)
        dt = self.grid.dt

        # Keep the raw tables on the planner (plots and cleanup read them there)
        self.result = result

        # Pin the saturated cells to the penalty, before the law is built from them
        if self.options.clean_infeasible:
            self.clean_infeasible_set()

        # The greedy policy: the minimizing action of every node, interpolated between
        # nodes
        policy = LookupTableController(result.grid, result.pi)

        # How the sweeps ended: converged to tol (a fixed horizon always completes
        # its sweeps)
        record = ValueIterationRecord(
            iterations=result.iterations,
            delta=float(result.delta),
            tol=tol,
            converged=fixed_horizon or result.delta <= tol,
            fixed_horizon=fixed_horizon,
        )

        # Optionally, roll the law out from the problem's start and score it over
        # its draws
        trajectory = None
        evaluation = None
        if evaluate:
            trajectory = nominal_trajectory(self.problem, policy, dt=dt)
            evaluation = self.evaluate(policy, dt=dt, n_trials=n_trials)

        # The solution: the law, the record, the rollout, the interpolated cost-to-go
        solution = PlanningSolution(
            self.problem, policy, record, trajectory, evaluation, result.value_at
        )

        return self.store_solution(solution)


@dataclass
class DynamicProgrammingOptions:
    """
    Workflow options for :class:`DynamicProgrammingPlanner`.

    Parameters
    ----------
    backend : {"numpy", "loop", "jax"}
        Backward-step engine. ``"numpy"`` (default) is the vectorized lookup
        table, ``"loop"`` the per-node Python reference, ``"jax"`` the jitted
        device version (linear/nearest interpolation only; traceable cost/sets;
        its tables are built once, so ``f`` and ``g`` must not depend on time).
    alpha : float
        Discount (exponential forgetting) factor; use ``alpha < 1`` for
        guaranteed contraction in infinite-horizon value iteration.
        When omitted, the cost's ``discount_factor(dt)`` is used if the
        cost declares a positive ``discount_rate``; otherwise ``1`` (undiscounted).
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
        Terminal time ``tf``; sweeps step backward as ``t = tf - k dt``. Left
        at ``0.0``, the planner reads ``problem.tf`` when the problem sets one.
    record_history : bool
        Keep ``(t, J, pi)`` per sweep for animation.
    verbose : bool
        Print build progress (50k-item counts with elapsed time and ETA) for
        lookup-table precompute and backward Bellman sweeps. Grid mesh and
        ``x_next`` progress use :attr:`StateSpaceGrid.verbose`; ``G``, ``J0``,
        and sweeps use this flag. Progress lines update in place about every
        2% until each step completes. Set both to ``False`` for silent runs.
    clean_infeasible : bool
        After each solve, pin saturated cost-to-go cells to ``out_of_bound_cost``
        and their policy to the nominal action (:meth:`DynamicProgrammingPlanner.clean_infeasible_set`).
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
    clean_infeasible: bool = True


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

    def value_at(self, x) -> float:
        """Interpolate the cost-to-go at an arbitrary state ``x``."""
        return float(self.grid.interpolate(self.J, np.atleast_2d(x))[0])

    def save(self, path: str) -> None:
        """Save ``J`` and ``pi`` to ``path`` (single ``.npz``)."""
        np.savez(path, J=self.J, pi=self.pi)

    @classmethod
    def load(cls, path: str, grid: StateSpaceGrid) -> DynamicProgrammingResult:
        """Load a result saved by :meth:`save`, bound to ``grid``."""
        data = np.load(path)

        return cls(
            grid=grid, J=data["J"], pi=data["pi"], iterations=0, delta=float("nan")
        )


@dataclass(frozen=True)
class ValueIterationRecord:
    """Sweeps run, the last cost-to-go change, and whether the sweeps converged to ``tol``."""

    iterations: int
    delta: float
    tol: float
    converged: bool
    fixed_horizon: bool

    @property
    def success(self) -> bool:
        """The sweeps converged (a fixed-horizon solve always completes its sweeps)."""
        return bool(self.converged)

    def __str__(self) -> str:
        if self.fixed_horizon:
            return f"{self.iterations} backward sweeps (fixed horizon)"

        if self.converged:
            return f"converged in {self.iterations} sweeps (delta={self.delta:.3g})"

        return (
            f"max_iterations={self.iterations} reached before tol={self.tol:g} "
            f"(delta={self.delta:.3g})"
        )


# Internal machinery


def grid_of(problem, grid, x_grid, u_grid, dt) -> StateSpaceGrid:
    """The grid passed in, or one built from the shapes and ``dt`` (never both)."""
    if grid is None:
        if x_grid is None or u_grid is None or dt is None:
            raise ValueError(
                "pass grid=StateSpaceGrid(...) or all of x_grid, u_grid, and dt"
            )

        return StateSpaceGrid(problem, x_grid_shape=x_grid, u_grid_shape=u_grid, dt=dt)

    if x_grid is not None or u_grid is not None or dt is not None:
        raise ValueError("pass either grid= or x_grid/u_grid/dt, not both")

    return grid


def options_of(problem, grid, options, **flat) -> DynamicProgrammingOptions:
    """The options bag with the passed flat keywords laid over it, then the problem's own defaults."""
    base = DynamicProgrammingOptions() if options is None else options
    given = {key: value for key, value in flat.items() if value is not _UNSET}
    opt = replace(base, **given) if given else base

    # The problem's price of infeasibility is the default price of leaving the grid
    if "out_of_bound_cost" not in given and isinstance(problem.infeasible_cost, float):
        opt = replace(opt, out_of_bound_cost=problem.infeasible_cost)

    # The cost's continuous discount rate becomes the per-step factor, unless alpha
    # is set
    if "alpha" not in given and float(problem.cost.discount_rate) > 0.0:
        opt = replace(opt, alpha=problem.cost.discount_factor(grid.dt))

    # The problem's horizon is the terminal time unless final_time is set
    if "final_time" not in given and opt.final_time == 0.0:
        tf = getattr(problem, "tf", None)
        if tf is not None and np.isfinite(tf):
            opt = replace(opt, final_time=float(tf))

    if opt.backend not in (BACKEND_LOOP, BACKEND_NUMPY, BACKEND_JAX):
        raise ValueError(f"Unknown backend {opt.backend!r}")

    return opt
