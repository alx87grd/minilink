"""Value iteration: the Bellman equation solved backward on a discretized state space."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import partial

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY
from minilink.planning.dynamic_programming import plotting
from minilink.planning.dynamic_programming.discretizer import StateSpaceGrid
from minilink.planning.dynamic_programming.lookup_policy import LookupPolicy
from minilink.planning.dynamic_programming.progress import SweepLog, progress
from minilink.planning.evaluator import nominal_trajectory
from minilink.planning.planner import (
    Planner,
    given_numbers,
    planner_at,
    planner_evidence,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution
from minilink.planning.step_problems import StepPlanningProblem

#: Per-node Python reference engine (pyro's base ``DynamicProgramming``).
BACKEND_LOOP = "loop"


###############################################################################
# Public API
###############################################################################


def bellman_backup(J_next, G, arrival, gamma):
    """
    One Bellman backup: ``J(x) = min_u [ c(x, u) + γ J_next(x_next) ]``, with ``π(x)`` its argmin.

    ``G`` is the stage cost of every (node, action) pair, shape ``(N, A)``, the
    failure cost on an inadmissible pair; ``arrival(J_next)`` reads the
    cost-to-go one step ahead at every pair's successor, shape ``(N, A)``, zero
    off the grid (the pair is inadmissible and ``G`` already prices it). The
    same backup on NumPy and JAX: each engine passes its own reading of the grid.
    """
    # The cost-to-go one step ahead at every pair's successor
    J_arrival = arrival(J_next)

    # Q(x, u) = c(x, u) + γ J_next(x_next), for every (node, action) pair at once
    Q = G + gamma * J_arrival

    # The best action at every node
    J = Q.min(axis=1)
    pi = Q.argmin(axis=1)

    return J, pi


class DynamicProgrammingPlanner(Planner):
    """
    Value-iteration planner over a discretized state space.

    Sweeps the Bellman equation backward in time on a
    :class:`~minilink.planning.dynamic_programming.discretizer.StateSpaceGrid`,
    from the terminal cost ``h`` at the horizon, and keeps the minimizing
    action of every node as the greedy policy ``pi``::

        DynamicProgrammingPlanner(problem, x_grid=(201, 201), u_grid=(21,), dt=0.05)

    builds the grid itself; pass ``grid=`` for a custom one. The backend picks
    how the backward step runs: ``"loop"`` is the textbook double loop,
    ``"numpy"`` (default) the same step over lookup tables, and ``"jax"`` the
    table step jitted on device (:mod:`~minilink.planning.dynamic_programming.dp_jax`).

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid, optional
        Discretization of ``problem``'s state and input spaces. Give
        ``x_grid``, ``u_grid`` and ``dt`` instead and the planner builds it.
    backend : {"numpy", "loop", "jax"}
        Backward-step engine. ``"numpy"`` (default) is the vectorized lookup
        table, ``"loop"`` the per-node Python reference, ``"jax"`` the jitted
        device version (linear/nearest interpolation only; traceable cost/sets;
        its tables are built once, so ``f`` and ``g`` must not depend on time).
    gamma : float, optional
        The discount factor per step, a number of :attr:`params`. Unset, the
        problem's ``discount_factor(dt)`` (``1`` on an undiscounted problem).
        Given on a continuous problem, it is the planner's own discount: the
        sweeps plan on a copy of the problem at ``ρ = −ln γ / dt`` (the stage
        integral and the backup both), the record names it, and the score of
        the policy stays the problem's.
    tol, max_iterations : float, int
        Stopping rules: the largest cost-to-go change per sweep, and the cap
        on the backward sweeps.
    interpolation : {"linear", "nearest", "cubic", "quintic"}
        Cost-to-go interpolation. ``"linear"`` (default) and ``"nearest"`` are
        robust; spline methods (``"cubic"``, the ``"spline"`` alias, ``"quintic"``)
        are smoother but can ring across the infeasibility penalty.
    out_of_bound_cost : float or callable, optional
        Finite penalty charged to inadmissible inputs or out-of-domain
        successors, a number of :attr:`params`: a scalar, or
        ``out_of_bound_cost(x_next, t)`` at each inadmissible pair's successor
        (the ``"jax"`` backend needs it traceable and time-invariant). Unset,
        the cost of a failure: the problem's ``failure_cost`` when a number,
        else the face's cost bound on this discount (:func:`grid_failure_cost`).
    clean_infeasible : bool
        After each solve, pin saturated cost-to-go cells to ``out_of_bound_cost``
        and their policy to the nominal action (:meth:`clean_infeasible_set`);
        a callable ``out_of_bound_cost`` leaves the tables as solved.
    record_history, verbose : bool
        Workflow: keep ``(t, J, pi)`` per sweep for animation; print build and
        sweep progress (the grid's own progress follows
        :attr:`StateSpaceGrid.verbose`).

    Notes
    -----
    The sweeps' terminal time, :attr:`final_time`, is the problem's horizon
    end (``t0 + horizon``, or a step problem's last tick); ``0`` without one.
    """

    number_names = ("gamma", "out_of_bound_cost")

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        grid: StateSpaceGrid | None = None,
        x_grid=None,
        u_grid=None,
        dt=None,
        solver="euler",
        backend: str = BACKEND_NUMPY,
        gamma: float | None = None,
        tol: float = 0.1,
        max_iterations: int = 1000,
        interpolation: str = "linear",
        out_of_bound_cost=None,
        clean_infeasible: bool = True,
        record_history: bool = False,
        verbose: bool = False,
    ) -> None:
        super().__init__(problem)
        self.require_cost()

        # Discretized state and input spaces, on the planner's own discount when
        # gamma overrides the problem's: one rate inside and between the periods
        grid = grid_of(problem, grid, x_grid, u_grid, dt, solver)
        self.discount_rate = tool_rate_of(problem, gamma, grid.dt)
        self.grid = grid_on_rate(grid, self.discount_rate)

        # Structure: the engine, the stopping rules, the interpolation
        self.backend = require_dp_backend(backend)
        self.tol = float(tol)
        self.max_iterations = int(max_iterations)
        self.interpolation = interpolation
        self.clean_infeasible = bool(clean_infeasible)

        # The numbers, the problem's unless given, and the sweeps' terminal time
        self.gamma, self.out_of_bound_cost, self.final_time = grid_numbers_of(
            self.grid, gamma, out_of_bound_cost
        )
        self.given = given_numbers(gamma=gamma, out_of_bound_cost=out_of_bound_cost)

        # Workflow
        self.record_history = bool(record_history)
        self.verbose = bool(verbose)

        # The JAX engine's compiled step and loops, by structure (their numbers are
        # arguments)
        self.compiled = {}

        # The JAX engine builds the grid's transition tables with JAX
        if self.backend == BACKEND_JAX:
            self.grid.ensure_jax_transition(self.final_time)

    def solve(self, *, params=None, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Infinite horizon: sweep backward until the cost-to-go stops changing.

        Stops once the largest change of ``J`` falls under ``tol``, or after
        ``max_iterations`` sweeps. The solution's policy is the greedy lookup
        table and its ``cost_to_go`` the interpolated ``J``. ``evaluate=True``
        also rolls the law out from the problem's start and scores it over the
        problem's draws. ``params`` (the complete tree of :attr:`params`, a copy
        edited) solves the planner rebuilt there (:func:`~minilink.planning.planner.planner_at`),
        this one unchanged.
        """
        # At another tree: the planner there, its solution kept here
        if params is not None:
            solution = planner_at(self, params).solve(
                evaluate=evaluate, n_trials=n_trials
            )
            return self.store_solution(solution)

        # Backward sweeps until the cost-to-go stops changing
        result = self.value_iteration(self.max_iterations, stop_on_tol=True)

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

    @property
    def result(self) -> DynamicProgrammingResult | None:
        """The latest solution's tables, ``J`` and ``pi`` (a shortcut: a tool reads ``solution.record.tables``)."""
        if self.last_solution is None:
            return None

        return self.last_solution.record.tables

    # Value iteration

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: the grid's shape, the engine, the stopping rules, the workflow, the numbers it was given."""
        grid = self.grid
        kwargs = dict(
            x_grid=grid.x_grid_shape,
            u_grid=grid.u_grid_shape,
            dt=grid.dt,
            solver=grid.solver,
            backend=self.backend,
            tol=self.tol,
            max_iterations=self.max_iterations,
            interpolation=self.interpolation,
            clean_infeasible=self.clean_infeasible,
            record_history=self.record_history,
            verbose=self.verbose,
        )

        return {**kwargs, **self.given}

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
        opt = self
        tf = opt.final_time
        dt = self.grid.dt
        tol = opt.tol

        # The JAX engine runs these same sweeps, jitted on device
        if opt.backend == BACKEND_JAX:
            # A real cycle: dp_jax imports DynamicProgrammingResult from this module
            from minilink.planning.dynamic_programming import dp_jax

            return dp_jax.value_iteration(self, max_sweeps, stop_on_tol)

        # The backward step: the textbook loop, or the same step over lookup tables
        if opt.backend == BACKEND_LOOP:
            backward_step = self.backward_step_loop
        else:
            backward_step = self.backward_step_table

        log = SweepLog(opt, max_sweeps, stop_on_tol)

        # The running-cost table of this solve, once on a precomputed grid (its
        # transitions do not depend on time); per sweep otherwise
        G = self.running_cost_table(tf) if self.grid.precomputed else None

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
            # J(x_k) = min_u [ c(x_k, u) + γ J_next(x_{k+1}) ], the face's stage cost and step
            J_next = J
            J, pi = backward_step(J_next, t, G)

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

    def backward_step_loop(self, J_next, t, G=None):
        """
        One backward Bellman step as an explicit loop over nodes and actions (``G`` unused: each pair prices its own step).

        The readable reference (pyro's ``DynamicProgramming``): for every
        state node and every control action, take a forward-Euler step, look
        up the arrival cost-to-go, and keep the cheapest action.
        """
        grid = self.grid
        face = grid.face
        params = face.params
        X = self.problem.X
        U = self.problem.U
        gamma = self.gamma
        INF = self.out_of_bound_cost  # The finite failure cost (pyro's cf.INF)

        # Interpolation of the cost-to-go one step ahead
        J_interpol = grid.build_interpolator(J_next, self.interpolation)

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

                # x_{k+1}, c = transition(x_k, u, t): the step face's tick
                x_next, c = face.transition_at(x, u, t, params)

                # If action is in allowable set
                if U.contains(u, x, t):
                    # If the next state is in X and on the grid (the table's domain)
                    if X.contains(x_next, t) and grid.X.contains(x_next):
                        # Estimated (interpolation) cost-to-go of the arrival state
                        # (the interpolator takes a batch of states: [0] is this one)
                        J_arrival = J_interpol(x_next)[0]

                        # Cost-to-go of a given action: Q = c + γ J(x_{k+1})
                        Q[a] = c + gamma * J_arrival

                    else:
                        # The next state leaves the admissible states: the penalty
                        Q[a] = infeasible_penalty(INF, x_next, t)

                else:
                    # Invalid control input at this state
                    Q[a] = infeasible_penalty(INF, x_next, t)

            # Best action at this node
            J[s] = Q.min()
            pi[s] = Q.argmin()

        return J, pi

    def backward_step_table(self, J_next, t, G=None):
        """The same backward step, vectorized over the successor and running-cost tables (``G`` the solve's table, else built at ``t``)."""
        grid = self.grid
        gamma = self.gamma
        method = self.interpolation
        n = grid.n  # State dimension
        N = grid.nodes_n  # Number of nodes
        A = grid.actions_n  # Number of actions

        # Euler successors x_next = x + f(x, u, t) dt of every (node, action) pair,
        # shape (N, A, n), and which pairs are admissible
        x_next, action_ok, x_next_ok = grid.transition(t)
        admissible = action_ok & x_next_ok

        # Running-cost table, with the out-of-bound cost on inadmissible pairs
        if G is None:
            G = self.running_cost_table(t, admissible, x_next)

        # The cost-to-go one step ahead read at the successors (zero off the grid):
        # the (N, A, n) successors flattened to N*A states, then back to (N, A)
        def arrival(J):
            return grid.interpolate(J, x_next.reshape(-1, n), method).reshape(N, A)

        # J(x) = min_u [ c(x, u, t) + γ J_next(x_next) ]
        J, pi = bellman_backup(J_next, G, arrival, gamma)

        return J, pi

    def terminal_cost(self, t) -> np.ndarray:
        """The terminal cost ``h`` at time ``t`` of every grid node, by node id."""
        grid = self.grid
        face = grid.face
        N = grid.nodes_n

        J = np.empty(N, dtype=float)
        nodes = progress(range(N), "Computing h(x,t) terminal cost", self.verbose)

        # For all state nodes
        for s in nodes:
            x = grid.states[s]

            # Final cost of the state
            J[s] = float(face.terminal(x, t))

        return J

    def running_cost_table(self, t, admissible=None, x_next=None) -> np.ndarray:
        """
        Stage cost of every (node, action) pair over one step, shape ``(nodes_n, actions_n)``: the step face's ``c`` (``g dt`` under Euler).

        Pairs that are not ``admissible`` (action outside the input set, or a
        successor outside the state set or the grid) cost ``out_of_bound_cost``:
        the scalar, or ``out_of_bound_cost(x_next, t)`` at the pair's successor.
        The mask and the successors are read from the grid's transition tables
        when not given.

        Warning: on a precomputed grid a solve builds the table once, at the
        final time, and reuses it for every sweep. A running cost ``g``, or a
        cost ``out_of_bound_cost(x_next, t)``, that depends on ``t`` is then
        frozen at that time. Use a grid with ``precompute=False`` for a
        time-varying cost.
        """
        grid = self.grid
        params = grid.face.params
        N = grid.nodes_n
        A = grid.actions_n
        INF = self.out_of_bound_cost  # The finite failure cost (pyro's cf.INF)

        # The grid's mask, and its successors when a callable cost needs them
        if admissible is None or (x_next is None and callable(INF)):
            successors, action_ok, x_next_ok = grid.transition(t)
            admissible = action_ok & x_next_ok if admissible is None else admissible
            x_next = successors if x_next is None else x_next

        G = np.empty((N, A), dtype=float)
        nodes = progress(
            range(N),
            "Computing g(x,u,t) look-up table",
            self.verbose,
            unit="pairs",
            per_item=A,
        )

        # The stage cost of the period at t, its laws built once for every pair
        stage_cost = grid.stage_cost_law(t, params)

        # For all state nodes
        for s in nodes:
            x = grid.states[s]

            # For all control actions
            for a in range(A):
                u = grid.inputs[a]

                # The stage cost over one step, c = ∫₀^dt e^{−ρτ} g dτ (g dt under Euler)
                G[s, a] = float(stage_cost(x, u))

        # Out of bound cost on the pairs that leave the admissible set: the scalar,
        # or the cost of each pair's successor
        if callable(INF):
            exits = x_next[~admissible]
            G[~admissible] = [infeasible_penalty(INF, x, t) for x in exits]
        else:
            G[~admissible] = INF

        return G

    # Controller, cleanup and plots

    def get_controller(self, **kwargs):
        """The solution's greedy lookup law; ``interpolation=`` builds a variant of it."""
        solution = self.require_solution()
        if not kwargs:
            return solution.policy

        return LookupPolicy(self.result.grid, self.result.pi, **kwargs)

    def value_at(self, x) -> float:
        """Interpolate the cost-to-go at the latest solve, by the options' interpolation."""
        self.require_solution()
        return self.result.value_at(x, self.interpolation)

    def nominal_trajectory(self, tf=None, *, solution=None):
        """``solution``'s greedy law (the latest when none is given) from the problem's start on the grid's control period."""
        solution = self.require_solution() if solution is None else solution
        return nominal_trajectory(self.problem, solution.policy, dt=self.grid.dt, tf=tf)

    def clean_infeasible_set(
        self, tol: float = 1.0, *, tables=None
    ) -> DynamicProgrammingResult:
        """
        Flag states whose cost-to-go has saturated at ``out_of_bound_cost``.

        Their value is pinned to the penalty and their policy to the action
        nearest the system's nominal input, mirroring pyro's cleanup pass. A
        callable ``out_of_bound_cost(x_next, t)`` has no single saturation
        level: the tables are then left as solved. ``tables`` defaults to the
        latest solution's; a solve passes its own before building its solution.
        """
        result = self.result if tables is None else tables
        require_tables(result)

        INF = self.out_of_bound_cost

        # A cost that varies with the exit state saturates at no single level
        if callable(INF):
            return result

        # The action nearest the system's nominal input
        u_nominal = self.problem.sys.get_u_from_input_ports()
        default_action = self.grid.nearest_action(u_nominal)

        # Nodes from which leaving the admissible set is unavoidable
        infeasible = result.J > (INF - tol)

        result.J[infeasible] = INF
        result.pi[infeasible] = default_action

        return result

    def plot_cost_to_go(self, **kwargs):
        """The cost-to-go table ``J`` on the grid itself (``jmax``, ``axes``, ``show_3d``, ``title``, ...)."""
        self.require_solution()
        kwargs.setdefault("jmax", plotting.cost_scale_limit(self.out_of_bound_cost))
        return plotting.plot_cost_to_go(self.result, **kwargs)

    def plot_control_law(self, **kwargs):
        """The policy table ``π`` on the grid itself (``trajectory``, ``axis``, ``axes``, ...)."""
        self.require_solution()
        return plotting.plot_control_law(self.result, **kwargs)

    def animate_cost_to_go(self, **kwargs):
        self.require_solution()
        kwargs.setdefault("jmax", plotting.cost_scale_limit(self.out_of_bound_cost))
        return plotting.animate_cost_to_go(self.result, **kwargs)

    def animate_policy(self, **kwargs):
        self.require_solution()
        return plotting.animate_policy(self.result, **kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def dp_result_to_planning_solution(
        self, result: DynamicProgrammingResult, *, fixed_horizon, evaluate, n_trials
    ) -> PlanningSolution:
        """
        Turn the value-iteration tables into the planner's :class:`PlanningSolution`.

        Cleans the infeasible cells, builds the greedy lookup law, records how
        the sweeps ended with the tables themselves, optionally rolls the law
        out, and stores the solution.
        """
        tol = float(self.tol)
        dt = self.grid.dt

        # Pin the saturated cells to the penalty, before the law is built from them
        if self.clean_infeasible:
            self.clean_infeasible_set(tables=result)

        # The greedy policy: the minimizing action of every node, interpolated between
        # nodes
        policy = LookupPolicy(result.grid, result.pi)

        # How the sweeps ended: converged to tol (a fixed horizon always completes
        # its sweeps)
        record = ValueIterationRecord(
            iterations=result.iterations,
            delta=float(result.delta),
            tol=tol,
            converged=fixed_horizon or result.delta <= tol,
            fixed_horizon=fixed_horizon,
            discount_rate=self.discount_rate,
            tables=result,
        )

        # Optionally, roll the law out from the problem's start and score it over
        # its draws
        trajectory = None
        evaluation = None
        if evaluate:
            trajectory = nominal_trajectory(self.problem, policy, dt=dt)
            evaluation = planner_evidence(self, policy, dt=dt, n_trials=n_trials)

        # The solution: the law, the record, the rollout, the cost-to-go interpolated
        # as the planner's options say
        cost_to_go = partial(result.value_at, method=self.interpolation)
        solution = PlanningSolution(
            self.problem, policy, record, trajectory, evaluation, cost_to_go
        )

        return self.store_solution(solution)


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

    def value_at(self, x, method="linear") -> float:
        """Interpolate the cost-to-go at an arbitrary state ``x`` (``method``: the grid's interpolation)."""
        J = self.grid.interpolate(self.J, np.atleast_2d(x), method)

        return float(J[0])

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
    """Sweeps run, the last cost-to-go change, whether the sweeps converged to ``tol``, the planner's own discount rate when ``gamma`` overrode the problem's (``None``: the problem's), and the tables ``J`` and ``pi`` the sweeps ended on."""

    iterations: int
    delta: float
    tol: float
    converged: bool
    fixed_horizon: bool
    discount_rate: float | None = None
    tables: DynamicProgrammingResult | None = field(
        default=None, repr=False, compare=False
    )

    @property
    def success(self) -> bool:
        """The sweeps converged (a fixed-horizon solve always completes its sweeps)."""
        return bool(self.converged)

    def __str__(self) -> str:
        rate = (
            ""
            if self.discount_rate is None
            else f"; planned at rho={self.discount_rate:.4g}/s (gamma), scored at the problem's"
        )
        if self.fixed_horizon:
            return f"{self.iterations} backward sweeps (fixed horizon){rate}"

        if self.converged:
            return (
                f"converged in {self.iterations} sweeps (delta={self.delta:.3g}){rate}"
            )

        return (
            f"max_iterations={self.iterations} reached before tol={self.tol:g} "
            f"(delta={self.delta:.3g}){rate}"
        )


###############################################################################
# Internal machinery
###############################################################################


def grid_of(problem, grid, x_grid, u_grid, dt, solver="euler") -> StateSpaceGrid:
    """The grid passed in, or one built from the shapes and ``dt`` (never both)."""
    if grid is None:
        # A step problem ticks on its own: its grid needs no dt, and takes none
        require_no_period_on_ticks(problem, dt)
        native = isinstance(problem, StepPlanningProblem) and dt is None
        if x_grid is None or u_grid is None or (dt is None and not native):
            raise ValueError(
                "pass grid=StateSpaceGrid(...) or all of x_grid, u_grid, and dt"
            )

        return StateSpaceGrid(
            problem, x_grid_shape=x_grid, u_grid_shape=u_grid, dt=dt, solver=solver
        )

    if x_grid is not None or u_grid is not None or dt is not None:
        raise ValueError("pass either grid= or x_grid/u_grid/dt, not both")

    return grid


def grid_numbers_of(grid, gamma, out_of_bound_cost):
    """``(gamma, out_of_bound_cost, final_time)``: the given numbers, else the face's discount and failure cost; the face's horizon end."""
    face = grid.face

    # The face's discount per step (e^{−ρ dt}, or a step problem's γ), unless gamma is given
    if gamma is None:
        gamma = face.gamma if face.gamma < 1.0 else 1.0
    gamma = float(gamma)

    # The cost of leaving the grid, unless given: the problem's, else the face's cost bound
    if out_of_bound_cost is None:
        out_of_bound_cost = grid_failure_cost(grid, gamma)

    # The horizon's end (t0 + horizon, or a step problem's last tick), 0 without one
    end = face.horizon_end()
    final_time = 0.0 if end is None else float(end)

    return gamma, out_of_bound_cost, final_time


def require_dp_backend(backend):
    """Refuse an unknown backward-step engine; return it."""
    if backend not in (BACKEND_LOOP, BACKEND_NUMPY, BACKEND_JAX):
        raise ValueError(f"Unknown backend {backend!r}")

    return backend


def tool_rate_of(problem, gamma, dt):
    """
    The planner's own discount rate ``ρ = −ln γ / dt`` when ``gamma`` overrides a continuous problem's, else ``None``.

    ``γ = 1`` is ``ρ = 0``; ``γ = 0`` (one step counts) keeps the problem's
    rate, which an Euler step never reads inside its period.
    """
    if gamma is None or not isinstance(problem, PlanningProblem):
        return None
    gamma = float(gamma)
    if gamma <= 0.0:
        return None

    # γ = e^{−ρ dt}
    rho = -np.log(gamma) / dt if gamma < 1.0 else 0.0
    if rho == problem.discount_rate:
        return None

    return float(rho)


def grid_on_rate(grid, rate):
    """The grid itself, or the same grid on a copy of its problem at the rate ``rate`` (``None``: unchanged)."""
    if rate is None:
        return grid
    problem = grid.problem.replace(discount_rate=rate)

    return StateSpaceGrid(
        problem,
        x_grid_shape=grid.x_grid_shape,
        u_grid_shape=grid.u_grid_shape,
        dt=grid.dt,
        solver=grid.solver,
    )


def grid_failure_cost(grid, gamma):
    """
    The cost of an inadmissible pair at the discount ``gamma`` per step: the face's failure cost over the sweeps' horizon.

    With ``γ < 1`` the sweeps of a continuous (or discretized) problem run to
    convergence, so the face's sum is its geometric limit
    ``g_max dt / (1 − γ)``; with ``γ = 1``, and on a native step problem, it is
    over the problem's own horizon.
    """
    face = grid.face
    native = (
        isinstance(face.problem, StepPlanningProblem) and face.problem.origin is None
    )
    N = None if gamma < 1.0 and not native else face.n_steps

    return face.failure_cost_over(N, gamma)


def infeasible_penalty(failure_cost, x_next, t):
    """The penalty of one inadmissible pair: the scalar ``failure_cost``, or ``failure_cost(x_next, t)``."""
    return float(failure_cost(x_next, t)) if callable(failure_cost) else failure_cost


def require_tables(result):
    """Refuse a cleanup before any solve has produced the tables."""
    if result is None:
        raise ValueError("No solution has been computed yet")


def require_no_period_on_ticks(problem, dt):
    """Refuse a ``dt`` for a step problem's grid: it ticks on its own clock."""
    if isinstance(problem, StepPlanningProblem) and dt is not None:
        raise ValueError(
            "a step problem ticks on its own clock: omit dt (its period, if it has "
            "one, is the problem's dt)"
        )
