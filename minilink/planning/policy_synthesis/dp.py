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
"""

from dataclasses import dataclass

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY
from minilink.planning.planner import Planner
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.problems import PlanningProblem

# Public API


@dataclass
class DynamicProgrammingOptions:
    """
    Workflow options for :class:`DynamicProgrammingPlanner`.

    Parameters
    ----------
    backend : {"numpy", "jax"}
        Evaluation backend. ``"jax"`` is reserved for a later slice.
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
        Print a one-line convergence report per sweep.
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
        if self.options.backend == BACKEND_JAX:
            raise NotImplementedError("The JAX backend lands in a later slice")
        self._G = None  # running-cost table, cached when the grid is precomputed

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
        grid = self.grid
        opt = self.options
        tf = opt.final_time

        J = self._terminal_cost(tf)
        pi = np.zeros(grid.nodes_n, dtype=int)

        history = [(tf, J.copy(), pi.copy())] if opt.record_history else None

        delta = np.inf
        k = 0
        while k < max_iterations and (not stop_on_tol or delta > opt.tol):
            k += 1
            t = tf - k * grid.dt
            J_next = J
            J, pi = self._backward_step(J_next, t)
            delta = float(np.max(np.abs(J - J_next)))
            if opt.verbose:
                print(f"DP step {k:4d}  t={t:7.3f}  Jmax={J.max():.3g}  dJ={delta:.3g}")
            if history is not None:
                history.append((t, J.copy(), pi.copy()))

        result = DynamicProgrammingResult(
            grid=grid, J=J, pi=pi, iterations=k, delta=delta, history=history
        )
        return self._store_result(result)

    def _backward_step(self, J, t):
        """One Bellman backup: arrival cost-to-go, then greedy minimisation."""
        grid = self.grid
        n, N, A = grid.n, grid.nodes_n, grid.actions_n
        alpha = self.options.alpha

        x_next, action_ok, x_next_ok = grid.transition(t)
        G = self._running_cost(action_ok, x_next_ok, t)

        J_next = grid.interpolate(J, x_next.reshape(-1, n), self.options.interpolation)
        Q = G + alpha * J_next.reshape(N, A)

        return Q.min(axis=1), Q.argmin(axis=1)

    def _terminal_cost(self, t):
        grid = self.grid
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        return np.array(
            [float(h(grid.states[s], t, cost_params)) for s in range(grid.nodes_n)]
        )

    def _running_cost(self, action_ok, x_next_ok, t):
        if self.grid.precomputed and self._G is not None:
            return self._G

        grid = self.grid
        g = self.problem.cost.g
        cost_params = self.problem.params.cost
        dt = grid.dt
        N, A = grid.nodes_n, grid.actions_n

        G = np.empty((N, A), dtype=float)
        for a in range(A):
            u = grid.inputs[a]
            for s in range(N):
                G[s, a] = float(g(grid.states[s], u, t, cost_params)) * dt

        G[(~action_ok) | (~x_next_ok)] = self.options.out_of_bound_cost

        if self.grid.precomputed:
            self._G = G
        return G
