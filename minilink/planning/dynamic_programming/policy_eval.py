"""Policy evaluation: the cost-to-go of a fixed control law on a discretized state space."""

import numpy as np

from minilink.planning.dynamic_programming.dp import (
    grid_numbers_of,
    infeasible_penalty,
    require_grid_on_sets,
)
from minilink.planning.dynamic_programming.plotting import cost_scale_limit
from minilink.planning.evaluator import controller_of
from minilink.planning.policies import policy_from_block
from minilink.planning.problems import PlanningProblem

###############################################################################
# Public API
###############################################################################


class PolicyEvaluator:
    """
    Evaluate the cost-to-go of a fixed feedback policy.

    Solves the Bellman *expectation* equation for a given policy: the
    dynamic-programming sweep with the action fixed by a control law instead
    of minimized over the action set. It reuses the same
    :class:`~minilink.planning.dynamic_programming.discretizer.StateSpaceGrid` and
    interpolation as :class:`~minilink.planning.dynamic_programming.dp.DynamicProgrammingPlanner`
    and returns the policy's cost-to-go field ``J``.

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid
        Discretization of the state and input spaces.
    policy : callable, System or PlanningSolution
        State-feedback law ``u = policy(x)``, a solution (its controller), or a
        controller block that reads the state on an input named ``x`` (or
        ``y``) and commands ``u`` (:func:`minilink.core.roles.block_roles`):
        the grid state feeds that port, the reference stays pinned at its
        nominal value, and the control port supplies ``u``.
    gamma, out_of_bound_cost : float, optional
        The discount per step and the failure cost, the planner's numbers
        (``**planner.params["method"]`` shares them); unset, the problem's
        discount and failure cost, as the planner reads them.
    tol, max_iterations, interpolation
        The stopping rules and the cost-to-go interpolation, as on
        :class:`~minilink.planning.dynamic_programming.dp.DynamicProgrammingPlanner`.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        grid,
        policy,
        gamma: float | None = None,
        out_of_bound_cost=None,
        tol: float = 0.1,
        max_iterations: int = 1000,
        interpolation: str = "linear",
    ) -> None:
        require_problem_cost(problem)
        require_grid_on_sets(problem, grid)

        # The problem and its discretization
        self.problem = problem
        self.grid = grid

        # The fixed law u = π(x): a callable, a solution's controller, or a controller block
        policy = controller_of(policy)
        self.policy = policy if callable(policy) else policy_from_block(policy)

        # The numbers, the problem's unless given, and the terminal time
        self.gamma, self.out_of_bound_cost, self.final_time = grid_numbers_of(
            grid, gamma, out_of_bound_cost
        )

        # The stopping rules and the interpolation
        self.tol = float(tol)
        self.max_iterations = int(max_iterations)
        self.interpolation = interpolation

        # Memory
        self.last_J = None  # The latest solve's cost-to-go

    def solve(self) -> np.ndarray:
        """Iterate the fixed-policy Bellman update to tolerance and return ``J``."""
        grid = self.grid
        tf = self.final_time
        gamma = self.gamma
        method = self.interpolation
        tol = self.tol
        max_iterations = self.max_iterations

        # Successor and running cost of every node under the policy's action
        x_next, G = self.policy_transition(tf)

        # Cost-to-go at the final time
        J = self.terminal_cost(tf)

        # Sweeps until the cost-to-go stops changing, at most max_iterations of them
        delta = np.inf
        k = 0

        while k < max_iterations and delta > tol:
            k = k + 1

            # Bellman expectation backup, the action fixed by the policy:
            # J(x_k) = c(x_k, π(x_k)) + γ J_next(x_{k+1}), the model's stage cost and step
            J_next = J
            J_arrival = grid.interpolate(J_next, x_next, method)
            J = G + gamma * J_arrival

            # Largest change of the cost-to-go: the convergence measure
            delta = float(np.max(np.abs(J - J_next)))

        self.last_J = J

        return J

    def value_at(self, x) -> float:
        """Interpolate the policy's cost-to-go at an arbitrary state ``x``, by the settings' interpolation."""
        require_solved(self.last_J, "value_at")

        method = self.interpolation
        J = self.grid.interpolate(self.last_J, np.atleast_2d(x), method)

        return float(J[0])

    def plot_cost_to_go(self, **kwargs):
        """Draw the policy's cost-to-go field; options as in :meth:`StateSpaceGrid.plot_value`."""
        require_solved(self.last_J, "plot_cost_to_go")

        kwargs.setdefault("vmax", cost_scale_limit(self.out_of_bound_cost))

        return self.grid.plot_value(self.last_J, **kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def policy_transition(self, t):
        """Per-state successor and running cost under the fixed policy."""
        grid = self.grid
        policy = self.policy
        X = self.problem.X
        U = self.problem.U
        tree = grid.model.params
        sys_params = tree["plant"]
        N = grid.nodes_n  # Number of nodes
        n = grid.n  # State dimension
        INF = self.out_of_bound_cost  # The finite failure cost (pyro's cf.INF)

        successors = np.empty((N, n), dtype=float)
        G = np.empty(N, dtype=float)

        # The stage cost of the period at t, its laws built once for every node
        stage_cost = grid.stage_cost_law(t, tree)

        # For all state nodes
        for s in range(N):
            x = grid.states[s]

            # The policy's action at this node
            u = np.asarray(policy(x), dtype=float).reshape(-1)

            # x_{k+1} = Φ(x_k, π(x_k), t), the grid's step model under the policy
            x_next = grid.successor(x, u, t, sys_params)
            successors[s] = x_next

            # An admissible action, and a successor in X and on the grid (the
            # table's domain)
            valid = (
                U.contains(u, x, t)
                and X.contains(x_next, t)
                and grid.X.contains(x_next)
            )

            if valid:
                # The stage cost of the period: c = ∫₀^dt e^{−ρτ} g dτ on the model
                G[s] = float(stage_cost(x, u))
            else:
                # The step leaves the admissible set: the penalty
                G[s] = infeasible_penalty(INF, x_next, t)

        return successors, G

    def terminal_cost(self, t):
        """The terminal cost ``h`` at time ``t`` of every grid node, by node id."""
        grid = self.grid
        model = grid.model
        tree = model.params
        N = grid.nodes_n

        # Final cost of every state node: the model's h(x, t)
        J = np.array([float(model.terminal(grid.states[s], t, tree)) for s in range(N)])

        return J


###############################################################################
# Internal machinery
###############################################################################


def require_problem_cost(problem):
    """Refuse a problem without a cost: there is no cost-to-go to evaluate."""
    if problem.cost is None:
        raise ValueError("PolicyEvaluator requires problem.cost")


def require_solved(J, verb):
    """Refuse a read of the cost-to-go before ``solve()`` has computed it."""
    if J is None:
        raise ValueError(f"call solve() before {verb}()")
