"""Policy evaluation: the cost-to-go of a fixed control law on a discretized state space."""

import numpy as np

from minilink.core.feedback import block_roles
from minilink.planning.evaluation import policy_of
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    infeasible_penalty,
)
from minilink.planning.policy_synthesis.plotting import cost_scale_limit
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
    :class:`~minilink.planning.policy_synthesis.discretizer.StateSpaceGrid` and
    interpolation as :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
    and returns the policy's cost-to-go field ``J``.

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid
        Discretization of the state and input spaces.
    policy : callable, System or PlanningSolution
        State-feedback law ``u = policy(x)``, a solution (its policy), or a
        controller block that reads the state on an input named ``x`` (or
        ``y``) and commands ``u`` (:func:`minilink.core.feedback.block_roles`):
        the grid state feeds that port, the reference stays pinned at its
        nominal value, and the control port supplies ``u``.
    options : DynamicProgrammingOptions, optional
        Shared workflow options (``alpha``, ``tol``, ``interpolation``, ...).
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        grid,
        policy,
        options: DynamicProgrammingOptions | None = None,
    ) -> None:
        require_problem_cost(problem)

        # The problem and its discretization
        self.problem = problem
        self.grid = grid

        # The fixed law u = π(x): a callable, a solution's policy, or a controller block
        policy = policy_of(policy)
        self.policy = policy if callable(policy) else policy_from_block(policy)

        # Workflow options, shared with the dynamic-programming planner
        self.options = DynamicProgrammingOptions() if options is None else options

        # Memory
        self.last_J = None  # The latest solve's cost-to-go

    def solve(self) -> np.ndarray:
        """Iterate the fixed-policy Bellman update to tolerance and return ``J``."""
        grid = self.grid
        opt = self.options
        tf = opt.final_time
        alpha = opt.alpha
        method = opt.interpolation
        tol = opt.tol
        max_iterations = opt.max_iterations

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
            # J(x) = g(x, π(x), t) dt + α J_next(x + f(x, π(x), t) dt)
            J_next = J
            J_arrival = grid.interpolate(J_next, x_next, method)
            J = G + alpha * J_arrival

            # Largest change of the cost-to-go: the convergence measure
            delta = float(np.max(np.abs(J - J_next)))

        self.last_J = J

        return J

    def value_at(self, x) -> float:
        """Interpolate the policy's cost-to-go at an arbitrary state ``x``."""
        require_solved(self.last_J, "value_at")

        return float(self.grid.interpolate(self.last_J, np.atleast_2d(x))[0])

    def plot_cost2go(self, **kwargs):
        """Draw the policy's cost-to-go field; options as in :meth:`StateSpaceGrid.plot_value`."""
        require_solved(self.last_J, "plot_cost2go")

        kwargs.setdefault("vmax", cost_scale_limit(self.options.out_of_bound_cost))

        return self.grid.plot_value(self.last_J, **kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def policy_transition(self, t):
        """Per-state successor and running cost under the fixed policy."""
        grid = self.grid
        policy = self.policy
        f = self.problem.sys.f
        g = self.problem.cost.g
        X = self.problem.X
        U = self.problem.U
        sys_params = self.problem.params.system
        cost_params = self.problem.params.cost
        set_params = self.problem.params.sets
        dt = grid.dt
        N = grid.nodes_n  # Number of nodes
        n = grid.n  # State dimension
        INF = self.options.out_of_bound_cost  # A large finite penalty (pyro's cf.INF)

        successors = np.empty((N, n), dtype=float)
        G = np.empty(N, dtype=float)

        # For all state nodes
        for s in range(N):
            x = grid.states[s]

            # The policy's action at this node
            u = np.asarray(policy(x), dtype=float).reshape(-1)

            # Forward Euler step under the policy's action
            x_next = x + f(x, u, t, sys_params) * dt
            successors[s] = x_next

            # An admissible action, and a successor in X and on the grid (the
            # table's domain)
            valid = (
                U.contains(u, x, t, set_params)
                and X.contains(x_next, t, set_params)
                and grid.X.contains(x_next)
            )

            if valid:
                # Running cost over one step
                G[s] = float(g(x, u, t, cost_params)) * dt
            else:
                # The step leaves the admissible set: the penalty
                G[s] = infeasible_penalty(INF, x_next, t)

        return successors, G

    def terminal_cost(self, t):
        """The terminal cost ``h`` at time ``t`` of every grid node, by node id."""
        grid = self.grid
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        N = grid.nodes_n

        # Final cost of every state node
        J = np.array([float(h(grid.states[s], t, cost_params)) for s in range(N)])

        return J


###############################################################################
# Internal machinery
###############################################################################


def require_problem_cost(problem):
    """Refuse a problem without a cost: there is no cost-to-go to evaluate."""
    if problem.cost is None:
        raise ValueError("PolicyEvaluator requires problem.cost")


def policy_from_block(block):
    """Adapt a controller block into a ``u = policy(x)`` callable.

    The block is evaluated through its control-port compute at nominal
    internal state, with the grid state on the measurement port and every
    other input (reference) pinned at its port nominal.
    """
    roles = block_roles(block)
    require_feedback_roles(block, roles)

    output = block.outputs[roles.command]
    measurement = block.get_input_port_slice(roles.measurement)
    u_base = block.get_u_from_input_ports().astype(float)
    x_ctrl = np.asarray(block.x0, dtype=float)

    # The command of the block, the grid state on its measurement port
    def policy(x):
        u = u_base.copy()
        u[measurement] = np.asarray(x, dtype=float).reshape(-1)
        command = np.asarray(output.compute(x_ctrl, u, 0.0, None), dtype=float)
        command = command.reshape(-1)

        return command

    return policy


def require_feedback_roles(block, roles):
    """Refuse a block that does not read the state on ``x`` (or ``y``) and command ``u``."""
    if roles.measurement is None or roles.command is None:
        raise ValueError(
            "policy must be a callable u = policy(x) or a controller block that "
            "reads the state on an input named 'x' (or 'y') and commands 'u'; "
            f"{type(block).__name__} reads {list(block.inputs)} and gives "
            f"{list(block.outputs)}"
        )


def require_solved(J, verb):
    """Refuse a read of the cost-to-go before ``solve()`` has computed it."""
    if J is None:
        raise ValueError(f"call solve() before {verb}()")
