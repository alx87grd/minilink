"""
Cost-to-go of a fixed control law on a discretized state space.

:class:`PolicyEvaluator` solves the Bellman *expectation* equation for a given
policy: it is the dynamic-programming sweep with the action fixed by a control
law instead of minimized over the action set. It reuses the same
:class:`~minilink.planning.policy_synthesis.discretizer.StateSpaceGrid` and
interpolation as :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
and returns the policy's cost-to-go field ``J``.
"""

import numpy as np

from minilink.core.feedback import feedback_ports
from minilink.planning.policy_synthesis.dp import DynamicProgrammingOptions
from minilink.planning.problems import PlanningProblem

# Public API


class PolicyEvaluator:
    """
    Evaluate the cost-to-go of a fixed feedback policy.

    Parameters
    ----------
    problem : PlanningProblem
        Planning problem (system, sets, and cost). A cost is required.
    grid : StateSpaceGrid
        Discretization of the state and input spaces.
    policy : callable or System
        State-feedback law ``u = policy(x)``, or a controller block with a
        feedback declaration (``feedback_profile`` or explicit port-role
        attrs — see :func:`minilink.core.feedback.feedback_ports`): the grid
        state feeds the measurement port, the reference stays pinned at its
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
        if problem.cost is None:
            raise ValueError("PolicyEvaluator requires problem.cost")
        self.problem = problem
        self.grid = grid
        self.policy = policy if callable(policy) else _policy_from_block(policy)
        self.options = DynamicProgrammingOptions() if options is None else options
        self.last_J = None

    def solve(self) -> np.ndarray:
        """Iterate the fixed-policy Bellman update to tolerance and return ``J``."""
        grid = self.grid
        opt = self.options
        tf = opt.final_time

        x_next, G = self._policy_transition(tf)

        J = self._terminal_cost(tf)
        delta = np.inf
        k = 0
        while k < opt.max_iterations and delta > opt.tol:
            k += 1
            J_next = J

            arrival = grid.interpolate(J_next, x_next, opt.interpolation)
            J = G + opt.alpha * arrival

            delta = float(np.max(np.abs(J - J_next)))

        self.last_J = J
        return J

    # Internal machinery

    def _policy_transition(self, t):
        """Per-state successor and running cost under the fixed policy."""
        grid = self.grid
        N, n = grid.nodes_n, grid.n
        f = self.problem.sys.f
        g = self.problem.cost.g
        X, U = self.problem.X, self.problem.U
        sys_params = self.problem.params.system
        cost_params = self.problem.params.cost
        set_params = self.problem.params.sets
        dt = grid.dt

        out_of_bound_cost = self.options.out_of_bound_cost
        x_next = np.empty((N, n), dtype=float)
        G = np.empty(N, dtype=float)

        for s in range(N):
            x = grid.states[s]
            u = np.asarray(self.policy(x), dtype=float).reshape(-1)

            # forward Euler step under the policy's action
            xnext = x + f(x, u, t, sys_params) * dt

            x_next[s] = xnext
            valid = U.contains(u, x, t, set_params) and X.contains(xnext, t, set_params)
            G[s] = float(g(x, u, t, cost_params)) * dt if valid else out_of_bound_cost

        return x_next, G

    def _terminal_cost(self, t):
        grid = self.grid
        h = self.problem.cost.h
        cost_params = self.problem.params.cost
        return np.array(
            [float(h(grid.states[s], t, cost_params)) for s in range(grid.nodes_n)]
        )


def _policy_from_block(block):
    """Adapt a declared controller block into a ``u = policy(x)`` callable.

    The block is evaluated through its control-port compute at nominal
    internal state, with the grid state on the measurement port and every
    other input (reference) pinned at its port nominal.
    """
    roles = feedback_ports(block)
    if roles is None:
        raise ValueError(
            "policy must be a callable u = policy(x) or a controller block "
            "with a feedback declaration (feedback_profile or "
            f"measurement_port / control_port); got {type(block).__name__}"
        )
    output = block.outputs[roles.control]
    measurement = block.get_input_port_slice(roles.measurement)
    u_base = block.get_u_from_input_ports().astype(float)
    x_ctrl = np.asarray(block.x0, dtype=float)

    def policy(x):
        u = u_base.copy()
        u[measurement] = np.asarray(x, dtype=float).reshape(-1)
        return np.asarray(output.compute(x_ctrl, u, 0.0, None), dtype=float).reshape(-1)

    return policy
