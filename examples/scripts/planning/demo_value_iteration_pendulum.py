"""Pendulum swing-up by dynamic-programming value iteration.

Run from the repo root::

    python examples/scripts/planning/demo_value_iteration_pendulum.py

Mirrors pyro's ``pendulum_optimal_swingup`` demo. Value iteration on a
discretized state space finds a global cost-to-go ``J`` and a feedback policy
over the whole ``(theta, dtheta)`` plane. With a torque limit below the gravity
torque the optimal policy pumps energy, swinging back and forth before reaching
the upright. The lookup-table controller then closes the loop on the continuous
pendulum.
"""

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

INF = 500.0  # out-of-bound penalty and cost-to-go color cap (pyro qcf.INF)
UPRIGHT = np.array([-np.pi, 0.0])  # pyro target xbar = [-3.14, 0]

# Plant: a single pendulum with a limited torque (must pump to swing up).
pendulum = Pendulum()
pendulum.state.lower_bound = np.array([-10.0, -10.0])
pendulum.state.upper_bound = np.array([10.0, 10.0])
pendulum.inputs["u"].lower_bound = np.array([-5.0])
pendulum.inputs["u"].upper_bound = np.array([5.0])
pendulum.x0 = np.array([-0.1, 0.0])  # near the downward rest, off the symmetry axis

# Cost: regulate to the upright equilibrium (pyro Q=I, R=1, S=diag(10, 10)).
cost = QuadraticCost.from_system(
    pendulum, xbar=UPRIGHT, Q=np.eye(2), R=np.array([[1.0]]), S=np.diag([10.0, 10.0])
)
problem = PlanningProblem(pendulum, x_goal=UPRIGHT, cost=cost)

# Discretize and solve the Bellman equation backward.
grid = StateSpaceGrid(problem, x_grid_shape=(201, 201), u_grid_shape=(21,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        alpha=1.0, tol=0.1, max_iterations=2000, out_of_bound_cost=INF, verbose=True
    ),
)
result = planner.compute_solution()
planner.clean_infeasible_set()

# Cost-to-go (2-D and 3-D) and policy over the state plane.
plotting.plot_value(grid, result.J, vmax=INF)
plotting.plot_value_3d(grid, np.clip(result.J, 0.0, INF))
plotting.plot_policy(grid, result.pi)

# Close the loop with the lookup-table controller and swing up from rest.
controller = result.controller()
diagram = DiagramSystem()
diagram.add_subsystem(controller, "controller")
diagram.add_subsystem(pendulum, "plant")
diagram.connect("plant", "y", "controller", "x")  # pendulum y = [theta, dtheta]
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Pendulum swing-up (value iteration)"

trajectory = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(trajectory)
diagram.animate()

angle_error = abs(trajectory.x[0, -1] - UPRIGHT[0])
print(f"\nfinal angle error to upright: {angle_error:.3f} rad")
