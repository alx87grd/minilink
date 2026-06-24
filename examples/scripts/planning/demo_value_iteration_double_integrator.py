"""Double-integrator regulation by dynamic-programming value iteration.

Run from the repo root::

    python examples/scripts/planning/demo_value_iteration_double_integrator.py

The simplest end-to-end value-iteration pipeline (mirrors pyro's float-mass DP
demo): discretize a free point mass, solve the Bellman equation for a quadratic
regulator cost, then close the loop with the lookup-table controller and watch
it drive a corner state back to the origin.
"""

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

# Plant: a free point mass (force input), bounded position and speed.
plant = DoubleIntegrator()
plant.state.lower_bound[:] = [-10.0, -5.0]
plant.state.upper_bound[:] = [10.0, 5.0]
plant.inputs["u"].lower_bound = np.array([-5.0])
plant.inputs["u"].upper_bound = np.array([5.0])

# Cost: regulate to the origin, penalize control and the terminal miss.
cost = QuadraticCost.from_system(
    plant, xbar=np.zeros(2), R=np.array([[10.0]]), S=np.diag([10.0, 10.0])
)
problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

INF = 300.0  # out-of-bound penalty and cost-to-go color cap (pyro qcf.INF)

# Discretize and solve the Bellman equation backward.
grid = StateSpaceGrid(problem, x_grid_shape=(101, 101), u_grid_shape=(41,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        alpha=1.0, tol=0.5, max_iterations=1000, out_of_bound_cost=INF, verbose=True
    ),
)
result = planner.compute_solution()
planner.clean_infeasible_set()

# Cost-to-go (2-D and 3-D) and policy maps.
plotting.plot_value(grid, result.J, vmax=INF)
plotting.plot_value_3d(grid, np.clip(result.J, 0.0, INF))
_, policy_ax = plotting.plot_policy(grid, result.pi)

# Close the loop with the lookup-table controller and simulate from a corner.
controller = result.controller()
diagram = DiagramSystem()
diagram.add_subsystem(controller, "controller")
diagram.add_subsystem(plant, "plant")
diagram.connect("plant", "x", "controller", "x")
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Double integrator (value iteration)"

plant.x0 = np.array([5.0, 3.0])
trajectory = diagram.compute_trajectory(tf=20.0)
diagram.plot_trajectory(trajectory)

# Overlay the closed-loop path on the policy map (phase plane).
policy_ax.plot(trajectory.x[0], trajectory.x[1], "k-", linewidth=2, label="closed loop")
policy_ax.plot(trajectory.x[0, 0], trajectory.x[1, 0], "go", label="start")
policy_ax.plot(
    trajectory.x[0, -1], trajectory.x[1, -1], "r*", markersize=12, label="end"
)
policy_ax.legend(loc="upper right", fontsize=8)

print("final state:", np.round(trajectory.x[:, -1], 3))
