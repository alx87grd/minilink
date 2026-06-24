"""Minimum-time control by dynamic-programming value iteration.

Run from the repo root::

    python examples/scripts/planning/demo_value_iteration_minimum_time.py

Mirrors pyro's ``dp_mass_min_time_optimal`` demo on a free mass. The running
cost is one unit per unit time until the target (:class:`~minilink.core.costs.TimeCost`),
so the cost-to-go *is* the time-to-go and the optimal policy is bang-bang: full
``+u`` then full ``-u`` across a switching curve. Three input levels give the
action set ``{-u_max, 0, +u_max}``.
"""

import numpy as np

from minilink.core.costs import TimeCost
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

INF = 10.0  # out-of-bound penalty and cost-to-go color cap (pyro tcf.INF)

# Plant: a free point mass with a unit force limit (pyro FloatingSingleMass).
plant = DoubleIntegrator()
plant.state.lower_bound[:] = [-2.0, -2.0]
plant.state.upper_bound[:] = [2.0, 2.0]
plant.inputs["u"].lower_bound = np.array([-1.0])
plant.inputs["u"].upper_bound = np.array([1.0])

# Cost: minimum time to the origin.
cost = TimeCost.from_system(plant, eps=1e-3)
problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

# Discretize (3 input levels -> {-1, 0, +1}) and solve with no discount.
grid = StateSpaceGrid(problem, x_grid_shape=(201, 201), u_grid_shape=(3,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        alpha=1.0, tol=1e-3, max_iterations=800, out_of_bound_cost=INF, verbose=True
    ),
)
result = planner.compute_solution()
planner.clean_infeasible_set()

# Time-to-go map (2-D and 3-D) and the bang-bang policy (note the switching curve).
plotting.plot_value(grid, result.J, vmax=INF)
plotting.plot_value_3d(grid, np.clip(result.J, 0.0, INF))
_, policy_ax = plotting.plot_policy(grid, result.pi)

# Close the loop and run to the goal in minimum time.
controller = result.controller()
diagram = DiagramSystem()
diagram.add_subsystem(controller, "controller")
diagram.add_subsystem(plant, "plant")
diagram.connect("plant", "x", "controller", "x")
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Minimum-time double integrator (value iteration)"

plant.x0 = np.array([1.2, 0.0])
trajectory = diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory(trajectory)

policy_ax.plot(trajectory.x[0], trajectory.x[1], "k-", linewidth=2, label="closed loop")
policy_ax.plot(trajectory.x[0, 0], trajectory.x[1, 0], "go", label="start")
policy_ax.legend(loc="upper right", fontsize=8)

print("predicted time-to-go from start:", round(result.value_at([1.2, 0.0]), 2), "s")
print("final state:", np.round(trajectory.x[:, -1], 3))
