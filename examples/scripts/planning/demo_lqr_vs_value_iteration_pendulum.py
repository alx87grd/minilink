"""LQR vs dynamic-programming value iteration on a pendulum.

Run from the repo root::

    python examples/scripts/planning/demo_lqr_vs_value_iteration_pendulum.py

Mirrors pyro's ``lqr_vs_valueiteration`` demo: both controllers come from the
*same* quadratic cost and are wired directly to the plant (no saturation). The
contrast is local vs global:

- **LQR** linearizes about the upright and gives a linear feedback plane —
  optimal near the target, unbounded far from it.
- **Value iteration** solves the global Bellman equation over the discretized
  torque range, so its policy is nonlinear and respects the torque limit.

Compare the two control-law maps and the two closed-loop trajectories.
"""

import numpy as np

from minilink.control.lqr import lqr_at_operating_point
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

TORQUE = 5.0
UPRIGHT = np.array([-np.pi, 0.0])  # pyro target xbar
Q = np.eye(2)
R = np.eye(1)


def pendulum():
    sys = Pendulum()
    # pyro SinglePendulum defaults: x in [-2*pi, 2*pi]^2, u in [-5, 5]
    sys.state.lower_bound = np.array([-2.0 * np.pi, -2.0 * np.pi])
    sys.state.upper_bound = np.array([2.0 * np.pi, 2.0 * np.pi])
    sys.inputs["u"].lower_bound = np.array([-TORQUE])
    sys.inputs["u"].upper_bound = np.array([TORQUE])
    return sys


# Value-iteration policy from the quadratic cost (global, on the torque grid).
vi_plant = pendulum()
cost = QuadraticCost.from_system(vi_plant, xbar=UPRIGHT, Q=Q, R=R)
problem = PlanningProblem(vi_plant, x_goal=UPRIGHT, cost=cost)
grid = StateSpaceGrid(problem, x_grid_shape=(101, 101), u_grid_shape=(11,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(alpha=1.0, tol=0.1, max_iterations=2000),
)
result = planner.compute_solution()
planner.clean_infeasible_set()

# LQR feedback gain about the upright from the same cost (local, unbounded).
lqr = lqr_at_operating_point(pendulum(), UPRIGHT, Q, R)
K = lqr.params["K"][0]
ubar = lqr.params["ubar"][0]
lqr_law = ubar - (grid.states - UPRIGHT) @ K

# Control-law maps: global VI policy vs the linear LQR plane.
plotting.plot_policy(grid, result.pi)
plotting.plot_value(
    grid, lqr_law, vmin=-TORQUE, vmax=TORQUE, cmap="bwr", title="LQR control law"
)


def closed_loop(controller, x0):
    """Simulate ``controller >> pendulum`` from the downward rest ``x0``."""
    plant = pendulum()
    plant.x0 = np.array(x0)
    diagram = DiagramSystem()
    diagram.add_subsystem(controller, "controller")
    diagram.add_subsystem(plant, "plant")
    diagram.connect("plant", "y", "controller", "x")  # pendulum y = [theta, dtheta]
    diagram.connect("controller", "u", "plant", "u")
    return diagram, diagram.compute_trajectory(tf=10.0)


# Both controllers from the downward rest (off the symmetry axis for VI).
vi_diagram, vi_traj = closed_loop(result.controller(), [-0.1, 0.0])
lqr_diagram, lqr_traj = closed_loop(lqr, [-0.1, 0.0])
vi_diagram.plot_trajectory(vi_traj)
lqr_diagram.plot_trajectory(lqr_traj)

print("VI  | final angle error:", round(abs(vi_traj.x[0, -1] - UPRIGHT[0]), 3), "rad")
print("LQR | final angle error:", round(abs(lqr_traj.x[0, -1] - UPRIGHT[0]), 3), "rad")
