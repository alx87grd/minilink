"""Double pendulum swing-up by value iteration — 4-D state, JAX backend.

Run from the repo root::

    python examples/scripts/planning/value_iteration/demo_double_pendulum_jax.py

Mirrors pyro ``double_pendulum_optimal_swingup.py``: asymmetric state bounds,
``(51, 41, 51, 41)`` state grid with ``(5, 5)`` torques, quadratic cost to
the upright ``[0, 0, 0, 0]``, and swing-up from the hanging configuration
``[-π, 1, 0, 0]``. At full scale this is ~4.4M nodes and ~109M state–action
pairs — a stress test for the JAX lookup-table pipeline (grid transition, ``G``,
and jitted Bellman sweeps).

Set ``RESOLUTION = "fast"`` for a smaller grid when iterating locally (~15M
pairs), or ``"high"`` for a finer state/control mesh and more Bellman sweeps.
"""

import time

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

RESOLUTION = "high"  # "fast" | "pyro" | "high"
# RESOLUTION = "fast"  # "fast" | "pyro" | "high"
# RESOLUTION = "pyro"  # "fast" | "pyro" | "high"

INF = 1000.0
GOAL = np.zeros(4)
HANGING = np.array([-np.pi, 1.0, 0.0, 0.0])

FAST_GRID = (31, 25, 31, 25)
PYRO_GRID = (51, 41, 51, 41)
HIGH_GRID = (71, 61, 71, 61)

PYRO_U = (5, 5)
HIGH_U = (7, 7)

if RESOLUTION == "fast":
    x_shape = FAST_GRID
    u_shape = PYRO_U
    n_steps = 30
elif RESOLUTION == "high":
    x_shape = HIGH_GRID
    u_shape = HIGH_U
    n_steps = 100
else:
    x_shape = PYRO_GRID
    u_shape = PYRO_U
    n_steps = 50

plant = DoublePendulum()
plant.state.lower_bound = np.array([-5.0, -1.5, -4.0, -4.0])
plant.state.upper_bound = np.array([0.5, 4.0, 5.5, 7.0])
plant.inputs["u"].lower_bound = np.array([-12.0, -12.0])
plant.inputs["u"].upper_bound = np.array([12.0, 12.0])
plant.x0 = HANGING.copy()

Q = np.diag([1.0, 0.5, 0.1, 0.05])
R = np.diag([0.05, 0.05])
cost = QuadraticCost.from_system(plant, xbar=GOAL, Q=Q, R=R, S=Q)
problem = PlanningProblem(plant, x_goal=GOAL, cost=cost)

nodes = int(np.prod(x_shape))
actions = int(np.prod(u_shape))
print(
    f"\nresolution={RESOLUTION}  grid {x_shape} x {u_shape}  dt=0.1  "
    f"nodes={nodes:,}  pairs={nodes * actions:,}"
)

t_plan = time.perf_counter()
grid = StateSpaceGrid(
    problem,
    x_grid_shape=x_shape,
    u_grid_shape=u_shape,
    dt=0.1,
    precompute=False,
    verbose=True,
)
t_mesh = time.perf_counter()
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        backend="jax",
        alpha=1.0,
        max_iterations=n_steps,
        out_of_bound_cost=INF,
        verbose=True,
    ),
)
t_xnext = time.perf_counter()
result = planner.solve_steps(n_steps)
planner.clean_infeasible_set()
t_done = time.perf_counter()

print("\ntiming summary:")
print(f"  mesh setup:       {t_mesh - t_plan:5.2f} s")
print(f"  x_next table:     {t_xnext - t_mesh:5.2f} s  (planner init)")
print(f"  G, J0, Bellman:   {t_done - t_xnext:5.2f} s  (each step timed above)")
print(f"  total pipeline:   {t_done - t_plan:5.2f} s")

plotting.plot_value(
    grid,
    result.J,
    axes=(0, 1),
    anchor=GOAL,
    vmax=INF,
    title="Cost-to-go (θ1, θ2) at goal velocities",
)
plotting.plot_value(
    grid,
    result.J,
    axes=(0, 2),
    anchor=GOAL,
    vmax=INF,
    title="Cost-to-go (θ1, dθ1) at goal θ2, dθ2",
)
plotting.plot_value(
    grid,
    result.J,
    axes=(1, 3),
    anchor=GOAL,
    vmax=INF,
    title="Cost-to-go (θ2, dθ2) at goal θ1, dθ1",
)
plotting.plot_policy(grid, result.pi, axis=0, axes=(0, 1), anchor=GOAL)
plotting.plot_policy(grid, result.pi, axis=1, axes=(0, 1), anchor=GOAL)

controller = result.controller()
diagram = DiagramSystem()
diagram.add_subsystem(controller, "controller")
diagram.add_subsystem(plant, "plant")
diagram.connect("plant", "y", "controller", "x")
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Double pendulum swing-up (value iteration, JAX)"

trajectory = diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory(trajectory)
diagram.animate()

final = trajectory.x[:, -1]
print("\npredicted cost-to-go from start:", round(result.value_at(HANGING), 2))
print("final state:", np.round(final, 3))
print("goal error ||x - x*||:", round(float(np.linalg.norm(final - GOAL)), 3))
