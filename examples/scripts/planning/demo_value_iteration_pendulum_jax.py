"""High-resolution pendulum swing-up by JAX-accelerated value iteration.

Run from the repo root::

    python examples/scripts/planning/demo_value_iteration_pendulum_jax.py

Same problem as :mod:`demo_value_iteration_pendulum` (pyro's iconic
``pendulum_optimal_swingup``) but on a much finer grid (``501 x 501 x 21``,
~5.3 M state-action pairs) — the resolution pyro shipped as its showcase figure.
The NumPy lookup-table backend takes minutes per backward sweep at this scale;
JAX runs the entire convergence loop as one jitted ``lax.while_loop`` with
``map_coordinates`` interpolation, cutting per-sweep cost ~30x and making the
problem comfortable on a laptop.

Prints a wall-clock comparison of NumPy vs JAX on a short solve at the end.
"""

import time

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

# Plant: pyro's pendulum_optimal_swingup defaults.
pendulum = Pendulum()
pendulum.state.lower_bound = np.array([-10.0, -10.0])
pendulum.state.upper_bound = np.array([10.0, 10.0])
pendulum.inputs["u"].lower_bound = np.array([-5.0])
pendulum.inputs["u"].upper_bound = np.array([5.0])
pendulum.x0 = np.array([-0.1, 0.0])

# Cost: regulate to the upright (pyro Q=I, R=1, S=diag(10, 10)).
cost = QuadraticCost.from_system(
    pendulum, xbar=UPRIGHT, Q=np.eye(2), R=np.array([[1.0]]), S=np.diag([10.0, 10.0])
)
problem = PlanningProblem(pendulum, x_goal=UPRIGHT, cost=cost)

# High-resolution discretization: 251K nodes, 21 actions, ~5.3 M pairs.
print("Building 501x501x21 lookup table (one-time, NumPy)...")
t0 = time.perf_counter()
grid = StateSpaceGrid(problem, x_grid_shape=(501, 501), u_grid_shape=(21,), dt=0.05)
print(f"  build: {time.perf_counter() - t0:.1f} s")

# Solve the Bellman equation backward on the JAX backend.
print("\nSolving with JAX (jitted lax.while_loop + map_coordinates)...")
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        backend="jax", alpha=1.0, tol=0.1, max_iterations=2000, out_of_bound_cost=INF
    ),
)
t0 = time.perf_counter()
result = planner.compute_solution()
print(
    f"  jax solve: {time.perf_counter() - t0:.2f} s   iters={result.iterations}"
    f"   delta={result.delta:.3f}"
)
planner.clean_infeasible_set()

# Cost-to-go (2-D and 3-D) and the iconic policy plane.
plotting.plot_value(grid, result.J, vmax=INF)
plotting.plot_value_3d(grid, np.clip(result.J, 0.0, INF))
plotting.plot_policy(grid, result.pi)

# Close the loop with the lookup-table controller and swing up from rest.
controller = result.controller()
diagram = DiagramSystem()
diagram.add_subsystem(controller, "controller")
diagram.add_subsystem(pendulum, "plant")
diagram.connect("plant", "y", "controller", "x")
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Pendulum swing-up (value iteration, 501x501)"

trajectory = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(trajectory)

print("\nfinal angle error to upright:", round(abs(trajectory.x[0, -1] - UPRIGHT[0]), 3), "rad")

# Wall-clock comparison: 50 NumPy sweeps vs 50 JAX sweeps on the same grid.
print("\n=== NumPy vs JAX wall-clock (50 backward sweeps on this grid) ===")
np_planner = DynamicProgrammingPlanner(
    problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy", alpha=1.0, out_of_bound_cost=INF)
)
t0 = time.perf_counter()
np_planner.solve_steps(50)
numpy_solve_s = time.perf_counter() - t0
print(f"  numpy: {numpy_solve_s:6.2f} s")

# Reuse the warm JAX planner so the report is compile-free
t0 = time.perf_counter()
planner.solve_steps(50)
jax_solve_s = time.perf_counter() - t0
print(f"  jax  : {jax_solve_s:6.2f} s   ({numpy_solve_s / jax_solve_s:.1f}x faster)")
