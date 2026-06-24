"""Value iteration basics — quadratic regulator and minimum-time control.

Run from the repo root::

    python examples/scripts/planning/value_iteration/demo_basics.py

Section A (default): the simplest end-to-end VI pipeline on a double integrator
(mirrors pyro's float-mass DP demo): discretize, solve the Bellman equation,
close the loop with a lookup-table controller.

Section B: same plant with :class:`~minilink.core.costs.TimeCost` and a
three-level input grid — the cost-to-go is time-to-go and the policy is
bang-bang (pyro ``dp_mass_min_time_optimal``).

Set ``MODE = "minimum_time"`` to run section B only.
"""

import numpy as np

from minilink.core.costs import QuadraticCost, TimeCost
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

MODE = "quadratic"  # "quadratic" | "minimum_time"


def run_quadratic_regulator():
    """Regulate a free point mass to the origin with a quadratic cost."""
    plant = DoubleIntegrator()
    plant.state.lower_bound[:] = [-10.0, -5.0]
    plant.state.upper_bound[:] = [10.0, 5.0]
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])

    cost = QuadraticCost.from_system(
        plant, xbar=np.zeros(2), R=np.array([[10.0]]), S=np.diag([10.0, 10.0])
    )
    problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

    inf = 300.0
    grid = StateSpaceGrid(problem, x_grid_shape=(101, 101), u_grid_shape=(41,), dt=0.05)
    planner = DynamicProgrammingPlanner(
        problem,
        grid=grid,
        options=DynamicProgrammingOptions(
            alpha=1.0, tol=0.5, max_iterations=1000, out_of_bound_cost=inf, verbose=True
        ),
    )
    result = planner.compute_solution()
    planner.clean_infeasible_set()

    plotting.plot_value(grid, result.J, vmax=inf)
    plotting.plot_value_3d(grid, np.clip(result.J, 0.0, inf))
    _, policy_ax = plotting.plot_policy(grid, result.pi)

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

    policy_ax.plot(trajectory.x[0], trajectory.x[1], "k-", linewidth=2, label="closed loop")
    policy_ax.plot(trajectory.x[0, 0], trajectory.x[1, 0], "go", label="start")
    policy_ax.plot(
        trajectory.x[0, -1], trajectory.x[1, -1], "r*", markersize=12, label="end"
    )
    policy_ax.legend(loc="upper right", fontsize=8)

    print("final state:", np.round(trajectory.x[:, -1], 3))


def run_minimum_time():
    """Drive the same plant to the origin in minimum time."""
    plant = DoubleIntegrator()
    plant.state.lower_bound[:] = [-2.0, -2.0]
    plant.state.upper_bound[:] = [2.0, 2.0]
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([1.0])

    cost = TimeCost.from_system(plant, eps=1e-3)
    problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

    inf = 10.0
    grid = StateSpaceGrid(problem, x_grid_shape=(201, 201), u_grid_shape=(3,), dt=0.05)
    planner = DynamicProgrammingPlanner(
        problem,
        grid=grid,
        options=DynamicProgrammingOptions(
            alpha=1.0, tol=1e-3, max_iterations=800, out_of_bound_cost=inf, verbose=True
        ),
    )
    result = planner.compute_solution()
    planner.clean_infeasible_set()

    plotting.plot_value(grid, result.J, vmax=inf)
    plotting.plot_value_3d(grid, np.clip(result.J, 0.0, inf))
    _, policy_ax = plotting.plot_policy(grid, result.pi)

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


if MODE == "minimum_time":
    run_minimum_time()
else:
    run_quadratic_regulator()
