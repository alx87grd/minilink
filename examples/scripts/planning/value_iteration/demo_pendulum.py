"""Pendulum swing-up by value iteration, with an LQR comparison.

Run from the repo root::

    python examples/scripts/planning/value_iteration/demo_pendulum.py

Part 1 (default): iconic swing-up via dynamic programming on the ``(theta, dtheta)``
plane (pyro ``pendulum_optimal_swingup``). The lookup-table controller closes
the loop on the continuous pendulum.

Part 2: side-by-side LQR vs value iteration on the *same* quadratic cost (pyro
``lqr_vs_valueiteration``) — local linear feedback vs global nonlinear policy.

Set ``MODE = "lqr_comparison"`` to run part 2 only.

For JAX vs NumPy timing at scale, run ``python benchmarks/run_dp_backends.py``.
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

MODE = "swingup"  # "swingup" | "lqr_comparison"

INF = 500.0
UPRIGHT = np.array([-np.pi, 0.0])
TORQUE = 5.0
Q = np.eye(2)
R = np.eye(1)


def make_pendulum(*, bounds=(-10.0, 10.0)):
    """Pendulum with bounded state and torque."""
    sys = Pendulum()
    lo, hi = bounds
    sys.state.lower_bound = np.array([lo, lo])
    sys.state.upper_bound = np.array([hi, hi])
    sys.inputs["u"].lower_bound = np.array([-TORQUE])
    sys.inputs["u"].upper_bound = np.array([TORQUE])
    return sys


def run_swingup():
    """Global swing-up policy from value iteration."""
    pendulum = make_pendulum()
    pendulum.x0 = np.array([-0.1, 0.0])

    cost = QuadraticCost.from_system(
        pendulum,
        xbar=UPRIGHT,
        Q=np.eye(2),
        R=np.array([[1.0]]),
        S=np.diag([10.0, 10.0]),
    )
    problem = PlanningProblem(pendulum, x_goal=UPRIGHT, cost=cost)

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

    plotting.plot_value(grid, result.J, vmax=INF)
    plotting.plot_value_3d(grid, np.clip(result.J, 0.0, INF))
    plotting.plot_policy(grid, result.pi)

    controller = result.controller()
    diagram = DiagramSystem()
    diagram.add_subsystem(controller, "controller")
    diagram.add_subsystem(pendulum, "plant")
    diagram.connect("plant", "y", "controller", "x")
    diagram.connect("controller", "u", "plant", "u")
    diagram.name = "Pendulum swing-up (value iteration)"

    trajectory = diagram.compute_trajectory(tf=10.0)
    diagram.plot_trajectory(trajectory)

    diagram.camera_scale = 2.0
    diagram.animate()

    angle_error = abs(trajectory.x[0, -1] - UPRIGHT[0])
    print(f"\nfinal angle error to upright: {angle_error:.3f} rad")


def run_lqr_comparison():
    """Compare global VI policy with local LQR on the same quadratic cost."""
    vi_plant = make_pendulum(bounds=(-2.0 * np.pi, 2.0 * np.pi))
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

    lqr = lqr_at_operating_point(
        make_pendulum(bounds=(-2.0 * np.pi, 2.0 * np.pi)), UPRIGHT, Q, R
    )
    K = lqr.params["K"][0]
    ubar = lqr.params["ubar"][0]
    lqr_law = ubar - (grid.states - UPRIGHT) @ K

    plotting.plot_policy(grid, result.pi)
    plotting.plot_value(
        grid, lqr_law, vmin=-TORQUE, vmax=TORQUE, cmap="bwr", title="LQR control law"
    )

    def closed_loop(controller, x0):
        plant = make_pendulum(bounds=(-2.0 * np.pi, 2.0 * np.pi))
        plant.x0 = np.array(x0)
        diagram = DiagramSystem()
        diagram.add_subsystem(controller, "controller")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "y", "controller", "x")
        diagram.connect("controller", "u", "plant", "u")
        return diagram, diagram.compute_trajectory(tf=10.0)

    vi_diagram, vi_traj = closed_loop(result.controller(), [-0.1, 0.0])
    lqr_diagram, lqr_traj = closed_loop(lqr, [-0.1, 0.0])
    vi_diagram.plot_trajectory(vi_traj)
    lqr_diagram.plot_trajectory(lqr_traj)

    print(
        "VI  | final angle error:", round(abs(vi_traj.x[0, -1] - UPRIGHT[0]), 3), "rad"
    )
    print(
        "LQR | final angle error:", round(abs(lqr_traj.x[0, -1] - UPRIGHT[0]), 3), "rad"
    )


if MODE == "lqr_comparison":
    run_lqr_comparison()
else:
    run_swingup()
