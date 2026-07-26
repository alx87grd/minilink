"""One-shot TrajOpt with a kinematic bicycle and a scene-pipeline obstacle cost.

Run from repo root::

    python examples/projects/car_trajopt/car_trajopt_kinematic_obstacle_scene.py

Same obstacle-avoidance workflow as
``car_trajopt_obstacle_scene.py``, but on
:class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleAcc`
(state ``[x, y, theta, v, delta]``, inputs ``[a_x, delta_dot]``).
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.dynamics.catalog.vehicles.jax_vehicles import BicycleAcc
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, point_probe
from minilink.planning.spatial.grid import sample_field_costs
from minilink.planning.spatial.plotting import plot_cost_field
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

TF = 4.0
N_STEPS = 20
U_0 = 5.0
U_TARGET = 10.0
Y_START = 0.0
Y_GOAL = 0.0
HEADING_TARGET = 0.0
DELTA_MAX = 0.55
V_MAX = 12.0
SPEED_DOT_MAX = 3.0
STEERING_DOT_MAX = 2.0

OBSTACLE_CENTER = (12.0, 0.2)
OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.2
OBSTACLE_REPULSION_WEIGHT = 1000.0
OBSTACLE_REPULSION_EPS = 1.0
PLOT_BOUNDS = ((-2.0, U_TARGET * TF + 2.0), (-3.5, 2.0))

configure_jax(enable_x64=True)

sys = BicycleAcc()
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
sys.state.lower_bound[3] = 0.0
sys.state.upper_bound[3] = V_MAX
sys.state.lower_bound[4] = -DELTA_MAX
sys.state.upper_bound[4] = DELTA_MAX
sys.inputs["u"].lower_bound = np.array([-SPEED_DOT_MAX, -STEERING_DOT_MAX])
sys.inputs["u"].upper_bound = np.array([SPEED_DOT_MAX, STEERING_DOT_MAX])

x_start = np.array([0.0, Y_START, 0.0, U_0, 0.0])
x_ref = np.array([U_TARGET * TF, Y_GOAL, HEADING_TARGET, U_TARGET, 0.0])
ubar = np.array([0.0, 0.0])

Q = np.diag([0.0, 10.0, 1.0, 0.1, 100.0])
R = np.diag([1.0, 10.0])
S = np.diag([1.0, 10.0, 100.0, 10.0, 100.0])

tracking_cost = QuadraticCost.from_system(
    sys,
    Q=Q,
    R=R,
    S=S,
    xbar=x_ref,
    ubar=ubar,
)
scene = Scene(obstacles=[Sphere(OBSTACLE_CENTER, keepout_radius)])
obstacle_cost = scene.clearance_field(bind(sys, point_probe())).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = tracking_cost + obstacle_cost

problem = PlanningProblem(sys=sys, x_start=x_start, cost=cost, tf=TF)
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=N_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    solve_disp=True,
    optimizer_options={
        "maxiter": 500,
        "ftol": 1e-1,
    },
)

traj = planner.solve().trajectory
planner.plot_solution(signals=("x", "u"))
sys.traj = traj
sys.animate(traj, overlays=[scene.as_visualizer(color="tab:red", opacity=0.45)])


# Plot the obstacle cost field

cost_grid = sample_field_costs(
    [obstacle_cost],
    bounds=PLOT_BOUNDS,
    state_dim=sys.n,
    grid=(120, 120),
    u=np.zeros(sys.m),
)
_, ax = plot_cost_field(
    cost_grid,
    show=False,
    log_scale=True,
    title=(
        f"Obstacle cost map (w={OBSTACLE_REPULSION_WEIGHT}, "
        f"eps={OBSTACLE_REPULSION_EPS})"
    ),
)
scene.plot(show=False, ax=ax, bounds=PLOT_BOUNDS, show_density=False, title="")
ax.plot(traj.x[0, :], traj.x[1, :], color="k", linewidth=1.5, label="planned")
ax.legend(loc="upper left")
plt.show()
