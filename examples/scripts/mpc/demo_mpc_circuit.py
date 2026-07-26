"""Hybrid MPC on the wide technical circuit (track + corridor + obstacles).

``ModelPredictiveController`` with ``warm_start=True`` and ``mpc @ plant``.
Scene matches the former wide-circuit lap demo (asymmetric loop + sphere keepouts).

Run from repo root::

    python examples/scripts/mpc/demo_mpc_circuit.py
"""

import numpy as np

from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
)
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline, point_probe
from minilink.planning.spatial.grid import sample_field_costs
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.plotting import plot_cost_field_3d
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- knobs ---
U_TARGET = 20.0
VX0 = 5.0
TF_SIM = 24.0
MPC_DT = 0.2
SIM_DT = 0.005
MPC_HORIZON = 2.0
MPC_STEPS = 10

CORRIDOR_HALF_WIDTH = 2.5
PATH_COST_WEIGHT = 20.0
CORRIDOR_COST_WEIGHT = 25.0
OBSTACLE_RADIUS = 0.25
OBSTACLE_MARGIN = 0.2
OBSTACLE_CENTERS = (
    (-10.0, -9.5),
    (-2.5, -11.5),
    (-2.5, -12.5),
    (-2.5, -13.5),
    (-2.5, -14.5),
    (8.0, -10.5),
    (14.0, 0.0),
    (5.0, 10.5),
    (-6.0, 9.5),
    (-14.0, 2.0),
    (-6.0, -5.0),
    (-6.0, -6.0),
    (-6.0, -7.0),
    (-6.0, -8.0),
    (-6.0, -9.0),
)
OBSTACLE_REPULSION_WEIGHT = 50.0
OBSTACLE_REPULSION_EPS = 0.08
SHOW_COST_FIELD = True
PLOT_MARGIN = 3.0

# Wide CCW loop: bottom chicane, fast right corners, tight left hairpin.
loop_xy = np.array(
    [
        (-14.0, -10.0),
        (-9.3333, -10.0),
        (-7.0, -10.0),
        (-3.0, -8.2),
        (-1.0, -10.0),
        (10.5, -10.0),
        (13.7101, -9.6985),
        (14.5, -9.3301),
        (15.2139, -8.8302),
        (15.8302, -8.2139),
        (16.3301, -7.5),
        (16.6985, -6.7101),
        (16.9240, -5.8682),
        (17.0, -5.0),
        (17.0, 0.0),
        (17.0, 2.5),
        (16.6985, 6.7101),
        (16.3301, 7.5),
        (15.8302, 8.2139),
        (15.2139, 8.8302),
        (14.5, 9.3301),
        (13.7101, 9.6985),
        (12.8682, 9.9240),
        (12.0, 10.0),
        (0.75, 10.0),
        (-4.875, 10.0),
        (-14.6971, 9.7889),
        (-15.25, 9.5311),
        (-15.7498, 9.1812),
        (-16.1812, 8.7498),
        (-16.5311, 8.25),
        (-16.7889, 7.6971),
        (-16.9468, 7.1078),
        (-17.0, 6.5),
        (-17.0, 0.75),
        (-17.0, -2.125),
        (-16.6985, -6.7101),
        (-16.3301, -7.5),
        (-15.8302, -8.2139),
        (-15.2139, -8.8302),
        (-14.5, -9.3301),
        (-13.7101, -9.6985),
        (-12.8682, -9.9240),
        (-12.0, -10.0),
    ]
)
track = ReferenceTrack(from_waypoints(loop_xy), half_width=CORRIDOR_HALF_WIDTH)
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(obstacles=[Sphere(c, keepout_radius) for c in OBSTACLE_CENTERS])

sys_mpc = BicycleDynRate()
sys_mpc.state.lower_bound[6] = 0.0
sys_mpc.state.upper_bound[6] = 90.0
sys_mpc.state.lower_bound[7] = -0.55
sys_mpc.state.upper_bound[7] = 0.55
sys_mpc.inputs["u"].lower_bound = np.array([-80.0, -2.0])
sys_mpc.inputs["u"].upper_bound = np.array([80.0, 2.0])

r_r = sys_mpc.params["r_r"]
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
probe = bind(sys_mpc, point_probe())
path_cost = track.distance_field(body).as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)
corridor_cost = track.corridor_field(body).as_cost(
    weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
)
obstacle_cost = scene.clearance_field(body).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = (
    QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        R=np.diag([1.0, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    )
    + path_cost
    + corridor_cost
    + obstacle_cost
)

start_xy = loop_xy[0].copy()
s_start, _ = track.path.project(start_xy)
tangent = track.path.tangent(s_start)
theta0 = float(np.arctan2(tangent[1], tangent[0]))
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

if SHOW_COST_FIELD:
    bounds = (
        (loop_xy[:, 0].min() - PLOT_MARGIN, loop_xy[:, 0].max() + PLOT_MARGIN),
        (loop_xy[:, 1].min() - PLOT_MARGIN, loop_xy[:, 1].max() + PLOT_MARGIN),
    )
    path_viz = track.distance_field(probe).as_cost(
        weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
    )
    corridor_viz = track.corridor_field(probe).as_cost(
        weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
    )
    obstacle_viz = scene.clearance_field(probe).as_cost(
        weight=OBSTACLE_REPULSION_WEIGHT,
        shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
    )
    grid = sample_field_costs(
        [path_viz, corridor_viz, obstacle_viz],
        bounds=bounds,
        state_dim=sys_mpc.n,
        grid=(80, 80),
    )
    plot_cost_field_3d(grid, title="Combined cost field", log_scale=True)

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 150, "ftol": 0.1},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, step_disp=True)

sys_sim = BicycleDynRate()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
sys_sim.camera_scale = 18.0
sys_sim.x0 = x0.copy()

hybrid = mpc @ sys_sim
hybrid.plot_diagram()
result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0,
    plant_dt_inner=SIM_DT,
    compile_backend="jax",
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track)
)
