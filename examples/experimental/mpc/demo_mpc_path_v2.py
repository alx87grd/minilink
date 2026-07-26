"""Dual-rate path-tracking MPC with a ROS2-style manual deploy loop.

Big ``=====`` boxes mark the ROS2 copy-paste surface: one ``__init__`` block
(path → planning problem → planner → ``mpc``) and two timer callbacks.
Everything else is offline sim / plotting only.

Flip ``BACKEND`` below: ``"jax"`` parametric compile, ``"numpy"`` rebuild each
replan tick (see ``demo_mpc_minimal_numpy.py``).

Run from repo root::

    python examples/experimental/mpc/demo_mpc_path_v2.py
"""

import numpy as np

from minilink.control.mpc import ModelPredictiveController
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.graphical.animation.primitives import (
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.catalog import SceneHistory
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline, point_probe
from minilink.planning.spatial.grid import sample_field_costs
from minilink.planning.spatial.overlays import TrackCorridorOverlay
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.plotting import plot_cost_field_3d
from minilink.planning.spatial.shaping import quadratic_excess
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# Offline sim / viz params (not in the ROS2 node).
BACKEND = "jax"  # "jax" | "numpy"
# BACKEND = "numpy"
TF_SIM = 20.0  # 20.0
SIM_DT = 0.01
SHOW_COST_FIELD = True
CAMERA_SCALE = 14.0

# =============================================================================
# ROS2 NODE __init__ — copy this whole block once (keep ``mpc`` as a member)
# =============================================================================
WAYPOINTS = [
    (0.0, 0.0),
    (12.0, 0.0),
    (22.0, 5.0),
    (34.0, 5.0),
    (44.0, 0.0),
    (55.0, 0.0),
    (75.0, -20.0),
    (75.0, -80.0),
]
U_TARGET = 20.0
VX0 = 5.0
MPC_DT = 0.2
DT_BROADCAST = 0.01
MPC_HORIZON = 2.0
MPC_STEPS = 10
PATH_HALF_WIDTH = 5.0
PATH_COST_WEIGHT = 40.0

waypoints = np.asarray(WAYPOINTS, dtype=float)
track = ReferenceTrack(from_waypoints(waypoints), half_width=PATH_HALF_WIDTH)

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
cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
    R=np.diag([1.0, 22.0]),
    S=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
    xbar=x_cruise,
    ubar=np.zeros(2),
) + track.distance_field(body).as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)

# First state from sensors / localization (node start).
x0 = np.array([waypoints[0, 0], waypoints[0, 1], 0.0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend=BACKEND,
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 100, "ftol": 0.1},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, step_disp=True)
# =============================================================================
# end ROS2 NODE __init__
# =============================================================================

# Offline: path-aligned IC as a richer stand-in for the first localization sample.
start_xy = waypoints[0].copy()
s_start, _ = track.path.project(start_xy)
tangent = track.path.tangent(s_start)
theta0 = float(np.arctan2(tangent[1], tangent[0]))
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

# Offline: optional 3-D cost surface (tuning aid).
if SHOW_COST_FIELD:
    bounds = (
        (waypoints[:, 0].min() - 4.0, waypoints[:, 0].max() + 4.0),
        (waypoints[:, 1].min() - 4.0, waypoints[:, 1].max() + 4.0),
    )
    path_viz = track.distance_field(bind(sys_mpc, point_probe())).as_cost(
        weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
    )
    grid = sample_field_costs(
        [path_viz], bounds=bounds, state_dim=sys_mpc.n, grid=(80, 80)
    )
    plot_cost_field_3d(grid, title="Path cost field", log_scale=True)

# Offline: plant twin + demo clock (ROS2 uses the real vehicle / estimator).
sys_sim = BicycleDynRate()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
sys_sim.camera_scale = CAMERA_SCALE
sys_sim.x0 = x0.copy()
plant = sys_sim.compile(backend=BACKEND, verbose=False)

t = 0.0
x = x0.copy()  # in ROS2: measured / estimated plant state

x[0] = 10.0
x[1] = -50
x[2] = 0.0
x[3] = VX0
x[4] = 0.0
x[5] = 0.0
x[6] = VX0 / r_r
x[7] = 0.0


u_nom = np.zeros(sys_sim.m)
next_replan_t = 0.0
next_broadcast_t = 0.0
t_hist, x_hist, u_hist, mpc_plans = [t], [x.copy()], [u_nom.copy()], []

while t < TF_SIM - 1e-12:
    # Demo clock stands in for ROS2 timers (``create_timer`` at MPC_DT / DT_BROADCAST).
    if t >= next_replan_t - 1e-12:
        # =============================================================================
        # ROS2 NODE slow timer (period = MPC_DT) — copy these 2 lines
        #   inputs: measured state ``x``, wall time ``t``
        # =============================================================================
        cmd = mpc.compute_command(x, t=t)
        mpc.generate_nominal_interpolator(derivatives=True)
        # =============================================================================
        # end ROS2 NODE slow timer
        # =============================================================================

        # offline only: stash horizon for animation overlay
        plan = cmd.plan.trajectory
        mpc_plans.append(
            (t, Trajectory(t=plan.t + t, x=plan.x.copy(), u=plan.u.copy()))
        )
        next_replan_t += MPC_DT

    if t >= next_broadcast_t - 1e-12:
        # =============================================================================
        # ROS2 NODE fast timer (period = DT_BROADCAST) — copy these 4 lines
        #   then publish / apply ``u_nom`` (and optionally x/du/dx) to the plant
        # =============================================================================
        u_nom = mpc.get_nominal_u(t)
        x_nom = mpc.get_nominal_x(t)
        du_nom = mpc.get_nominal_u_dot(t)
        dx_nom = mpc.get_nominal_x_dot(t)
        # =============================================================================
        # end ROS2 NODE fast timer
        # =============================================================================
        next_broadcast_t += DT_BROADCAST

    # offline only: integrate plant (ROS2 talks to the real vehicle / estimator)
    x = plant.rk4_step(x, u_nom, t, SIM_DT)
    t += SIM_DT
    t_hist.append(t)
    x_hist.append(x.copy())
    u_hist.append(u_nom.copy())

# Offline: animate.
traj = Trajectory(t=np.asarray(t_hist), x=np.asarray(x_hist).T, u=np.asarray(u_hist).T)
sys_sim.traj = traj
sys_sim.animate(
    traj,
    # renderer="meshcat",
    overlays=[
        TrackCorridorOverlay(track),
        SceneHistory(
            trail=TrajectoryPolyline(
                traj, window="prefix", color="b", style="--", linewidth=1.0
            ),
            horizon=HorizonPolyline(
                mpc_plans, color="tab:orange", linewidth=2.0, style="--"
            ),
        ),
    ],
)
