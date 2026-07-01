"""MPC fast lap on a wide 50 m × 10 m stadium track with obstacles.

Higher target speed and faster MPC rate than ``demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py``.
Uses compile-once :class:`~minilink.planning.mpc.planner.MPCPlanner`.

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_fast_stadium_lap.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.graphical.animation.primitives import HorizonPolyline, TrajectoryPolyline
from minilink.graphical.catalog import SceneHistory
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.mpc import (
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.overlays import TrackCorridorOverlay
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
)

TRACK_CENTER = (0.0, 0.0)
TRACK_WIDTH = 50.0
TRACK_HEIGHT = 10.0
TURN_RADIUS = 5.0
CORRIDOR_HALF_WIDTH = 4.0
PATH_COST_WEIGHT = 40.0
CORRIDOR_COST_WEIGHT = 45.0
UNROLL_LAPS = 3

OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.05
OBSTACLE_CENTERS = (
    (0.0, 8.0),
    (0.0, -9.0),
    (-20.0, 0.0),
    (-19.0, 0.0),
    (-18.0, 0.0),
    (-17.0, 0.0),
    (-16.0, 0.0),
    (-15.0, 0.0),
    (-14.0, 0.0),
    (-13.0, 0.0),
    (-12.0, 0.0),
    (-11.0, 0.0),
    (-10.0, 0.0),
    (-9.0, 0.0),
    (-8.0, 0.0),
    (-7.0, 0.0),
    (-6.0, 0.0),
    (-5.0, 0.0),
    (-4.0, 0.0),
    (-3.0, 0.0),
    (-2.0, 0.0),
    (-1.0, 0.0),
    (0.0, 0.0),
    (1.0, 0.0),
    (2.0, 0.0),
    (3.0, 0.0),
    (4.0, 0.0),
    (5.0, 0.0),
    (6.0, 0.0),
    (7.0, 0.0),
    (8.0, 0.0),
    (9.0, 0.0),
    (10.0, 0.0),
    (11.0, 0.0),
    (12.0, 0.0),
    (13.0, 0.0),
    (14.0, 0.0),
    (15.0, 0.0),
    (16.0, 0.0),
    (17.0, 0.0),
    (18.0, 0.0),
    (19.0, 0.0),
    (20.0, 0.0),
)
OBSTACLE_REPULSION_WEIGHT = 3.0
OBSTACLE_REPULSION_EPS = 0.08

U_TARGET = 30.0
VX0 = 2.5
TF_SIM = 20.0

MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 10
MPC_MAXITER = 50
MPC_FTOL = 1.0
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0
CAMERA_SCALE = 16.0
PLOT_MARGIN = 3.0


def _quarter_arc_waypoints(cx, cy, radius, angle_start, n=12):
    angles = np.linspace(angle_start, angle_start + np.pi / 2, n)
    return np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles)])


def rounded_rect_loop_waypoints(*, cx, cy, width, height, radius, n_arc=10):
    w2, h2, r = width / 2.0, height / 2.0, radius
    hx = w2 - r
    start = np.array([[cx - hx, cy - h2]])
    bottom = np.linspace([cx - hx, cy - h2], [cx + hx, cy - h2], 5)[1:]
    arc_br = _quarter_arc_waypoints(cx + hx, cy - h2 + r, r, -np.pi / 2, n_arc)[1:]
    right = np.linspace([cx + w2, cy - h2 + r], [cx + w2, cy + h2 - r], 4)[1:-1]
    arc_tr = _quarter_arc_waypoints(cx + hx, cy + h2 - r, r, 0.0, n_arc)[1:]
    top = np.linspace([cx + hx, cy + h2], [cx - hx, cy + h2], 5)[1:-1]
    arc_tl = _quarter_arc_waypoints(cx - hx, cy + h2 - r, r, np.pi / 2, n_arc)[1:]
    left = np.linspace([cx - w2, cy + h2 - r], [cx - w2, cy - h2 + r], 4)[1:-1]
    arc_bl = _quarter_arc_waypoints(cx - hx, cy - h2 + r, r, np.pi, n_arc)[1:]
    return np.vstack(
        [start, bottom, arc_br, right, arc_tr, top, arc_tl, left, arc_bl, start]
    )


def unroll_track(waypoints, n_laps=3):
    parts = [np.asarray(waypoints, dtype=float)]
    for _ in range(n_laps - 1):
        parts.append(parts[-1][1:])
    return np.vstack(parts)


LOOP_WAYPOINTS = rounded_rect_loop_waypoints(
    cx=TRACK_CENTER[0],
    cy=TRACK_CENTER[1],
    width=TRACK_WIDTH,
    height=TRACK_HEIGHT,
    radius=TURN_RADIUS,
)
REFERENCE_WAYPOINTS = unroll_track(LOOP_WAYPOINTS, n_laps=UNROLL_LAPS)
START_XY = LOOP_WAYPOINTS[0].copy()
PLOT_BOUNDS = (
    (
        TRACK_CENTER[0] - TRACK_WIDTH / 2 - PLOT_MARGIN,
        TRACK_CENTER[0] + TRACK_WIDTH / 2 + PLOT_MARGIN,
    ),
    (
        TRACK_CENTER[1] - TRACK_HEIGHT / 2 - PLOT_MARGIN,
        TRACK_CENTER[1] + TRACK_HEIGHT / 2 + PLOT_MARGIN,
    ),
)

configure_jax(enable_x64=True)

sys_mpc = JaxDynamicBicycleRateInputs()
sys_sim = JaxDynamicBicycleRateInputs()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

for sys in (sys_mpc, sys_sim):
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = W_REAR_MAX
    sys.state.lower_bound[7] = -DELTA_MAX
    sys.state.upper_bound[7] = DELTA_MAX
    sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
    sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
    sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
    sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX

keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
r_r = sys_mpc.params["r_r"]
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])

loop_track = ReferenceTrack(
    from_waypoints(LOOP_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
track = ReferenceTrack(
    from_waypoints(REFERENCE_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS)
)

cost = (
    QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.3, 1.0, 1.0, 0.01, 0.1]),
        R=np.diag([1.0, 1.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.3, 1.0, 1.0, 0.01, 0.1]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    )
    + track.distance_field(body).as_cost(
        weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
    )
    + track.corridor_field(body).as_cost(
        weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
    )
    + scene.clearance_field(body).as_cost(
        weight=OBSTACLE_REPULSION_WEIGHT,
        shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
    )
)

s_start, _ = loop_track.path.project(START_XY)
theta0 = float(
    np.arctan2(
        loop_track.path.tangent(s_start)[1],
        loop_track.path.tangent(s_start)[0],
    )
)
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([START_XY[0], START_XY[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

sim_evaluator = sys_sim.compile(backend="jax", verbose=False)
template_problem = PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost)
mpc_planner = MPCPlanner(
    template_problem,
    transcription=MPCDirectCollocationTranscription(
        DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
    ),
    options=MPCOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        record_solve_time=True,
        optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
    ),
)

lap_length = loop_track.path.total_length
t_hist, x_hist, u_hist = [0.0], [x0.copy()], [np.zeros(sys_sim.m)]
mpc_plans, progress_hist = [], [s_start]
x, t, u_hold = x0.copy(), 0.0, np.zeros(sys_sim.m)
prev_plan, next_mpc_t = None, 0.0

print("MPC fast stadium lap (50 m × 10 m track)")
print(f"  compile={mpc_planner.compile_time_s:.3f}s (once)")
print(
    f"  lap length={lap_length:.1f} m, u_target={U_TARGET} m/s, "
    f"mpc_hz={MPC_HZ}, tf_sim={TF_SIM:.1f} s, {len(OBSTACLE_CENTERS)} obstacles"
)

while t < TF_SIM - 1e-12:
    if t >= next_mpc_t - 1e-12:
        guess = None
        if prev_plan is not None and prev_plan.n_samples >= 3:
            t_shift = prev_plan.t + MPC_DT
            mask = t_shift <= MPC_HORIZON + 1e-9
            if np.count_nonzero(mask) >= 3:
                x_guess = prev_plan.x[:, mask].copy()
                x_guess[:, 0] = x
                guess = Trajectory(
                    t=t_shift[mask] - t, x=x_guess, u=prev_plan.u[:, mask]
                )
        else:
            guess = default_initial_trajectory(
                template_problem,
                mpc_planner.transcription.initial_guess_time_grid(template_problem),
            )

        plan = mpc_planner.step(x, initial_guess=guess)
        res = mpc_planner.last_optimization_result
        print(
            f"MPC @ t={t:.2f}s  success={res.success}  "
            f"solve={res.solve_time_s:.3f}s  step={mpc_planner.last_step_time_s:.3f}s"
        )
        prev_plan = plan
        u_hold = plan.u[:, 0].copy()
        mpc_plans.append(
            (t, Trajectory(t=plan.t + t, x=plan.x.copy(), u=plan.u.copy()))
        )
        next_mpc_t += MPC_DT

    for _ in range(SUBSTEPS):
        if t >= TF_SIM:
            break
        x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
        t += SIM_DT
        t_hist.append(t)
        x_hist.append(x.copy())
        u_hist.append(u_hold.copy())
        progress_hist.append(loop_track.path.project(x[0:2])[0])

traj = Trajectory(t=np.asarray(t_hist), x=np.asarray(x_hist).T, u=np.asarray(u_hist).T)
progress = np.asarray(progress_hist)
progress_delta = np.diff(progress, prepend=progress[0])
progress_delta = np.where(
    progress_delta < -0.5 * lap_length, progress_delta + lap_length, progress_delta
)
laps_completed = np.cumsum(np.maximum(progress_delta, 0.0))[-1] / lap_length
clearances = [
    np.hypot(traj.x[0, :] - cx, traj.x[1, :] - cy) - OBSTACLE_RADIUS
    for cx, cy in OBSTACLE_CENTERS
]
print(
    f"done: laps~={laps_completed:.2f}, mean vx={float(np.mean(traj.x[3, :])):.2f} m/s, "
    f"max vx={float(np.max(traj.x[3, :])):.2f} m/s, "
    f"min obstacle clearance={float(np.min(clearances)):.2f} m"
)

fig, ax = loop_track.plot(
    show=False, bounds=PLOT_BOUNDS, title="Fast stadium track + MPC executed path"
)
scene.plot(show=False, ax=ax, bounds=PLOT_BOUNDS, show_density=False, title=None)
ax.plot(
    traj.x[0, :],
    traj.x[1, :],
    color="tab:blue",
    linewidth=1.8,
    label="executed",
    zorder=7,
)
ax.scatter([START_XY[0]], [START_XY[1]], c="C1", s=36, zorder=8, label="start/finish")
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()

history = SceneHistory(
    trail=TrajectoryPolyline(
        traj, window="prefix", color="#1565c0", style="--", linewidth=1.0
    ),
    horizon=HorizonPolyline(mpc_plans, color="#ef6c00", linewidth=2.0, style="--"),
)
sys_sim.params = dict(sys_sim.params)
sys_sim.camera_scale = CAMERA_SCALE
sys_sim.traj = traj
sys_sim.plot_trajectory(signals=("x", "u"))
sys_sim.animate(
    traj,
    overlays=[
        TrackCorridorOverlay(loop_track),
        scene.as_visualizer(color="tab:red", opacity=0.45),
        history,
    ],
)

# sys_sim.animate(
#     traj,
#     overlays=[
#         TrackCorridorOverlay(loop_track),
#         scene.as_visualizer(color="tab:red", opacity=0.45),
#         history,
#     ],
#     save=True,
#     file_name="fast_stadium_lap.gif",
#     show=False,  # optional: skip the popup window
# )
