"""MPC lap-time pursuit on a closed rounded-rectangle track with obstacles.

Fork of ``demo_dynamic_bicycle_rate_mpc_curvy_path.py``: the reference path is a
closed loop (one physical lap). The planner uses an unrolled copy of the loop so
the receding horizon always sees track ahead. Path and corridor costs keep the
vehicle on the racing line; a weak surge-speed penalty lets MPC push velocity on
straights while respecting obstacles and lateral stability.

Toggle ``MULTI_LAP`` for a longer rollout (several complete laps).

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py
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
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
    time_channel_matrix,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.robot import car
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# --- Track geometry ---
TRACK_CENTER = (0.0, 0.0)
TRACK_WIDTH = 24.0
TRACK_HEIGHT = 14.0
TURN_RADIUS = 3.5
CORRIDOR_HALF_WIDTH = 2.0
PATH_COST_WEIGHT = 40.0
CORRIDOR_COST_WEIGHT = 25.0
UNROLL_LAPS = 3

# --- Obstacles (workspace centers) ---
OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.05
OBSTACLE_CENTERS = (
    (4.0, -6.2),
    (-2.0, 5.0),
    (10.0, 1.5),
)
OBSTACLE_REPULSION_WEIGHT = 3.0
OBSTACLE_REPULSION_EPS = 0.08

# --- Speed objective ---
U_TARGET = 20.0
VX0 = 2.5

# --- Simulation horizon ---
# Short default for quick iteration; flip MULTI_LAP for several laps.
MULTI_LAP = True
TF_SIM = 16

MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 1.5
MPC_STEPS = 15
MPC_MAXITER = 120
MPC_FTOL = 1e-1
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0
CAMERA_SCALE = 16.0
PLOT_MARGIN = 3.0
TRACK_ANIM_SAMPLES = 200


def _sample_track_boundaries(track, n_samples=TRACK_ANIM_SAMPLES):
    """Centerline plus corridor upper/lower edges (same geometry as ``plot_track``)."""
    path = track.path
    ss = np.linspace(0.0, path.total_length, n_samples)
    center = np.array([path.sample(s) for s in ss])
    tangents = np.array([path.tangent(s) for s in ss])
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)
    half = float(track.half_width)
    upper = center + half * normals
    lower = center - half * normals
    return center, upper, lower


def _line3(xy):
    return np.hstack([xy, np.zeros((xy.shape[0], 1))])


def _quarter_arc_waypoints(cx, cy, radius, angle_start, n=12):
    """90° counter-clockwise arc from ``angle_start``.

    Always sweep ``angle_start + pi/2``. Do **not** use ``linspace(a0, a1)`` when
    ``a1 < a0`` (e.g. ``pi`` to ``-pi/2``): that traces the 270° long arc.
    """
    angles = np.linspace(angle_start, angle_start + np.pi / 2, n)
    return np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles)])


def rounded_rect_loop_waypoints(
    *,
    cx=0.0,
    cy=0.0,
    width=24.0,
    height=14.0,
    radius=3.5,
    n_arc=10,
):
    """Closed CCW loop: rounded rectangle centered at ``(cx, cy)``.

    Start/finish sits on the bottom straight, heading east.
    """
    w2 = width / 2.0
    h2 = height / 2.0
    r = radius
    hx = w2 - r
    hy = h2 - r

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
    """Concatenate ``n_laps`` copies so MPC always has path ahead of the vehicle."""
    if n_laps < 1:
        raise ValueError("n_laps must be >= 1")
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

half_w = TRACK_WIDTH / 2.0
half_h = TRACK_HEIGHT / 2.0
PLOT_BOUNDS = (
    (TRACK_CENTER[0] - half_w - PLOT_MARGIN, TRACK_CENTER[0] + half_w + PLOT_MARGIN),
    (TRACK_CENTER[1] - half_h - PLOT_MARGIN, TRACK_CENTER[1] + half_h + PLOT_MARGIN),
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
w_rear_ref = U_TARGET / r_r
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
ubar = np.array([0.0, 0.0])

loop_track = ReferenceTrack(
    from_waypoints(LOOP_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
track = ReferenceTrack(
    from_waypoints(REFERENCE_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
robot = car(length=2.4, width=0.2, position=(0, 1), heading=2, margin=0.05)
scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS)
)

stability_cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
    R=np.diag([1.0, 22.0]),
    S=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
    xbar=x_cruise,
    ubar=ubar,
)
path_cost = track.distance_field(robot).as_cost(
    weight=PATH_COST_WEIGHT,
    shaping=quadratic_excess(threshold=0.1),
)
corridor_cost = track.corridor_field(robot).as_cost(
    weight=CORRIDOR_COST_WEIGHT,
    shaping=quadratic_hinge(threshold=0.0),
)
obstacle_cost = scene.clearance_field(robot).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = stability_cost + path_cost + corridor_cost + obstacle_cost

s_start, _ = loop_track.path.project(START_XY)
theta0 = float(
    np.arctan2(
        loop_track.path.tangent(s_start)[1],
        loop_track.path.tangent(s_start)[0],
    )
)
# Path-distance costs can make the first SLSQP iterate infeasible at exactly
# axis-aligned headings (theta = 0, ±pi/2); a tiny nudge avoids that corner.
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array(
    [
        START_XY[0],
        START_XY[1],
        theta0,
        VX0,
        0.0,
        0.0,
        VX0 / r_r,
        0.0,
    ]
)
sim_evaluator = sys_sim.compile(backend="jax", verbose=False)

transcription = DirectCollocationTranscription(
    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
)
trajopt_options = TrajectoryOptimizationOptions(
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    solve_disp=False,
    record_solve_time=True,
    optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
)

lap_length = loop_track.path.total_length

t_hist = [0.0]
x_hist = [x0.copy()]
u_hist = [np.zeros(sys_sim.m)]
mpc_plans = []
progress_hist = [s_start]
x = x0.copy()
t = 0.0
u_hold = np.zeros(sys_sim.m)
prev_plan = None
next_mpc_t = 0.0

print("MPC closed-loop lap pursuit (rate inputs)")
print(
    f"  lap length={lap_length:.1f} m, "
    f"u_target={U_TARGET} m/s, tf_sim={TF_SIM:.1f} s "
    f"({'multi-lap' if MULTI_LAP else 'short'}), "
    f"{len(OBSTACLE_CENTERS)} obstacles"
)

while t < TF_SIM - 1e-12:
    if t >= next_mpc_t - 1e-12:
        problem = PlanningProblem(sys=sys_mpc, x_start=x, cost=cost)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=trajopt_options,
        )

        guess = None
        if prev_plan is not None and prev_plan.n_samples >= 3:
            t_shift = prev_plan.t + MPC_DT
            mask = t_shift <= MPC_HORIZON + 1e-9
            if np.count_nonzero(mask) >= 3:
                x_guess = prev_plan.x[:, mask].copy()
                x_guess[:, 0] = x
                guess = Trajectory(
                    t=t_shift[mask] - t,
                    x=x_guess,
                    u=prev_plan.u[:, mask],
                )
        else:
            guess = default_initial_trajectory(
                problem,
                transcription.initial_guess_time_grid(problem),
            )

        plan = planner.compute_solution(initial_guess=guess)
        res = planner.last_optimization_result
        print(f"MPC @ t={t:.2f}s  success={res.success}  solve={res.solve_time_s:.3f}s")
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
        s_now, _ = loop_track.path.project(x[0:2])
        progress_hist.append(s_now)

traj = Trajectory(t=np.asarray(t_hist), x=np.asarray(x_hist).T, u=np.asarray(u_hist).T)
progress = np.asarray(progress_hist)
progress_delta = np.diff(progress, prepend=progress[0])
progress_delta = np.where(
    progress_delta < -0.5 * lap_length, progress_delta + lap_length, progress_delta
)
arc_progress = np.cumsum(np.maximum(progress_delta, 0.0))
laps_completed = arc_progress[-1] / lap_length
mean_speed = float(np.mean(traj.x[3, :]))
max_speed = float(np.max(traj.x[3, :]))

clearances = [
    np.hypot(traj.x[0, :] - cx, traj.x[1, :] - cy) - OBSTACLE_RADIUS
    for cx, cy in OBSTACLE_CENTERS
]
path_margins = [
    float(loop_track.corridor_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
print(
    f"done: laps~={laps_completed:.2f}, mean vx={mean_speed:.2f} m/s, "
    f"max vx={max_speed:.2f} m/s, "
    f"min obstacle clearance={float(np.min(clearances)):.2f} m, "
    f"min corridor margin={min(path_margins):.3f} m"
)

fig, ax = loop_track.plot(
    show=False, bounds=PLOT_BOUNDS, title="Closed loop track + MPC executed path"
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
ax.scatter(
    [START_XY[0]],
    [START_XY[1]],
    c="C1",
    s=36,
    zorder=8,
    label="start/finish",
)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()


class MpcClosedLoopLapBicycleRate(JaxDynamicBicycleRateInputs):
    def __init__(
        self, track, mpc_plans, executed_traj, *, obstacle_centers, keepout_radius
    ):
        super().__init__()
        center, upper, lower = _sample_track_boundaries(track)
        self._upper = CustomLine(
            _line3(upper),
            color="#98df8a",
            linewidth=1.2,
            style="-",
        )
        self._lower = CustomLine(
            _line3(lower),
            color="#98df8a",
            linewidth=1.2,
            style="-",
        )
        self._centerline = CustomLine(
            _line3(center),
            color="#2ca02c",
            linewidth=2.0,
            style="-",
        )
        self._obstacles = [
            Circle(
                radius=keepout_radius,
                center=(cx, cy, 0.0),
                color="tab:red",
                fill=True,
            )
            for cx, cy in obstacle_centers
        ]
        self._executed = TrajectoryPolyline(
            executed_traj, window="prefix", color="b", style="--", linewidth=1.0
        )
        self._mpc_plan = HorizonPolyline(
            mpc_plans, color="tab:orange", linewidth=2.0, style="--"
        )

    def get_kinematic_geometry(self):
        vehicle = super().get_kinematic_geometry()
        return (
            [self._upper, self._lower, self._centerline]
            + self._obstacles
            + [self._executed]
            + vehicle
            + [self._mpc_plan]
        )

    def get_kinematic_transforms(self, x, u, t):
        vehicle = super().get_kinematic_transforms(x, u, t)
        n_obstacles = len(self._obstacles)
        return (
            [np.eye(4)] * 3
            + [np.eye(4)] * n_obstacles
            + [time_channel_matrix(t)]
            + list(vehicle)
            + [time_channel_matrix(t)]
        )


mpc_anim_sys = MpcClosedLoopLapBicycleRate(
    loop_track,
    mpc_plans,
    traj,
    obstacle_centers=OBSTACLE_CENTERS,
    keepout_radius=keepout_radius,
)
mpc_anim_sys.params = dict(sys_sim.params)
mpc_anim_sys.camera_scale = CAMERA_SCALE
mpc_anim_sys.traj = traj
mpc_anim_sys.plot_trajectory(signals=("x", "u"))
mpc_anim_sys.animate()
