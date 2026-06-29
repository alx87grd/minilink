"""MPC on a smooth U-turn reference path with scene obstacles — spatial track pipeline.

Fork of ``demo_dynamic_bicycle_rate_mpc_multi_obstacle_scene.py``: rounded 90°
corners replace sharp polyline kinks; the reference path extends before and after
the driven mission segment so the vehicle does not slow for "end of path".
Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_curvy_path.py
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

U_TARGET = 10.5
TF_SIM = 7.0
VX0 = 5.0

TURN_RADIUS = 6.0
LEAD_IN = 4.0
LEAD_OUT = 4.0
CORRIDOR_HALF_WIDTH = 2.0
PATH_COST_WEIGHT = 40.0
CORRIDOR_COST_WEIGHT = 25.0
OBSTACLE_RADIUS = 0.2
OBSTACLE_MARGIN = 0.2
OBSTACLE_CENTERS = (
    (9.0, -1.0),
    # (12.0, -1.0),
)
OBSTACLE_REPULSION_WEIGHT = 35.0
OBSTACLE_REPULSION_EPS = 0.08
PLOT_BOUNDS = ((-2.0, 20.0), (-3.5, 10.0))

MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 120
MPC_FTOL = 1e-1
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0
CAMERA_SCALE = 14.0
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


def smooth_u_turn_waypoints(
    *,
    x_left=0.0,
    y_top=8.0,
    y_bottom=-1.0,
    x_right=18.0,
    radius=6.0,
    n_arc=12,
):
    """U path with two rounded 90° corners (dense polyline approximation)."""
    r = radius
    y_arc = y_bottom + r
    cx_left = x_left + r
    cx_right = x_right - r

    left_vert = np.array([[x_left, y_top], [x_left, y_arc]])
    arc_left = _quarter_arc_waypoints(cx_left, y_arc, r, np.pi, n_arc)
    bottom = np.linspace([cx_left, y_bottom], [cx_right, y_bottom], 6)[1:-1]
    arc_right = _quarter_arc_waypoints(cx_right, y_arc, r, -np.pi / 2, n_arc)[1:]
    right_vert = np.linspace([x_right, y_arc], [x_right, y_top], 5)[1:-1]
    return np.vstack(
        [left_vert, arc_left[1:], bottom, arc_right, right_vert, [[x_right, y_top]]]
    )


def extended_u_turn_waypoints(*, lead_in=4.0, lead_out=4.0, **u_kwargs):
    mission = smooth_u_turn_waypoints(**u_kwargs)
    x_left = u_kwargs.get("x_left", 0.0)
    x_right = u_kwargs.get("x_right", 18.0)
    y_top = u_kwargs.get("y_top", 8.0)
    lead = np.linspace([x_left, y_top + lead_in], [x_left, y_top], 4)[0:-1]
    tail = np.linspace([x_right, y_top], [x_right, y_top + lead_out], 4)[1:]
    return np.vstack([lead, mission, tail]), mission


WAYPOINTS, MISSION_WAYPOINTS = extended_u_turn_waypoints(
    radius=TURN_RADIUS,
    lead_in=LEAD_IN,
    lead_out=LEAD_OUT,
)
MISSION_START = MISSION_WAYPOINTS[0].copy()
MISSION_END = MISSION_WAYPOINTS[-1].copy()

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

track = ReferenceTrack(from_waypoints(WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH)
robot = car(length=2.4, width=0.2, position=(0, 1), heading=2, margin=0.05)
scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS)
)

stability_cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
    R=np.diag([1.0, 22.0]),
    S=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
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

s_start, _ = track.path.project(MISSION_START)
theta0 = float(
    np.arctan2(track.path.tangent(s_start)[1], track.path.tangent(s_start)[0])
)
x0 = np.array(
    [
        MISSION_START[0],
        MISSION_START[1],
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

t_hist = [0.0]
x_hist = [x0.copy()]
u_hist = [np.zeros(sys_sim.m)]
mpc_plans = []
x = x0.copy()
t = 0.0
u_hold = np.zeros(sys_sim.m)
prev_plan = None
next_mpc_t = 0.0

s_goal, _ = track.path.project(MISSION_END)
print("MPC smooth U-turn reference path + obstacles (rate inputs)")
print(
    f"  path length={track.path.total_length:.1f} m "
    f"(mission {s_goal - s_start:.1f} m), "
    f"R={TURN_RADIUS:.1f} m, u_target={U_TARGET} m/s, {len(OBSTACLE_CENTERS)} obstacles"
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

traj = Trajectory(t=np.asarray(t_hist), x=np.asarray(x_hist).T, u=np.asarray(u_hist).T)

clearances = [
    np.hypot(traj.x[0, :] - cx, traj.x[1, :] - cy) - OBSTACLE_RADIUS
    for cx, cy in OBSTACLE_CENTERS
]
path_margins = [
    float(track.corridor_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
print(
    f"done: min obstacle clearance={float(np.min(clearances)):.2f} m, "
    f"min corridor margin={min(path_margins):.3f} m, "
    f"final position error={np.linalg.norm(traj.x[0:2, -1] - MISSION_END):.2f} m"
)

fig, ax = track.plot(
    show=False, bounds=PLOT_BOUNDS, title="Smooth U-turn + MPC executed path"
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
    [MISSION_START[0], MISSION_END[0]],
    [MISSION_START[1], MISSION_END[1]],
    c=["C1", "C4"],
    s=36,
    zorder=8,
)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()


class MpcCurvyPathBicycleRate(JaxDynamicBicycleRateInputs):
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

    # === v2 frame-keyed visualization contract ===========================
    # Overlays are honest dynamic geometry: each frame bakes the world-frame
    # polyline from ``points_at(t)`` into a CustomLine (no T[3,3] time channel).

    def get_kinematic_geometry_v2(self):
        geometry = super().get_kinematic_geometry_v2()
        geometry.setdefault("world", [])
        geometry["world"] = [
            self._upper,
            self._lower,
            self._centerline,
            *self._obstacles,
            *geometry["world"],
        ]
        return geometry

    def tf_v2(self, x, u, t=0, params=None):
        frames = super().tf_v2(x, u, t)
        frames.setdefault("world", np.eye(4))
        return frames

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        dynamic = super().get_dynamic_geometry_v2(x, u, t)
        dynamic.setdefault("world", [])
        dynamic["world"] = [
            *dynamic["world"],
            CustomLine(
                self._executed.points_at(t), color="b", style="--", linewidth=1.0
            ),
            CustomLine(
                self._mpc_plan.points_at(t),
                color="tab:orange",
                style="--",
                linewidth=2.0,
            ),
        ]
        return dynamic


mpc_anim_sys = MpcCurvyPathBicycleRate(
    track,
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
