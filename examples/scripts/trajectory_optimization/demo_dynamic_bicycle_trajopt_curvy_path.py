"""Dynamic-bicycle 90° left turn — JAX direct collocation with rate inputs.

Fork of ``demo_dynamic_bicycle_trajopt_lanechange.py``: a short east-to-north
reference path (straight leg + quarter circle) is tracked with
:class:`~minilink.planning.spatial.track.ReferenceTrack` path-distance cost on
:class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycleRateInputs`.
Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_trajopt_curvy_path.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.graphical.animation.primitives import (
    CustomLine,
    TrajectoryPolyline,
    time_channel_matrix,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.robot import car
from minilink.planning.spatial.shaping import quadratic_excess
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

PRINT_SOLVE_REPORT = True
TF = 4.0
N_STEPS = 24
W_REAR_MAX = 80.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0
TURN_RADIUS = 2.5
STRAIGHT_BEFORE = 3.0
STRAIGHT_AFTER = 2.0
CORRIDOR_HALF_WIDTH = 1.2
PATH_COST_WEIGHT = 15.0
PLOT_BOUNDS = ((-1.0, 8.0), (-1.5, 6.5))
CAMERA_SCALE = 12.0
TRACK_ANIM_SAMPLES = 150


def _quarter_arc_waypoints(cx, cy, radius, angle_start, n=12):
    """90° CCW arc from ``angle_start`` (always ``+ pi/2`` sweep)."""
    angles = np.linspace(angle_start, angle_start + np.pi / 2, n)
    return np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles)])


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


class TrajoptCurvyPathBicycleRate(JaxDynamicBicycleRateInputs):
    """Rate-input bicycle with corridor, reference path, and traj trail."""

    def __init__(self, track, executed_traj):
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
            style="--",
        )
        self._executed = TrajectoryPolyline(
            executed_traj,
            window="prefix",
            color="#1f77b4",
            linewidth=2.5,
            style="-",
        )

    def get_kinematic_geometry(self):
        vehicle = super().get_kinematic_geometry()
        return [self._upper, self._lower, self._centerline, self._executed] + vehicle

    def get_kinematic_transforms(self, x, u, t):
        vehicle = super().get_kinematic_transforms(x, u, t)
        return [np.eye(4)] * 3 + [time_channel_matrix(t)] + list(vehicle)


# East straight, quarter-circle left to north, short north leg.
R = TURN_RADIUS
L = STRAIGHT_BEFORE
arc = _quarter_arc_waypoints(L, R, R, -np.pi / 2, 12)
WAYPOINTS = np.vstack(
    [
        [[0.0, 0.0], [L, 0.0]],
        arc[1:],
        [[L + R, R + STRAIGHT_AFTER]],
    ]
)

sys = JaxDynamicBicycleRateInputs()
sys.state.lower_bound[6] = 0.0
sys.state.upper_bound[6] = W_REAR_MAX
sys.state.lower_bound[7] = -DELTA_MAX
sys.state.upper_bound[7] = DELTA_MAX
sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX

track = ReferenceTrack(from_waypoints(WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH)
robot = car(length=2.4, width=1.1, position=(0, 1), heading=2, margin=0.05)
r_r = sys.params["r_r"]
v_ref = track.path.total_length / TF
ubar = np.array([0.0, 0.0])
theta0 = float(np.arctan2(track.path.tangent(0.0)[1], track.path.tangent(0.0)[0]))
x_start = np.array([0.0, 0.0, theta0, v_ref, 0.0, 0.0, v_ref / r_r, 0.0])
x_end = WAYPOINTS[-1]
theta_f = float(
    np.arctan2(
        track.path.tangent(track.path.total_length)[1],
        track.path.tangent(track.path.total_length)[0],
    )
)
x_ref = np.array([x_end[0], x_end[1], theta_f, v_ref, 0.0, 0.0, v_ref / r_r, 0.0])

stability_cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([0.0, 0.0, 0.0, 0.1, 2.0, 1.0, 0.1, 40.0]),
    R=np.diag([1.0, 25.0]),
    S=np.diag([40.0, 40.0, 30.0, 1.0, 8.0, 4.0, 0.1, 60.0]),
    xbar=x_ref,
    ubar=ubar,
)
path_cost = track.distance_field(robot).as_cost(
    weight=PATH_COST_WEIGHT,
    shaping=quadratic_excess(threshold=0.1),
)
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    cost=stability_cost + path_cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(tf=TF, n_steps=N_STEPS)
    ),
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        solve_disp=PRINT_SOLVE_REPORT,
        optimizer_options={"maxiter": 400, "ftol": 1e-2},
    ),
)

traj = planner.compute_solution()

path_errors = [
    float(track.distance_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
print("dynamic bicycle 90° left turn trajopt (rate inputs)")
print(f"  path length {track.path.total_length:.1f} m, tf={TF:.1f} s")
print(f"  min body-edge margin to path {min(path_errors):.3f} m")
print(f"  terminal position error {np.linalg.norm(traj.x[0:2, -1] - x_end):.3f} m")

fig, ax = track.plot(
    show=False, bounds=PLOT_BOUNDS, title="90° left turn + bicycle trajopt"
)
ax.plot(
    traj.x[0, :],
    traj.x[1, :],
    color="#1f77b4",
    linewidth=2.5,
    label="trajopt",
    zorder=7,
)
ax.scatter(
    [x_start[0], x_end[0]], [x_start[1], x_end[1]], c=["C1", "C4"], s=40, zorder=8
)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()

planner.plot_solution(signals=("x", "u"))

anim_sys = TrajoptCurvyPathBicycleRate(track, traj)
anim_sys.params = dict(sys.params)
anim_sys.camera_scale = CAMERA_SCALE
anim_sys.traj = traj
anim_sys.animate()
