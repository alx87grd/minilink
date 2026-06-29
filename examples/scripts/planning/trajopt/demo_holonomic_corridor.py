"""Holonomic path tracking — reference corridor, obstacles, and trajopt.

Run from the repo root::

    python examples/scripts/planning/trajopt/demo_holonomic_corridor.py

Waypoints define a polyline centerline (:func:`~minilink.planning.spatial.paths.from_waypoints`).
A :class:`~minilink.planning.spatial.track.ReferenceTrack` exports a hard corridor
constraint and a soft path-distance cost, composed with obstacle clearance like
``demo_holonomic_obstacles.py``. Direct collocation finds a feasible trajectory
that stays in the tube and reaches the terminal goal.
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    TrajectoryPolyline,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene
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

TF = 14.0
N_STEPS = 50
ROBOT_RADIUS = 0.25
CORRIDOR_HALF_WIDTH = 0.85
PATH_COST_WEIGHT = 10.0
LEAD_IN = 3.0
LEAD_OUT = 3.0
PLOT_PAD = CORRIDOR_HALF_WIDTH + ROBOT_RADIUS + 0.3
CAMERA_SCALE = 12.0
TRACK_ANIM_SAMPLES = 200

# Mission U-turn: down, east along the bottom, up (traj stays on this segment).
MISSION_WAYPOINTS = np.array(
    [
        [0.0, 4.0],
        [0.0, 1.0],
        [0.0, -2.0],
        [4.0, -2.0],
        [8.0, -2.0],
        [8.0, 1.0],
        [8.0, 4.0],
    ]
)
X_START = MISSION_WAYPOINTS[0].copy()
X_GOAL = MISSION_WAYPOINTS[-1].copy()

# Reference path extends before/after the mission so terminal cost is not "end of path".
WAYPOINTS = np.vstack(
    [
        [[0.0, X_START[1] + LEAD_IN], [0.0, X_START[1] + 0.5]],
        MISSION_WAYPOINTS,
        [[X_GOAL[0], X_GOAL[1] + 0.5], [X_GOAL[0], X_GOAL[1] + LEAD_OUT]],
    ]
)

# Spheres sit on the centerline so the planner must weave inside the corridor.
OBSTACLES = (
    Sphere([0.0, 0.5], 0.35),
    Sphere([4.0, -2.0], 0.35),
    Sphere([8.0, -0.5], 0.35),
)


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


class HolonomicCorridorScene(HolonomicMobileRobot):
    """Holonomic robot with reference track, corridor, obstacles, and traj trail."""

    def __init__(self, track, obstacles, executed_traj, *, robot_radius=ROBOT_RADIUS):
        super().__init__()
        self.camera_scale = CAMERA_SCALE
        self._robot_radius = robot_radius

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
                radius=obs.radius,
                center=(obs.center[0], obs.center[1], 0.0),
                color="tab:red",
                fill=True,
            )
            for obs in obstacles
        ]
        self._executed = TrajectoryPolyline(
            executed_traj,
            window="prefix",
            color="#1f77b4",
            linewidth=2.5,
            style="-",
        )

    def get_kinematic_geometry(self):
        geometry = super().get_kinematic_geometry()
        geometry.setdefault("world", [])
        geometry["world"] = [
            self._upper,
            self._lower,
            self._centerline,
            *self._obstacles,
            *geometry["world"],
        ]
        return geometry

    def tf(self, x, u, t=0, params=None):
        frames = super().tf(x, u, t)
        frames.setdefault("world", np.eye(4))
        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        dynamic = super().get_dynamic_geometry(x, u, t)
        dynamic.setdefault("world", [])
        dynamic["world"] = [
            *dynamic["world"],
            CustomLine(
                self._executed.points_at(t),
                color="#1f77b4",
                linewidth=2.5,
                style="-",
            ),
        ]
        return dynamic


sys = HolonomicMobileRobot()
sys.state.lower_bound = np.array([-2.0, -6.0])
sys.state.upper_bound = np.array([10.0, 8.0])
sys.inputs["u"].lower_bound = np.array([-1.5, -1.5])
sys.inputs["u"].upper_bound = np.array([1.5, 1.5])

scene = Scene(obstacles=OBSTACLES)
track = ReferenceTrack(from_waypoints(WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH)
robot = sphere(radius=ROBOT_RADIUS, position=(0, 1))

X = (
    BoxSet.from_system_state(sys)
    & scene.clearance_field(robot).as_constraint()
    & track.corridor_field(robot).as_constraint(lower=0.0)
)
control_cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([0.0, 0.0]),
    R=np.diag([0.2, 0.2]),
    S=np.diag([80.0, 80.0]),
    xbar=X_GOAL,
)
path_cost = track.distance_field(robot).as_cost(
    weight=PATH_COST_WEIGHT,
    shaping=quadratic_excess(threshold=0.05),
)
problem = PlanningProblem(
    sys=sys,
    x_start=X_START,
    x_goal=X_GOAL,
    X=X,
    Xf=SingletonSet(X_GOAL),
    cost=control_cost + path_cost,
)

transcription = DirectCollocationTranscription(
    DirectCollocationOptions(tf=TF, n_steps=N_STEPS)
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=transcription,
    options=TrajectoryOptimizationOptions(
        compile_backend="numpy",
        optimizer_options={"maxiter": 500, "ftol": 1e-1},
    ),
)

t_grid = transcription.options.t
s_start, _ = track.path.project(X_START)
s_goal, _ = track.path.project(X_GOAL)
arc = np.linspace(s_start, s_goal, t_grid.size)
xy = np.array([track.path.sample(s) for s in arc])
guess = Trajectory(
    t=t_grid,
    x=xy.T,
    u=np.gradient(xy, t_grid, axis=0).T,
)

traj = planner.compute_solution(initial_guess=guess)

distances = [
    float(track.distance_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
corridor_margins = [
    float(track.corridor_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
print("holonomic corridor trajopt")
print(
    f"  path length {track.path.total_length:.1f} m "
    f"(mission {s_goal - s_start:.1f} m), horizon {TF:.1f} s"
)
print(f"  min corridor margin {min(corridor_margins):.3f} m")
print(f"  final position error {np.linalg.norm(traj.x[:, -1] - X_GOAL):.4f} m")

plot_bounds = (
    (WAYPOINTS[:, 0].min() - PLOT_PAD, WAYPOINTS[:, 0].max() + PLOT_PAD),
    (WAYPOINTS[:, 1].min() - PLOT_PAD, WAYPOINTS[:, 1].max() + PLOT_PAD),
)
fig, ax = track.plot(
    show=False, bounds=plot_bounds, title="U-turn reference track + trajopt"
)
scene.plot(show=False, ax=ax, bounds=plot_bounds, show_density=False, title=None)
ax.plot(
    traj.x[0, :],
    traj.x[1, :],
    color="#1f77b4",
    linewidth=2.5,
    label="trajopt",
    zorder=7,
)
ax.scatter(
    [X_START[0], X_GOAL[0]],
    [X_START[1], X_GOAL[1]],
    c=["#ff7f0e", "#9467bd"],
    s=48,
    zorder=8,
)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()

planner.plot_solution(signals=("x", "u"))

anim_sys = HolonomicCorridorScene(track, OBSTACLES, traj, robot_radius=ROBOT_RADIUS)
anim_sys.traj = traj
anim_sys.animate()
