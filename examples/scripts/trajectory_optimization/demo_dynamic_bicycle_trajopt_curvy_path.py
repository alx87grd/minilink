"""Dynamic-bicycle 90° turn — JAX direct collocation with rate inputs.

A reference path with lead-in/out extensions surrounds a short east-to-north
mission (straight leg + rounded corner). The solve runs in two phases — JAX
terminal tracking, then a short NumPy refinement with path-distance cost
(polyline projection is not JAX-traceable).
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
from minilink.core.trajectory import Trajectory
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

PRINT_SOLVE_REPORT = False
TF = 5.5
N_STEPS = 20
W_REAR_MAX = 80.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0
TURN_RADIUS = 6.0
STRAIGHT_BEFORE = 4.0
STRAIGHT_AFTER = 2.0
LEAD_IN = 8.0
LEAD_OUT = 8.0
CORRIDOR_HALF_WIDTH = 1.4
PATH_COST_WEIGHT = 15.0
PLOT_BOUNDS = ((-10.0, 20.0), (-2.0, 18.0))


def _mission_path_guess(track, s_start, s_goal, t_grid, v_ref, r_r):
    """State trajectory sampled on the mission centerline."""
    s = np.linspace(s_start, s_goal, t_grid.size)
    xy = np.array([track.path.sample(si) for si in s])
    theta = np.array(
        [
            float(np.arctan2(track.path.tangent(si)[1], track.path.tangent(si)[0]))
            for si in s
        ]
    )
    speed = np.maximum(
        np.linalg.norm(np.gradient(xy, t_grid, axis=0), axis=1), 0.5 * v_ref
    )
    x = np.vstack(
        [
            xy[:, 0],
            xy[:, 1],
            theta,
            speed,
            np.zeros(t_grid.size),
            np.zeros(t_grid.size),
            speed / r_r,
            np.zeros(t_grid.size),
        ]
    )
    return Trajectory(t=t_grid, x=x, u=np.zeros((2, t_grid.size)))


def _quarter_arc_waypoints(cx, cy, radius, angle_start, n=12):
    """90° counter-clockwise arc from ``angle_start``.

    Always sweep ``angle_start + pi/2``. Do **not** use ``linspace(a0, a1)`` when
    ``a1 < a0`` (e.g. ``pi`` to ``-pi/2``): that traces the 270° long arc.
    """
    angles = np.linspace(angle_start, angle_start + np.pi / 2, n)
    return np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles)])


R = TURN_RADIUS
L = STRAIGHT_BEFORE
arc = _quarter_arc_waypoints(L, R, R, -np.pi / 2, 14)
MISSION_END = np.array([L + R, R + STRAIGHT_AFTER])
mission = np.vstack([[[0.0, 0.0], [L, 0.0]], arc[1:], [MISSION_END]])
WAYPOINTS = np.vstack(
    [
        [[-LEAD_IN, 0.0], [0.0, 0.0]],
        mission[1:],
        [[MISSION_END[0], MISSION_END[1] + LEAD_OUT]],
    ]
)
X_START = np.array([0.0, 0.0])
X_GOAL = MISSION_END.copy()

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
s_start, _ = track.path.project(X_START)
s_goal, _ = track.path.project(X_GOAL)
mission_length = s_goal - s_start
v_ref = mission_length / TF
ubar = np.array([0.0, 0.0])
theta0 = float(
    np.arctan2(track.path.tangent(s_start)[1], track.path.tangent(s_start)[0])
)
theta_f = float(
    np.arctan2(track.path.tangent(s_goal)[1], track.path.tangent(s_goal)[0])
)
x_start = np.array([X_START[0], X_START[1], theta0, v_ref, 0.0, 0.0, v_ref / r_r, 0.0])
x_ref = np.array([X_GOAL[0], X_GOAL[1], theta_f, v_ref, 0.0, 0.0, v_ref / r_r, 0.0])

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
warmup_problem = PlanningProblem(sys=sys, x_start=x_start, cost=stability_cost)

transcription = DirectCollocationTranscription(
    DirectCollocationOptions(tf=TF, n_steps=N_STEPS)
)
t_grid = transcription.initial_guess_time_grid(problem)
path_guess = _mission_path_guess(track, s_start, s_goal, t_grid, v_ref, r_r)

warmup_options = TrajectoryOptimizationOptions(
    compile_backend="jax",
    solve_disp=False,
    record_solve_time=True,
    optimizer_options={"maxiter": 120, "ftol": 1e-1},
)
refine_options = TrajectoryOptimizationOptions(
    compile_backend="numpy",
    solve_disp=PRINT_SOLVE_REPORT,
    record_solve_time=True,
    optimizer_options={"maxiter": 100, "ftol": 1e-1},
)

warmup_planner = TrajectoryOptimizationPlanner(
    warmup_problem,
    transcription=transcription,
    options=warmup_options,
)
warm_traj = warmup_planner.compute_solution(initial_guess=path_guess)

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=transcription,
    options=refine_options,
)

traj = planner.compute_solution(initial_guess=warm_traj)
warm_s = warmup_planner.last_optimization_result.solve_time_s
refine_s = planner.last_optimization_result.solve_time_s

path_errors = [
    float(track.distance_field(robot).value(traj.x[:, k]))
    for k in range(traj.n_samples)
]
print("dynamic bicycle 90° turn trajopt (rate inputs)")
print(
    f"  path length {track.path.total_length:.1f} m "
    f"(mission {mission_length:.1f} m), tf={TF:.1f} s, R={TURN_RADIUS:.1f} m"
)
print(
    f"  solve warmup={warm_s:.2f} s, refine={refine_s:.2f} s, total={warm_s + refine_s:.2f} s"
)
print(f"  min body-edge margin to path {min(path_errors):.3f} m")
print(f"  terminal position error {np.linalg.norm(traj.x[0:2, -1] - X_GOAL):.3f} m")

fig, ax = track.plot(show=False, bounds=PLOT_BOUNDS, title="90° turn + bicycle trajopt")
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
    c=["C1", "C4"],
    s=40,
    zorder=8,
)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
plt.show()

planner.plot_solution(signals=("x", "u"))
sys.traj = traj
sys.animate()
