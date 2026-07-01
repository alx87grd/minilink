"""Pedagogical guide: waypoints and obstacles → MPC problem (no solve).

Walks through the ``planning/spatial`` pipeline for a rate-input dynamic bicycle.
Scenario matches ``demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py``.

1. **Workspace geometry** — rounded-rectangle loop, :class:`ReferenceTrack`, sphere obstacles.
2. **Planner plant** — minimal setup for ``bind``.
3. **Workspace cost heatmaps** — ``point_probe`` raster (world frame).
4. **Collision body** — ``bind(sys, car_outline)``.
5. **State fields** — ``value(x)`` from scene/track queries.
6. **Exports** — ``as_constraint()`` and ``as_cost(shaping=...)``.
7. **Assembly** — :class:`PlanningProblem` + direct collocation (not solved).

Set ``SHOW_PLOTS = False`` for a headless walkthrough.

Run from repo root::

    python examples/scripts/mpc/demo_mpc_spatial_scene_guide.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline, point_probe
from minilink.planning.spatial.grid import pad_bounds, sample_field_costs
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.plotting import plot_cost_field_exports
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

TRACK_CENTER = (0.0, 0.0)
TRACK_WIDTH = 24.0
TRACK_HEIGHT = 14.0
TURN_RADIUS = 3.5
CORRIDOR_HALF_WIDTH = 2.0
UNROLL_LAPS = 3

OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.05
OBSTACLE_CENTERS = (
    (4.0, -6.2),
    (-2.0, 5.0),
    (10.0, 1.5),
)

PATH_COST_WEIGHT = 40.0
CORRIDOR_COST_WEIGHT = 25.0
OBSTACLE_REPULSION_WEIGHT = 3.0
OBSTACLE_REPULSION_EPS = 0.08
U_TARGET = 20.0
VX0 = 2.5

MPC_HORIZON = 1.5
MPC_STEPS = 15
MPC_MAXITER = 120
MPC_FTOL = 1.0

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

PLOT_MARGIN = 3.0
COST_FIELD_MARGIN = 4.0
SHOW_PLOTS = True


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


def _section(title: str) -> None:
    print(f"\n=== {title} ===")


def _print_field(name: str, value: float) -> None:
    print(f"  {name} = {value:.4f}")


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

_section("Step 1: workspace geometry")
loop_track = ReferenceTrack(
    from_waypoints(LOOP_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
track = ReferenceTrack(
    from_waypoints(REFERENCE_WAYPOINTS), half_width=CORRIDOR_HALF_WIDTH
)
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS)
)
sample_p = START_XY + np.array([2.0, 0.5])
print(f"  lap length = {loop_track.path.total_length:.2f} m")
_print_field("loop_track.distance(sample)", float(loop_track.distance(sample_p)))
_print_field("scene.clearance(sample)", float(scene.clearance(sample_p)))

_section("Step 2: planner plant")
configure_jax(enable_x64=True)
sys_mpc = JaxDynamicBicycleRateInputs()
sys_mpc.state.lower_bound[6] = 0.0
sys_mpc.state.upper_bound[6] = W_REAR_MAX
sys_mpc.state.lower_bound[7] = -DELTA_MAX
sys_mpc.state.upper_bound[7] = DELTA_MAX
sys_mpc.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
sys_mpc.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
sys_mpc.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
sys_mpc.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX
print(f"  state dim n={sys_mpc.n}, input dim m={sys_mpc.m}")

_section("Step 3: workspace cost heatmaps (point_probe)")
probe = bind(sys_mpc, point_probe())
path_cost_viz = track.distance_field(probe).as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)
corridor_cost_viz = track.corridor_field(probe).as_cost(
    weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
)
obstacle_cost_viz = scene.clearance_field(probe).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost_bounds = pad_bounds(PLOT_BOUNDS, COST_FIELD_MARGIN - PLOT_MARGIN)
_cost_kw = dict(bounds=cost_bounds, state_dim=sys_mpc.n, grid=(120, 120))
layers = {
    "path": sample_field_costs([path_cost_viz], **_cost_kw),
    "corridor": sample_field_costs([corridor_cost_viz], **_cost_kw),
    "obstacle": sample_field_costs([obstacle_cost_viz], **_cost_kw),
    "combined": sample_field_costs(
        [path_cost_viz, corridor_cost_viz, obstacle_cost_viz], **_cost_kw
    ),
}
print("  sample_field_costs rasterizes workspace penalties before state-space bind")

_section("Step 4: collision body via bind(sys, car_outline)")
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
r_r = sys_mpc.params["r_r"]
w_rear_ref = U_TARGET / r_r
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
ubar = np.zeros(sys_mpc.m)
s_start, _ = loop_track.path.project(START_XY)
tangent = loop_track.path.tangent(s_start)
theta0 = float(np.arctan2(tangent[1], tangent[0]))
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([START_XY[0], START_XY[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

_section("Step 5: state fields — value(x) at start pose")
clearance_field = scene.clearance_field(body)
corridor_field = track.corridor_field(body)
distance_field = track.distance_field(body)
_print_field("clearance_field.value(x0)", float(clearance_field.value(x0)))
_print_field("corridor_field.value(x0)", float(corridor_field.value(x0)))
_print_field("distance_field.value(x0)", float(distance_field.value(x0)))

_section("Step 6: hard constraints — field.as_constraint()")
X = (
    BoxSet.from_system_state(sys_mpc)
    & clearance_field.as_constraint(lower=0.0)
    & corridor_field.as_constraint(lower=0.0)
)
print("  X = state_bounds & obstacle_free & corridor_tube")

_section("Step 7: soft costs — field.as_cost(shaping=...)")
stability_cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
    R=np.diag([1.0, 22.0]),
    S=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
    xbar=x_cruise,
    ubar=ubar,
)
path_cost = distance_field.as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)
corridor_cost = corridor_field.as_cost(
    weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
)
obstacle_cost = clearance_field.as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = stability_cost + path_cost + corridor_cost + obstacle_cost

_section("Step 8: PlanningProblem")
problem = PlanningProblem(sys=sys_mpc, x_start=x0, X=X, cost=cost)
print("  MPC lap demo uses soft costs only: PlanningProblem(sys, x_start=x, cost=cost)")

_section("Step 9: direct collocation planner (not solved)")
transcription = DirectCollocationTranscription(
    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=transcription,
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        solve_disp=False,
        optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
    ),
)
print(f"  horizon={MPC_HORIZON}s, n_steps={MPC_STEPS}")
print("  Next: planner.compute_solution(...) or MPCPlanner closed loop in the notebook")

if SHOW_PLOTS:
    _, ax_scene = scene.plot(
        show=False,
        bounds=PLOT_BOUNDS,
        title="",
        show_density=False,
    )
    loop_track.plot(show=False, ax=ax_scene, bounds=PLOT_BOUNDS, title="")
    ax_scene.set_title("Scene + reference track")
    plt.tight_layout()
    plt.show()

    plot_cost_field_exports(
        layers,
        track=loop_track,
        scene=scene,
        overlay_bounds=PLOT_BOUNDS,
        log_scale=True,
        show=True,
    )

print(
    "\nDone — MPC problem assembled. See demo_mpc_spatial_scene_guide.ipynb for NLP + MPC solve."
)
