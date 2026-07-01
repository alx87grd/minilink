"""Pedagogical guide: waypoints and obstacles → MPC problem (no solve).

Walks through the ``planning/spatial`` pipeline for a rate-input dynamic bicycle:

1. **Workspace geometry** — waypoint polyline, :class:`ReferenceTrack`, sphere obstacles,
   :class:`Scene`.
2. **Collision body** — ``bind(sys, geometry)`` maps state ``x`` to workspace probes
   (``point_probe``, ``disc``, ``car_outline``).
3. **State fields** — scalar ``value(x)`` from scene/track queries fused over the body.
4. **Exports** — ``as_constraint()`` for hard feasible sets, ``as_cost(shaping=...)``
   for soft penalties (:func:`~minilink.planning.spatial.shaping.quadratic_excess`,
   :func:`~minilink.planning.spatial.shaping.quadratic_hinge`,
   :func:`~minilink.planning.spatial.shaping.inverse_barrier`,
   :func:`~minilink.planning.spatial.shaping.occupancy`).
5. **Assembly** — compose ``X`` and ``cost``, build :class:`PlanningProblem`,
   wire direct collocation — then stop before the MPC loop.

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
    occupancy,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.spatial.workspace_fields import GaussianField
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# --- Teaching scenario (compact loop + a few obstacles) ---
WAYPOINTS = np.array(
    [
        [-8.0, -5.0],
        [8.0, -5.0],
        [12.0, 0.0],
        [8.0, 5.0],
        [-8.0, 5.0],
        [-12.0, 0.0],
        [-8.0, -5.0],
    ]
)
CORRIDOR_HALF_WIDTH = 2.0
OBSTACLE_RADIUS = 0.35
OBSTACLE_MARGIN = 0.15
OBSTACLE_CENTERS = ((-4.0, -4.0), (4.0, 0.0), (0.0, 4.0))
TERRAIN_CENTER = (2.0, 2.0)

PATH_COST_WEIGHT = 30.0
CORRIDOR_COST_WEIGHT = 20.0
OBSTACLE_REPULSION_WEIGHT = 25.0
TERRAIN_COST_WEIGHT = 5.0
U_TARGET = 12.0

MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 120
MPC_FTOL = 1e-1

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

PLOT_MARGIN = 2.0
COST_FIELD_MARGIN = 4.0
SHOW_PLOTS = True


def _section(title: str) -> None:
    print(f"\n=== {title} ===")


def _print_field(name: str, value: float) -> None:
    print(f"  {name} = {value:.4f}")


# ---------------------------------------------------------------------------
# Step 1 — Planner plant (rate-input bicycle used inside MPC transcription)
# ---------------------------------------------------------------------------
_section("Step 1: planner plant")
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

# ---------------------------------------------------------------------------
# Step 2 — Waypoints → ReferencePath → ReferenceTrack
# ---------------------------------------------------------------------------
_section("Step 2: reference track from waypoints")
path = from_waypoints(WAYPOINTS)
track = ReferenceTrack(path, half_width=CORRIDOR_HALF_WIDTH)
print(f"  centerline length = {path.total_length:.2f} m")
print(f"  corridor half-width = {track.half_width:.2f} m")

sample_p = np.array([0.0, -4.0])
_print_field("track.distance(sample)", float(track.distance(sample_p)))
_print_field(
    "track.corridor_margin(sample)",
    float(track.corridor_margin(sample_p)),
)

# ---------------------------------------------------------------------------
# Step 3 — Obstacles (+ optional soft terrain) → Scene
# ---------------------------------------------------------------------------
_section("Step 3: scene — hard obstacles and soft workspace fields")
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS),
    workspace_fields=(GaussianField(TERRAIN_CENTER, amplitude=1.0, sigma=2.5),),
)
_print_field("scene.clearance(sample)", float(scene.clearance(sample_p)))
_print_field("scene.cost_density(sample)", float(scene.cost_density(sample_p)))
print("  Scene factories:")
print("    scene.clearance_field(body)  → min body clearance to obstacles")
print("    scene.cost_field(body)       → max body exposure to terrain density")

# ---------------------------------------------------------------------------
# Step 4 — Collision body: state x → workspace probes
# ---------------------------------------------------------------------------
_section("Step 4: collision body via bind(sys, geometry)")
body = bind(sys_mpc, car_outline(length=2.2, width=0.2, margin=0.05))
probe = bind(sys_mpc, point_probe())
print("  car_outline — MPC planner body (oriented footprint)")
print("  point_probe — workspace heatmaps (position only, no heading)")

r_r = sys_mpc.params["r_r"]
w_rear_ref = U_TARGET / r_r
x_cruise = np.array([0.0, -4.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
ubar = np.zeros(sys_mpc.m)

# ---------------------------------------------------------------------------
# Step 5 — State fields: scalar value(x) over the body
# ---------------------------------------------------------------------------
_section("Step 5: state fields — value(x) at cruise pose")
clearance_field = scene.clearance_field(body)
corridor_field = track.corridor_field(body)
distance_field = track.distance_field(body)
terrain_field = scene.cost_field(body)
_print_field("clearance_field.value(x)", float(clearance_field.value(x_cruise)))
_print_field("corridor_field.value(x)", float(corridor_field.value(x_cruise)))
_print_field("distance_field.value(x)", float(distance_field.value(x_cruise)))
_print_field("terrain_field.value(x)", float(terrain_field.value(x_cruise)))

# ---------------------------------------------------------------------------
# Step 6 — Hard feasible set: field.as_constraint()
# ---------------------------------------------------------------------------
_section("Step 6: hard constraints — field.as_constraint()")
X = (
    BoxSet.from_system_state(sys_mpc)
    & clearance_field.as_constraint(lower=0.0)
    & corridor_field.as_constraint(lower=0.0)
)
print("  X = state_bounds & obstacle_free & corridor_tube")
print("  margin(z) > 0  ↔  feasible along a trajectory node")

# ---------------------------------------------------------------------------
# Step 7 — Soft costs: field.as_cost(shaping=...)
# ---------------------------------------------------------------------------
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
    weight=PATH_COST_WEIGHT,
    shaping=quadratic_excess(threshold=0.1),
)
corridor_cost = corridor_field.as_cost(
    weight=CORRIDOR_COST_WEIGHT,
    shaping=quadratic_hinge(threshold=0.0),
)
obstacle_cost = clearance_field.as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=0.08),
)
terrain_cost = terrain_field.as_cost(
    weight=TERRAIN_COST_WEIGHT,
    shaping=occupancy(scale=1.0),
)
print("  quadratic_excess  — penalize path distance above threshold")
print("  quadratic_hinge   — penalize leaving the corridor tube")
print("  inverse_barrier   — blow up as clearance → 0")
print("  occupancy         — bounded [0,1] terrain score")

cost = stability_cost + path_cost + corridor_cost + obstacle_cost + terrain_cost
print("  cost = stability + path + corridor + obstacle + terrain  (CostFunction +)")

# Separate viz-only costs (point probe, for workspace heatmaps)
path_cost_viz = track.distance_field(probe).as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)
corridor_cost_viz = track.corridor_field(probe).as_cost(
    weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
)
obstacle_cost_viz = scene.clearance_field(probe).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT, shaping=inverse_barrier(epsilon=0.08)
)

# ---------------------------------------------------------------------------
# Step 8 — PlanningProblem (MPC passes x_start each tick; no x_goal here)
# ---------------------------------------------------------------------------
_section("Step 8: PlanningProblem")
problem = PlanningProblem(sys=sys_mpc, x_start=x_cruise, X=X, cost=cost)
print("  Receding-horizon MPC uses: PlanningProblem(sys, x_start=x, cost=cost)")
print("  Hard X is optional; this guide includes it to show the full export path.")

# ---------------------------------------------------------------------------
# Step 9 — Transcription + planner (assembled, not solved)
# ---------------------------------------------------------------------------
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
print("  Next step in a full MPC demo: planner.compute_solution(initial_guess=...)")

# ---------------------------------------------------------------------------
# Step 10 — Workspace cost heatmaps (point probe, heading-independent)
# ---------------------------------------------------------------------------
_section("Step 10: workspace cost raster for tuning")
PLOT_BOUNDS = (
    (WAYPOINTS[:, 0].min() - PLOT_MARGIN, WAYPOINTS[:, 0].max() + PLOT_MARGIN),
    (WAYPOINTS[:, 1].min() - PLOT_MARGIN, WAYPOINTS[:, 1].max() + PLOT_MARGIN),
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
print("  sample_field_costs rasterizes FieldCost terms at workspace (x, y)")
print("  plot_cost_field_exports → per-layer 2D heatmaps + 3D surfaces")

if SHOW_PLOTS:
    _, ax_scene = scene.plot(
        show=False,
        bounds=PLOT_BOUNDS,
        title="Scene: obstacles + terrain density",
        show_density=True,
    )
    track.plot(show=False, ax=ax_scene, bounds=PLOT_BOUNDS, title="")
    ax_scene.set_title("Scene + reference track")
    plt.tight_layout()

    plot_cost_field_exports(
        layers,
        track=track,
        scene=scene,
        overlay_bounds=PLOT_BOUNDS,
        log_scale=True,
        show=True,
    )

print(
    "\nDone — MPC problem assembled. See demo_mpc_spatial_scene_guide.ipynb for MPC solve."
)
