"""Assemble-only spatial MPC problem (wide circuit). No solve.

Walks the ``planning/spatial`` pipeline: track + obstacles → fields → costs →
``PlanningProblem`` + ``TrajectoryOptimizationPlanner`` (not solved).

Full NLP + hybrid MPC: ``examples/notebooks/demo_mpc_circuit.ipynb``.
Closed-loop hybrid script: ``examples/scripts/mpc/demo_mpc_circuit.py``.

Set ``SHOW_PLOTS = False`` for a headless walkthrough.

Run from repo root::

    python examples/scripts/mpc/demo_mpc_spatial.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRatePorts,
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
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

SHOW_PLOTS = True
U_TARGET = 20.0
VX0 = 5.0
MPC_HORIZON = 2.0
MPC_STEPS = 10
CIRCUIT_WIDTH = 34.0
CIRCUIT_HEIGHT = 20.0
RADIUS_FAST = 5.0
RADIUS_SLOW = 3.5
CORRIDOR_HALF_WIDTH = 2.5
PATH_COST_WEIGHT = 40.0
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
OBSTACLE_REPULSION_WEIGHT = 35.0
OBSTACLE_REPULSION_EPS = 0.08
PLOT_MARGIN = 3.0


def _quarter_arc(cx, cy, radius, angle_start, n=10):
    angles = np.linspace(angle_start, angle_start + np.pi / 2, n)
    return np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles)])


def _join(*parts):
    out = np.asarray(parts[0], dtype=float)
    for part in parts[1:]:
        seg = np.asarray(part, dtype=float)
        if seg.size == 0:
            continue
        out = np.vstack([out, seg[1:]])
    return out


def wide_circuit_waypoints(
    *, cx=0.0, cy=0.0, width=34.0, height=20.0, r_fast=5.0, r_slow=3.5
):
    w2, h2 = width / 2.0, height / 2.0
    y_bot, y_top = cy - h2, cy + h2
    x_left, x_right = cx - w2, cx + w2
    r_br = r_tr = r_bl = r_fast
    r_tl = r_slow
    hx_br = x_right - r_br
    hx_tl = x_left + r_tl
    start = np.array([[x_left + 3.0, y_bot]])
    launch = np.linspace(start[0], [x_left + 10.0, y_bot], 4)[1:]
    chicane = np.array(
        [
            [x_left + 12.0, y_bot],
            [x_left + 14.0, y_bot + 1.8],
            [x_left + 16.0, y_bot],
            [hx_br - 1.5, y_bot],
        ]
    )
    arc_br = _quarter_arc(hx_br, y_bot + r_br, r_br, -np.pi / 2)[1:]
    right = np.linspace([x_right, y_bot + r_br], [x_right, y_top - r_tr], 5)[1:-1]
    arc_tr = _quarter_arc(hx_br, y_top - r_tr, r_tr, 0.0)[1:]
    top = np.linspace([hx_br, y_top], [hx_tl + 3.0, y_top], 5)[1:-1]
    arc_tl = _quarter_arc(hx_tl, y_top - r_tl, r_tl, np.pi / 2)[1:]
    left = np.linspace([x_left, y_top - r_tl], [x_left, y_bot + r_bl], 5)[1:-1]
    arc_bl = _quarter_arc(x_left + r_bl, y_bot + r_bl, r_bl, np.pi)[1:]
    return _join(
        start, launch, chicane, arc_br, right, arc_tr, top, arc_tl, left, arc_bl, start
    )


loop_xy = wide_circuit_waypoints(
    width=CIRCUIT_WIDTH,
    height=CIRCUIT_HEIGHT,
    r_fast=RADIUS_FAST,
    r_slow=RADIUS_SLOW,
)
track = ReferenceTrack(from_waypoints(loop_xy), half_width=CORRIDOR_HALF_WIDTH)
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(obstacles=[Sphere(c, keepout_radius) for c in OBSTACLE_CENTERS])
start_xy = loop_xy[0].copy()
plot_bounds = (
    (loop_xy[:, 0].min() - PLOT_MARGIN, loop_xy[:, 0].max() + PLOT_MARGIN),
    (loop_xy[:, 1].min() - PLOT_MARGIN, loop_xy[:, 1].max() + PLOT_MARGIN),
)

print(f"lap length = {track.path.total_length:.2f} m")

sys_mpc = BicycleDynRatePorts()
sys_mpc.state.lower_bound[6] = 0.0
sys_mpc.state.upper_bound[6] = 90.0
sys_mpc.state.lower_bound[7] = -0.55
sys_mpc.state.upper_bound[7] = 0.55
sys_mpc.inputs["w_rear_dot"].lower_bound[0] = -80.0
sys_mpc.inputs["w_rear_dot"].upper_bound[0] = 80.0
sys_mpc.inputs["delta_dot"].lower_bound[0] = -2.0
sys_mpc.inputs["delta_dot"].upper_bound[0] = 2.0

probe = bind(sys_mpc, point_probe())
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
r_r = sys_mpc.params["r_r"]
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
s_start, _ = track.path.project(start_xy)
tangent = track.path.tangent(s_start)
theta0 = float(np.arctan2(tangent[1], tangent[0]))
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])

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

X = (
    BoxSet.from_system_state(sys_mpc)
    & scene.clearance_field(body).as_constraint(lower=0.0)
    & track.corridor_field(body).as_constraint(lower=0.0)
)
problem = PlanningProblem(sys=sys_mpc, x_start=x0, X=X, cost=cost, tf=MPC_HORIZON)
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    solve_disp=False,
    optimizer_options={"maxiter": 150, "ftol": 0.1},
)
print(f"planner ready (not solved): horizon={MPC_HORIZON}s, n_steps={MPC_STEPS}")
print(
    "Next: demo_mpc_circuit.ipynb (full stack) or demo_mpc_circuit.py (hybrid script)"
)

if SHOW_PLOTS:
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
    cost_bounds = pad_bounds(plot_bounds, 3.0)
    kw = dict(bounds=cost_bounds, state_dim=sys_mpc.n, grid=(120, 120))
    layers = {
        "path": sample_field_costs([path_viz], **kw),
        "corridor": sample_field_costs([corridor_viz], **kw),
        "obstacle": sample_field_costs([obstacle_viz], **kw),
        "combined": sample_field_costs([path_viz, corridor_viz, obstacle_viz], **kw),
    }
    _, ax = scene.plot(show=False, bounds=plot_bounds, title="", show_density=False)
    track.plot(show=False, ax=ax, bounds=plot_bounds, title="")
    ax.set_title("Wide circuit + keepouts")
    plt.tight_layout()
    plt.show()
    plot_cost_field_exports(
        layers,
        track=track,
        scene=scene,
        overlay_bounds=plot_bounds,
        log_scale=True,
        show=True,
    )
