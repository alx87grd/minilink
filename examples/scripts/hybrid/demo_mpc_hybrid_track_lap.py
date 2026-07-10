"""Minimal hybrid MPC on a compact track with obstacles.

Same rounded-rectangle loop and sphere scene as
``demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py``, using warm-start
``mpc % schedule`` and ``computer @ plant``.

Run from repo root::

    python examples/scripts/hybrid/demo_mpc_hybrid_track_lap.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputsUY,
)
from minilink.planning.mpc import (
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    mpc_animation_overlays,
    mpc_stateful_controller,
)
from minilink.planning.mpc.warm_start import mpc_default_computer_x0
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline
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

configure_jax(enable_x64=True)

U_TARGET = 20.0
VX0 = 2.5
TF_SIM = 16.0
MPC_DT = 0.2
SIM_DT = 0.005
OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.05
OBSTACLE_CENTERS = ((4.0, -6.2), (-2.0, 5.0), (10.0, 1.5))


def _rounded_rect_loop(cx=0.0, cy=0.0, width=24.0, height=14.0, radius=3.5):
    w2, h2, r = width / 2.0, height / 2.0, radius
    hx = w2 - r

    def arc(ccx, ccy, a0):
        t = np.linspace(a0, a0 + np.pi / 2, 8)
        return np.column_stack([ccx + r * np.cos(t), ccy + r * np.sin(t)])

    pts = [
        [[cx - hx, cy - h2]],
        np.linspace([cx - hx, cy - h2], [cx + hx, cy - h2], 4)[1:],
        arc(cx + hx, cy - h2 + r, -np.pi / 2)[1:],
        np.linspace([cx + w2, cy - h2 + r], [cx + w2, cy + h2 - r], 3)[1:-1],
        arc(cx + hx, cy + h2 - r, 0.0)[1:],
        np.linspace([cx + hx, cy + h2], [cx - hx, cy + h2], 4)[1:-1],
        arc(cx - hx, cy + h2 - r, np.pi / 2)[1:],
        np.linspace([cx - w2, cy + h2 - r], [cx - w2, cy - h2 + r], 3)[1:-1],
        arc(cx - hx, cy - h2 + r, np.pi)[1:],
        [[cx - hx, cy - h2]],
    ]
    return np.vstack(pts)


loop_xy = _rounded_rect_loop()
track = ReferenceTrack(from_waypoints(loop_xy), half_width=2.0)
keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(obstacles=[Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS])

sys_mpc = JaxDynamicBicycleRateInputsUY()
sys_sim = JaxDynamicBicycleRateInputsUY()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

for sys in (sys_mpc, sys_sim):
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = 90.0
    sys.state.lower_bound[7] = -0.55
    sys.state.upper_bound[7] = 0.55
    sys.inputs["u"].lower_bound = np.array([-80.0, -2.0])
    sys.inputs["u"].upper_bound = np.array([80.0, 2.0])
sys_sim.camera_scale = 16.0

r_r = sys_mpc.params["r_r"]
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
cost = (
    QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        R=np.diag([1.0, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    )
    + track.distance_field(body).as_cost(
        weight=40.0, shaping=quadratic_excess(threshold=0.1)
    )
    + track.corridor_field(body).as_cost(
        weight=25.0, shaping=quadratic_hinge(threshold=0.0)
    )
    + scene.clearance_field(body).as_cost(
        weight=3.0, shaping=inverse_barrier(epsilon=0.08)
    )
)

s0, _ = track.path.project(loop_xy[0])
tangent = track.path.tangent(s0)
theta0 = float(np.arctan2(tangent[1], tangent[0]))
if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
    theta0 += 1e-4
x0 = np.array([loop_xy[0, 0], loop_xy[0, 1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])
sys_sim.x0 = x0.copy()

planner = MPCPlanner(
    PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost),
    transcription=MPCDirectCollocationTranscription(
        DirectCollocationOptions(tf=2.0, n_steps=20)
    ),
    options=MPCOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 120, "ftol": 1.0},
    ),
)

mpc = mpc_stateful_controller(planner, dt_mpc=MPC_DT, step_disp=True)
computer = mpc % MPC_DT
hybrid = computer @ sys_sim

hybrid.plot_diagram()

result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0,
    x0_computer=mpc_default_computer_x0(planner),
    plant_dt_inner=SIM_DT,
    compile_backend="jax",
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track)
)
