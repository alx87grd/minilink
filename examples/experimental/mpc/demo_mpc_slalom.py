"""Hybrid MPC slalom: straight lane + staggered keepout spheres.

Tracking cost on ``y=0`` plus soft clearance from a ``Scene`` of four
obstacles. Closed loop via ``ModelPredictiveController`` and ``mpc @ plant``.

Run from repo root::

    python examples/experimental/mpc/demo_mpc_slalom.py
"""

import numpy as np

from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
)
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, point_probe
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- knobs ---
U_TARGET = 10.0
VX0 = 6.4
TF_SIM = 10.0
MPC_DT = 0.2
SIM_DT = 0.005
MPC_HORIZON = 2.0
MPC_STEPS = 20
REF_X_PAD = 20.0

OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.2
OBSTACLE_CENTERS = (
    (12.0, 0.5),
    (20.0, -0.4),
    (28.0, 0.6),
    (36.0, -0.3),
)
OBSTACLE_REPULSION_WEIGHT = 20.0
OBSTACLE_REPULSION_EPS = 0.08

keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
scene = Scene(obstacles=[Sphere(c, keepout_radius) for c in OBSTACLE_CENTERS])

sys_mpc = BicycleDynRate()
sys_mpc.state.lower_bound[6] = 0.0
sys_mpc.state.upper_bound[6] = 90.0
sys_mpc.state.lower_bound[7] = -0.55
sys_mpc.state.upper_bound[7] = 0.55
sys_mpc.inputs["u"].lower_bound = np.array([-80.0, -2.0])
sys_mpc.inputs["u"].upper_bound = np.array([80.0, 2.0])

r_r = sys_mpc.params["r_r"]
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
x0 = np.array([0.0, 3.0, 0.0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])
body = bind(sys_mpc, point_probe())
cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    R=np.diag([1.0, 25.0]),
    S=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    xbar=x_ref,
    ubar=np.zeros(2),
) + scene.clearance_field(body).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 100, "ftol": 0.1},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, step_disp=True)

sys_sim = BicycleDynRate()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
sys_sim.camera_scale = 12.0
sys_sim.x0 = x0.copy()

hybrid = mpc @ sys_sim
hybrid.plot_diagram()
result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0,
    plant_dt_inner=SIM_DT,
    compile_backend="jax",
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(
        result, planner, scene=scene, reference_pad=REF_X_PAD
    )
)
