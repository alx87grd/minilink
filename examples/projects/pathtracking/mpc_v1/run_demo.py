"""MPC path tracking + EngineBicycle cascade (dual-rate broadcast).

PathTracking is replaced by MPC on ``BicycleDynRate``; the cascade
(Servos → Allocation) tracks broadcast ``x_nom`` refs on the real plant.

Run from repo root::

    PYTHONPATH=. python examples/projects/bicycle_los_v3/run_demo.py
"""

import numpy as np

from examples.projects.pathtracking.mpc_v1.allocation import Allocation
from examples.projects.pathtracking.mpc_v1.mpc_dual_rate import dual_rate_computer_ahead
from examples.projects.pathtracking.mpc_v1.nominal_refs import NominalRefs
from examples.projects.pathtracking.mpc_v1.path_generator import rounded_rectangle_path
from examples.projects.pathtracking.mpc_v1.plant_to_mpc import (
    PlantToMpcState,
    plant_y_to_mpc_x,
)
from examples.projects.pathtracking.mpc_v1.servos import Servos
from examples.projects.pathtracking.mpc_v1.vehicle import create_vehicle
from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
    mpc_default_computer_x0,
)
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.shaping import quadratic_excess, quadratic_hinge
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- knobs ---
U_TARGET = 12.0
TF_SIM = 20.0
MPC_DT = 0.2
DT_BROADCAST = 0.01
DT_PROJECT = 0.2
SIM_DT = 0.01
MPC_HORIZON = 2.0
MPC_STEPS = 10
PATH_HALF_WIDTH = 5.0
PATH_COST_WEIGHT = 40.0
CORRIDOR_COST_WEIGHT = 25.0

path_xy = rounded_rectangle_path(Lx=40.0, Ly=20.0, R=7.0, nseg=2, narc=4, closed=True)
track = ReferenceTrack(from_waypoints(path_xy), half_width=PATH_HALF_WIDTH)

vehicle = create_vehicle(Y=0.0, vx=0.0, theta=np.pi, tire_slip_mode=None)
x_mpc0 = plant_y_to_mpc_x(vehicle.x0)

sys_mpc = BicycleDynRate()
sys_mpc.state.lower_bound[6] = 0.0
sys_mpc.state.upper_bound[6] = 90.0
sys_mpc.state.lower_bound[7] = -0.55
sys_mpc.state.upper_bound[7] = 0.55
sys_mpc.inputs["u"].lower_bound = np.array([-80.0, -2.0])
sys_mpc.inputs["u"].upper_bound = np.array([80.0, 2.0])

r_r = sys_mpc.params["r_r"]
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
body = bind(sys_mpc, car_outline(length=2.4, width=0.2, margin=0.05))
path_cost = track.distance_field(body).as_cost(
    weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
)
corridor_cost = track.corridor_field(body).as_cost(
    weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
)
cost = (
    QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
        R=np.diag([1.0, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.5, 4.0, 6.0, 0.1, 80.0]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    )
    + path_cost
    + corridor_cost
)

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=sys_mpc, x_start=x_mpc0, cost=cost, tf=MPC_HORIZON),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 100, "ftol": 0.1},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, step_disp=True)
computer = dual_rate_computer_ahead(
    mpc, dt_broadcast=DT_BROADCAST, dt_project=DT_PROJECT
)

plant_to_mpc = PlantToMpcState()
nominal_refs = NominalRefs()
servos = Servos(
    mass=vehicle.mass,
    max_steer=vehicle.max_steer,
    min_steer=vehicle.min_steer,
)
allocation = Allocation(
    r_r=vehicle.r_r,
    engine_power_peak=vehicle.engine_power_peak,
    transmission_ratio=vehicle.transmission_ratio,
)

plant = DiagramSystem()
plant.name = "Engine bicycle cascade"
plant.add_subsystem(vehicle, "vehicle")
plant.add_subsystem(plant_to_mpc, "plant_to_mpc")
plant.add_subsystem(nominal_refs, "nominal_refs")
plant.add_subsystem(servos, "servos")
plant.add_subsystem(allocation, "allocation")

plant.connect("vehicle", "y", "plant_to_mpc", "y")
plant.connect("vehicle", "y", "servos", "y")
plant.connect("vehicle", "y", "allocation", "y")
plant.connect("nominal_refs", "heading_ref", "servos", "heading_ref")
plant.connect("nominal_refs", "vx_ref", "servos", "vx_ref")
plant.connect("nominal_refs", "delta_ff", "servos", "delta_ff")
plant.connect("servos", "F_rear", "allocation", "F_rear")
plant.connect("servos", "delta", "allocation", "delta")
plant.connect("allocation", "throttle", "vehicle", "throttle")
plant.connect("allocation", "delta", "vehicle", "delta")

plant.add_input_port("x_nom", dim=8, nominal_value=np.zeros(8))
plant.connect("input", "x_nom", "nominal_refs", "x_nom")
plant.connect_new_output_port("plant_to_mpc", "y_mpc", "y_mpc")

plant.camera_follow_frame = "vehicle:body"
plant.camera_scale = 15.0

vehicle.x0[0] = 0.0
vehicle.x0[1] = -20.0
vehicle.x0[2] = 0.0
vehicle.x0[3] = U_TARGET
vehicle.x0[4] = 0.0
vehicle.x0[5] = 0.0
vehicle.x0[6] = U_TARGET / r_r
vehicle.x0[7] = 0.0

x0_plant = np.concatenate([vehicle.x0, servos.x0])
hybrid = hybrid_closed_loop(
    computer.diagram,
    plant,
    schedule=computer.schedule,
    computer=computer,
    computer_out="x_nom",
    plant_in="x_nom",
    plant_out="y_mpc",
    computer_in="y",
)

hybrid.plot_diagram()
result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0_plant,
    x0_computer=mpc_default_computer_x0(planner),
    plant_dt_inner=SIM_DT,
    compile_backend="numpy",
    verbose=True,
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(result, planner, track=track),
    renderer="matplotlib",
)
