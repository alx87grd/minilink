"""Hybrid MPC on the kinematic 1/10 racecar: a small circuit, one cone."""

import numpy as np

from minilink import (
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
    UdeSRacecar,
)
from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    bind,
    circuit_waypoints,
    from_waypoints,
    point_probe,
    quadratic_hinge,
)

V_REF = 3.5  # [m/s]
TF = 10.0
MPC_DT = 0.1
MPC_HORIZON = 1.0

# --- circuit: a rounded rectangle and one cone on the far straight ---
path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
scene = Scene(obstacles=[Sphere((0.0, 2.0), 0.15)])

# --- plant: kinematic bicycle, inputs are speed and steer ---
car = UdeSRacecar()
# car.skin = racecar_skin_3d
car.camera_follow_frame = None
car.camera_scale = 4.0
car.inputs["u"].lower_bound = np.array([0.0, -0.52])
car.inputs["u"].upper_bound = np.array([5.0, 0.52])

# start a little off the line: path distance has no derivative on it
start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])
x0 = np.array(
    [start[0] - 0.1 * np.sin(heading), start[1] + 0.1 * np.cos(heading), heading]
)
car.x0 = x0

# --- cost: cruise, stay in the lane, miss the cone ---
probe = bind(car, point_probe())
quad = QuadraticCost.from_system(
    car, Q=np.zeros((3, 3)), R=np.diag([2.0, 1.0]), ubar=np.array([V_REF, 0.0])
)
corridor = track.corridor_field(probe).as_cost(weight=20.0, shaping=quadratic_hinge())
obstacle = scene.clearance_field(probe).as_cost(
    weight=40.0, shaping=quadratic_hinge(threshold=0.2)
)
cost = quad + corridor + obstacle

# --- MPC: receding-horizon collocation on that cost ---
planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=car, x_start=x0, cost=cost, tf=MPC_HORIZON),
    n_steps=8,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 40, "ftol": 0.05},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, verbose=True)

# --- closed loop ---
diagram = mpc @ car

diagram.plot_diagram()
result = diagram.compute_trajectory(
    tf=TF, x0_plant=x0, plant_dt_inner=0.01, compile_backend="jax"
)
diagram.plot_trajectory()
diagram.animate(
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track),
    # renderer="meshcat",
)

# --- 3-D look: same lap ---
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d

car.skin = racecar_skin_3d

diagram.animate(
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track),
    renderer="meshcat",
    native=True,
)
