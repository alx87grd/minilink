"""Hybrid MPC on the kinematic 1/10 racecar: a small circuit, one cone."""

import numpy as np

from minilink import (
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
    UdeSRacecar,
)
from minilink.analysis import discretize
from minilink.control import ModelPredictiveController
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    circuit_waypoints,
    from_waypoints,
    mount,
    point_probe,
    quadratic_hinge,
)
from minilink.planning.trajectory_optimization.mpc import mpc_animation_overlays

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
probe = mount(point_probe(), on=car)
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
    PlanningProblem(sys=car, x_start=x0, cost=cost, tf=MPC_HORIZON, U=car.input_box),
    n_steps=8,
    transcription="direct_collocation",
    backend="jax",
    method="scipy_slsqp",
    optimizer_options={"maxiter": 40, "ftol": 0.05},
)
mpc = ModelPredictiveController(planner)

# --- closed loop ---
diagram = (mpc % MPC_DT) @ discretize(car, MPC_DT, substeps=round(MPC_DT / 0.01))

# diagram.plot_diagram()
result = diagram.compute_trajectory(tf=TF)
# diagram.plot_trajectory()

# --- 3-D look: same lap ---
car.camera_follow_frame = "body"
car.skin = racecar_skin_3d

diagram.animate(
    overlays=mpc_animation_overlays(
        result, mpc, loop=diagram, scene=scene, track=track
    ),
    renderer="meshcat",
    native=False,
)
