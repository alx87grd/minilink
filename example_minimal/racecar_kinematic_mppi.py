"""Racecar MPPI: a plan sampled on the kinematic model drives the dynamic car through an inner speed loop."""

import numpy as np

from minilink import (
    PI,
    DiagramSystem,
    PlanningProblem,
    QuadraticCost,
    UdeSRacecar,
    UdeSRacecarDyn,
)
from minilink.blocks import Demux, Gain
from minilink.control import ModelPredictiveController
from minilink.graphical.catalog import racecar_skin_3d
from minilink.planning import (
    MPPIPlanner,
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

########################################################
### Tunables
########################################################

V_CRUISE = 3.0  # [m/s]
TF = 8.0  # [s]
N_FUTURES = 32

########################################################
### Track and scene
########################################################

path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
scene = Scene(obstacles=[Sphere((0.0, 2.0), 0.15)])

########################################################
### Kinematic model for planning
########################################################

model = UdeSRacecar()
model.inputs["u"].lower = np.array([0.0, -0.52])
model.inputs["u"].upper = np.array([5.0, 0.52])
model.inputs["u"].nominal_value = np.array([V_CRUISE, 0.0])

########################################################
### Cost and initial pose
########################################################

centre = mount(point_probe(), on=model)

cost = (
    QuadraticCost.from_system(
        model,
        Q=np.zeros((3, 3)),
        R=np.diag([2.0, 1.0]),
        u_bar=np.array([V_CRUISE, 0.0]),
    )
    + track.corridor_field(centre).as_cost(weight=20.0, shaping=quadratic_hinge())
    + scene.clearance_field(centre).as_cost(
        weight=40.0, shaping=quadratic_hinge(threshold=0.2)
    )
)

heading = np.arctan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
x0 = np.array(
    [path[0, 0] - 0.1 * np.sin(heading), path[0, 1] + 0.1 * np.cos(heading), heading]
)

########################################################
### MPPI planner
########################################################

planner = MPPIPlanner(
    PlanningProblem(model, x0=x0, cost=cost, U=model.input_box, horizon=1.0),
    dt=0.1,
    n_rollouts=2048,
    temperature=0.05,
    noise=np.square([0.3, 0.15]),
    control_cost=1.0,
)

########################################################
### SQP MPC (commented)
########################################################

# from minilink import TrajectoryOptimizationPlanner

# planner = TrajectoryOptimizationPlanner(
#     PlanningProblem(model, x0=x0, cost=cost, U=model.input_box, horizon=1.0),
#     n_steps=8,
#     backend="jax",
#     method="slsqp",
#     options={"maxiter": 40, "ftol": 0.01},
# )

mpc = ModelPredictiveController(planner, delay=False)

########################################################
### Dynamic car and inner speed loop
########################################################

car = UdeSRacecarDyn(named_ports=True)
car.x0[0:3] = x0
car.skin = racecar_skin_3d

speed_loop = PI(Kp=40.0, Ki=40.0, ports="reference", u_min=-80.0, u_max=80.0)

inner_loop = DiagramSystem()
inner_loop.add_subsystem(car, "car")
inner_loop.add_subsystem(speed_loop, "speed_loop")
inner_loop.add_subsystem(Demux([1, 1]), "commands")
inner_loop.add_subsystem(Gain(np.eye(9)[:3]), "pose")
inner_loop.add_input_port("u", dim=2, nominal_value=np.array([V_CRUISE, 0.0]))
inner_loop.connect("input", "u", "commands", "u")
inner_loop.connect("commands", "u[0]", "speed_loop", "r")
inner_loop.connect("car", "speed", "speed_loop", "y")
inner_loop.connect("speed_loop", "u", "car", "P_cmd")
inner_loop.connect("commands", "u[1]", "car", "delta_cmd")
inner_loop.connect("car", "y", "pose", "u")
inner_loop.connect_new_output_port("pose", "y", "y")
# inner_loop.plot_diagram()

########################################################
### Closed loop and lap animation
########################################################

closed_loop = (mpc % 0.1) @ inner_loop
closed_loop.plot_diagram()

traj = closed_loop.compute_trajectory(tf=TF, dt=0.01)

lap = traj.of("plant")
inner_loop.plot_trajectory(lap)

overlays = mpc_animation_overlays(
    traj,
    mpc,
    loop=closed_loop,
    scene=scene,
    track=track,
    samples=N_FUTURES,
)
inner_loop.animate(lap, overlays=overlays, renderer="meshcat")
