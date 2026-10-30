"""MPPI laps the UdeS racecar circuit around a cone: the path-integral planner in the MPC loop."""

import numpy as np

from minilink import PlanningProblem, QuadraticCost, UdeSRacecar
from minilink.control import ModelPredictiveController
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
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
from minilink.planning.trajectory_optimization.mpc import (
    mpc_animation_overlays,
)

V_REF = 3.5  # [m/s]
TF = 10.0  # closed-loop run, s
DT = 0.1  # control period, s
HORIZON = 1.0  # MPPI horizon, s (10 periods)
SIM_DT = 0.01  # plant integration step inside one control period
K = 1024  # sampled input sequences per tick
LAMBDA = 0.05  # temperature of the softmin
SIGMA = (0.5, 0.2)  # std of the input noise: speed [m/s], steer [rad]
ALPHA = 1.0  # 1 turns the information-theoretic control term off
FAN_TICKS = (5, 20, 40, 60)  # ticks whose sampled futures the track figure shows
FAN_SAMPLES = 48  # samples drawn in each fan of the figure
ANIMATION_SAMPLES = 32  # sampled futures drawn at every tick of the animation

path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
scene = Scene(obstacles=[Sphere((0.0, 2.0), 0.15)])

car = UdeSRacecar()
car.camera_follow_frame = None
car.camera_scale = 4.0
car.inputs["u"].lower_bound = np.array([0.0, -0.52])
car.inputs["u"].upper_bound = np.array([5.0, 0.52])
car.inputs["u"].nominal_value = np.array([V_REF, 0.0])  # the plan starts at speed
start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])
car.x0 = np.array(
    [start[0] - 0.1 * np.sin(heading), start[1] + 0.1 * np.cos(heading), heading]
)

# The cost: speed and steering near the reference, the corridor, the cone
probe = mount(point_probe(), on=car)
quad = QuadraticCost.from_system(
    car, Q=np.zeros((3, 3)), R=np.diag([2.0, 1.0]), ubar=np.array([V_REF, 0.0])
)
corridor = track.corridor_field(probe).as_cost(weight=20.0, shaping=quadratic_hinge())
cone = scene.clearance_field(probe).as_cost(
    weight=40.0, shaping=quadratic_hinge(threshold=0.2)
)
problem = PlanningProblem(
    sys=car, x_start=car.x0, cost=quad + corridor + cone, tf=HORIZON, U=car.input_box
)

planner = MPPIPlanner(
    problem,
    dt=DT,
    n_samples=K,
    temperature=LAMBDA,
    noise=np.square(SIGMA),
    control_cost=ALPHA,
)
mpc = ModelPredictiveController(planner, delay=False, verbose=True)
loop = (mpc % DT) @ car
run = loop.compute_trajectory(tf=TF, dt=SIM_DT)
loop.plot_trajectory()
# loop.animate()

# The animation: at each tick, the sampled futures shaded by weight and the plan
overlays = mpc_animation_overlays(
    run, mpc, loop=loop, scene=scene, track=track, samples=ANIMATION_SAMPLES
)
loop.animate(run, overlays=overlays)

# The 3-D look: the racecar skin, the camera on the body
car.camera_follow_frame = "body"
car.skin = racecar_skin_3d
loop.animate(run, overlays=overlays, renderer="meshcat", native=False)
