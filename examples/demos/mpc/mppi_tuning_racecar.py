"""Tune MPPI's temperature for a lap of the dynamic racecar, the planner on the kinematic model."""

import numpy as np

from minilink import (
    DiagramSystem,
    Optimizer,
    PlanningProblem,
    QuadraticCost,
    UdeSRacecar,
    Uniform,
)
from minilink.blocks import Gain, Mux
from minilink.catalog import UdeSRacecarDyn
from minilink.control import ModelPredictiveController
from minilink.core.geometry import circuit_waypoints, from_waypoints
from minilink.graphical.catalog import racecar_skin_2d
from minilink.planning import (
    MPPIPlanner,
    ReferenceTrack,
    mount,
    point_probe,
    quadratic_hinge,
)
from minilink.planning.trajectory_optimization.mpc import mpc_animation_overlays

TF = 4.0  # [s] the scored run, a little more than one lap
V_REF = 2.0  # [m/s] the speed asked for
N_TRIALS = 5  # random starts the score is averaged over

# The circuit: a 6 × 4 m rounded rectangle, and a lane 0.4 m either side of its line

waypoints = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(waypoints, closed=True), half_width=0.4)

# The plant: the dynamic car behind a speed servo, P_cmd = (k_v + c_v) v_ref − k_v vx,
# so it takes the kinematic model's [speed, steer] and gives back its pose [x, y, θ]

k_v = 10.0  # [W s/m] power per m/s of speed error
c_v = 3.0  # [W s/m] power that holds a speed on the straight

car = UdeSRacecarDyn(named_ports=False)
servo = Gain(np.zeros((2, 11)))
servo.params["K"][0, 0] = k_v + c_v  # the speed command, from [v_ref, δ_ref, y]
servo.params["K"][0, 5] = -k_v  # the measured vx, y[3]
servo.params["K"][1, 1] = 1.0  # the steer command passes through
pose = Gain(np.eye(9)[:3])

plant = DiagramSystem()
plant.add_subsystem(car, "car")
plant.add_subsystem(Mux(dims=(2, 9)), "mux")
plant.add_subsystem(servo, "servo")
plant.add_subsystem(pose, "pose")
plant.expose_input("u", [("mux", "in0")])
plant.connect("car", "y", "mux", "in1")
plant.connect("mux", "y", "servo", "u")
plant.connect("servo", "y", "car", "u")
plant.connect("car", "y", "pose", "u")
plant.connect_new_output_port("pose", "y", "y")

# It sets off at speed on the first waypoint, along the path

theta_0 = np.arctan2(*(waypoints[1] - waypoints[0])[::-1])
plant.x0 = np.array(
    [
        waypoints[0, 0],
        waypoints[0, 1],
        theta_0,
        V_REF,
        0.0,
        0.0,
        V_REF / car.params["r_r"],
        0.0,
        c_v * V_REF,
    ]
)

# The model the planner believes: the kinematic car, its speed and steer boxed

model = UdeSRacecar()
model.inputs["u"].lower_bound = np.array([0.0, -0.52])
model.inputs["u"].upper_bound = np.array([4.0, 0.52])
model.inputs["u"].nominal_value = np.array([V_REF, 0.0])

# The planner's cost: hold the speed and the wheel straight, keep inside the lane

centre = mount(point_probe(), on=model)
plan_cost = QuadraticCost.from_system(
    model, Q=np.zeros((3, 3)), R=np.diag([2.0, 1.0]), u_bar=np.array([V_REF, 0.0])
) + track.corridor_field(centre).as_cost(weight=20.0, shaping=quadratic_hinge())

# MPPI, 128 sampled sequences over a 1 s horizon, from a poor start: λ = 0.236 and a
# speed noise of 1 m/s

lam = 0.236
sigma = np.array([1.0, 0.3])  # [m/s, rad]

planner = MPPIPlanner(
    PlanningProblem(
        sys=model, x0=plant.x0[:3], cost=plan_cost, horizon=1.0, U=model.input_box
    ),
    dt=0.1,
    n_rollouts=128,
    temperature=lam,
    noise=np.diag(sigma**2),
)
mpc = ModelPredictiveController(planner)

# The task the closed loop is scored on: the car's distance from the line, over TF,
# from a start drawn up to 0.2 m and 0.2 rad off the nominal one

dx_0 = np.zeros(9)
dx_0[0:3] = [0.2, 0.2, 0.2]  # [m, m, rad]

car_centre = mount(point_probe(), on=plant, frame="car:body")
problem = PlanningProblem(
    sys=plant,
    x0=Uniform(plant.x0 - dx_0, plant.x0 + dx_0),
    cost=track.distance_field(car_centre).as_cost(weight=10.0),
    horizon=TF,
)

# The closed loop wrapped in its evaluator: compiled once, J of the controller's numbers
# averaged over N_TRIALS starts

evaluator = problem.compile(mpc % 0.2, substeps=10, backend="jax")
start = evaluator.params
print(evaluator.evaluate(start, n_trials=N_TRIALS))

# The tuning program over the temperature λ alone, exported by the evaluator

wrt = ["controller.solve.method.temperature"]
bounds = {wrt[0]: (1e-3, 1.0)}
program = evaluator.program(wrt, bounds=bounds, n_trials=N_TRIALS, params=start)
best = Optimizer(
    program,
    # method="l-bfgs",
    method="ipopt",
    options={"maxiter": 8},
).solve()
print(best)

# The tuned closed loop, scored, drawn and animated round the circuit

tuned = evaluator.unpack(best.z, wrt, params=start)
print(evaluator.evaluate(tuned, n_trials=N_TRIALS))

closed_loop = evaluator.loop
result = evaluator.trajectory(tuned)
closed_loop.plot_trajectory(result)

car.skin = racecar_skin_2d
closed_loop.animate(
    result,
    overlays=mpc_animation_overlays(result, mpc, loop=closed_loop, track=track),
)
