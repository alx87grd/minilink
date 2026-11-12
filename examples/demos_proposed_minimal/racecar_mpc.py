"""Racecar MPC: a lap of a closed circuit, past two cones, on a floor slipperier than the plan believes."""

import numpy as np

from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
from minilink.catalog import UdeSRacecarDyn
from minilink.control import ModelPredictiveController
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    car_outline,
    circuit_waypoints,
    from_waypoints,
    mount,
    point_probe,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.trajectory_optimization.mpc import mpc_animation_overlays

V_TARGET = 6.0  # [m/s] asked for everywhere; the corners are what take it back
TF = 5.0  # [s] the far straight and its cones; a full lap takes about 8.5 s
MU = 0.6  # [-] the floor's grip; the plan believes in 0.7

# The circuit: a rounded rectangle 36.6 m round, a lane 0.6 m either side of its line

width = 8.0  # [m]

path = circuit_waypoints(length=12.0, width=width, radius=2.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)

# Two cones on the far straight, each leaning 0.13 m over the line, one each way: the
# car has to pick a side, then cross between them. Each is drawn 0.10 m wide with a
# keep-out of 0.15 m around it

cones = [(2.0, 0.5 * width - 0.13), (-2.0, 0.5 * width + 0.13)]

scene = Scene(obstacles=[Sphere(cone, 0.25) for cone in cones])

# The model the plan runs on: the 1/10 racecar with a little grip to spare, its wheel
# rate, steering and power kept within what the car has

p_cruise = 15.0  # [W] about what holds V_TARGET on a straight

model = UdeSRacecarDyn()
model.params["terrain"]["friction"] = 0.7
r_r = model.params["r_r"]
delta_max = model.params["delta_max"]
P_max = model.params["P_max"]
model.state.lower[6:9] = [0.0, -delta_max, -P_max]
model.state.upper[6:9] = [3.0 * V_TARGET / r_r, delta_max, P_max]

# The cost: hold the speed, with little steering; keep the car's centre on the line,
# its footprint inside the lane and 0.30 m clear of the cones. The quadratic part says
# nothing of where the car is on the map: the three fields do

centre = mount(point_probe(), on=model)
body = mount(car_outline(length=0.34, width=0.20, margin=0.02), on=model)

x_cruise = np.zeros(model.n)
x_cruise[3] = V_TARGET
x_cruise[6] = V_TARGET / r_r
x_cruise[8] = p_cruise

cost = (
    QuadraticCost.from_system(
        model,
        Q=np.diag([0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 2.0, 0.0]),
        R=np.diag([0.0005, 60.0]),
        S=np.diag([0.0, 0.0, 0.0, 8.0, 2.0, 0.0, 0.0, 2.0, 0.0]),
        x_bar=x_cruise,
        u_bar=np.array([p_cruise, 0.0]),
    )
    + track.distance_field(centre).as_cost(
        weight=60.0, shaping=quadratic_excess(threshold=0.05)
    )
    + track.corridor_field(body).as_cost(
        weight=200.0, shaping=quadratic_hinge(threshold=0.0)
    )
    + scene.clearance_field(body).as_cost(
        weight=400.0, shaping=quadratic_hinge(threshold=0.30)
    )
)

# The start: halfway along the near straight at V_TARGET, 0.15 m beside the line and a
# hair off its axis (the distance to a line has no derivative on the line itself)

first = int(np.argmin(np.abs(path[:, 0]) + np.abs(path[:, 1] + 0.5 * width)))
step = path[(first + 1) % path.shape[0]] - path[first]
heading = np.arctan2(step[1], step[0]) + 1.0e-4
start = path[first] + 0.15 * np.array([-np.sin(heading), np.cos(heading)])

x0 = x_cruise.copy()
x0[0:3] = [start[0], start[1], heading]

# The law: a plan 1.2 s ahead on 12 intervals, solved again every 0.1 s from the
# measured state, each solve starting from the last plan

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        model, x0=x0, cost=cost, X=model.state.box, U=model.input_box, horizon=1.2
    ),
    n_steps=12,
    transcription="direct_collocation",
    backend="jax",
    method="slsqp",
    options={"maxiter": 60, "ftol": 0.3},
)
mpc = ModelPredictiveController(planner, delay=False)

# The loop: the law on its clock, closed on the car on the real floor

car = UdeSRacecarDyn()
car.params["terrain"]["friction"] = MU
car.x0 = x0.copy()
car.skin = racecar_skin_3d
car.camera_scale = 2.0

closed_loop = (mpc % 0.1) @ car
traj = closed_loop.compute_trajectory(tf=TF, dt=0.002)

closed_loop.plot_trajectory()
closed_loop.animate(
    traj.resample(n_samples=250),
    overlays=mpc_animation_overlays(
        traj, mpc, loop=closed_loop, scene=scene, track=track
    ),
    renderer="meshcat",
    native=False,
)
