"""Racecar fast lap: an MPC on the dynamic model brakes into the corners and uses the tires' grip."""

import numpy as np

from minilink import (
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
    UdeSRacecarDyn,
)
from minilink.control import ModelPredictiveController
from minilink.graphical.catalog import racecar_skin_3d
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

########################################################
### Tunables
########################################################

V_TARGET = 5.5  # [m/s]
TF = 8.0  # [s]

########################################################
### Track, scene, and planning corridor
########################################################

path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
planned_track = ReferenceTrack(from_waypoints(path), half_width=0.4)
scene = Scene(obstacles=[Sphere((0.0, 2.0), 0.15)])

########################################################
### Dynamic model for planning
########################################################

model = UdeSRacecarDyn()
r_r = model.params["r_r"]
delta_max = model.params["delta_max"]
P_max = model.params["P_max"]
model.state.lower[6:9] = [0.0, -delta_max, -P_max]
model.state.upper[6:9] = [3.0 * V_TARGET / r_r, delta_max, P_max]

########################################################
### Cost and initial state
########################################################

p_cruise = 15.0  # [W]

centre = mount(point_probe(), on=model)

x_target = np.zeros(model.n)
x_target[3] = V_TARGET
x_target[6] = V_TARGET / r_r
x_target[8] = p_cruise

cost = QuadraticCost.from_system(
    model,
    Q=np.diag([0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 2.0, 0.0]),
    R=np.diag([0.0005, 60.0]),
    S=np.diag([0.0, 0.0, 0.0, 8.0, 2.0, 0.0, 0.0, 2.0, 0.0]),
    x_bar=x_target,
    u_bar=np.array([p_cruise, 0.0]),
) + planned_track.corridor_field(centre).as_cost(
    weight=400.0, shaping=quadratic_hinge()
)

v_0 = 0.0  # [m/s]

heading = np.arctan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
x0 = np.zeros(model.n)
x0[0:3] = [
    path[0, 0] - 0.1 * np.sin(heading),
    path[0, 1] + 0.1 * np.cos(heading),
    heading,
]
x0[3] = v_0
x0[6] = v_0 / r_r
x0[8] = p_cruise

########################################################
### MPC on the dynamic model
########################################################

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        model, x0=x0, cost=cost, X=model.state.box, U=model.input_box, horizon=1.2
    ),
    n_steps=12,
    backend="jax",
    method="slsqp",
    options={"maxiter": 60, "ftol": 0.3},
)
mpc = ModelPredictiveController(planner, delay=False)

car = UdeSRacecarDyn()
car.x0 = x0.copy()
car.skin = racecar_skin_3d

closed_loop = (mpc % 0.1) @ car
closed_loop.plot_diagram()

traj = closed_loop.compute_trajectory(tf=TF, dt=0.002)

########################################################
### Lap replay
########################################################

lap = traj.of("plant")
car.plot_trajectory(lap)

overlays = mpc_animation_overlays(traj, mpc, loop=closed_loop, scene=scene, track=track)
car.animate(lap, overlays=overlays, renderer="meshcat")
