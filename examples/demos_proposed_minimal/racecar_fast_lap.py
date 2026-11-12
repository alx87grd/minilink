"""Racecar fast lap: an MPC on the dynamic model brakes into the corners and uses the tires' grip."""

import matplotlib.pyplot as plt
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
    circuit_waypoints,
    from_waypoints,
    mount,
    point_probe,
    quadratic_hinge,
)

V_TARGET = 5.5  # [m/s] asked for everywhere: more than the corners allow
TF = 8.0  # [s] a lap and a half

# The circuit: a rounded rectangle 18 m round, its corners of 1 m radius, and a lane
# 0.6 m either side of its line. The plan keeps a margin: it is given a lane of 0.4 m

path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
planned_track = ReferenceTrack(from_waypoints(path), half_width=0.4)

# The model the plan runs on: the dynamic racecar itself, nine states, with its tires,
# its drivetrain and its steering servo, so the plan knows what a corner costs. Its
# wheel rate, steering and power are kept within what the car has

model = UdeSRacecarDyn()
r_r = model.params["r_r"]
delta_max = model.params["delta_max"]
P_max = model.params["P_max"]
model.state.lower[6:9] = [0.0, -delta_max, -P_max]
model.state.upper[6:9] = [3.0 * V_TARGET / r_r, delta_max, P_max]

# The cost: hold V_TARGET with little sideslip and little steering, and stay in the lane

p_cruise = 15.0  # [W] about what holds the speed on a straight

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

# The start: on the near straight at 3 m/s, 0.1 m beside the line

v_0 = 3.0  # [m/s]

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

# The law: a plan 1.2 s ahead on 12 intervals, solved by SQP with exact derivatives,
# again every 0.1 s from the measured state, each solve starting from the last plan

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

# The loop: the law on its clock, closed on the car

car = UdeSRacecarDyn()
car.x0 = x0.copy()
car.skin = racecar_skin_3d

closed_loop = (mpc % 0.1) @ car
closed_loop.plot_diagram()

traj = closed_loop.compute_trajectory(tf=TF, dt=0.002)

# The lap, coloured by the speed: slow into each corner, fast out of it

lap = traj.of("plant")

fig, ax = track.plot(
    bounds=((-4.5, 4.5), (-3.5, 3.5)), waypoint_color="none", show=False
)
speed = ax.scatter(
    lap.x[0, ::20], lap.x[1, ::20], c=lap.x[3, ::20], s=6, cmap="viridis"
)
fig.colorbar(speed, ax=ax, label="speed [m/s]")
plt.show()

car.animate(lap, renderer="meshcat")
