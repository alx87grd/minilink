"""Racecar MPC: a plan made on the kinematic model drives the dynamic car through an inner speed loop."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import (
    PI,
    DiagramSystem,
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
    UdeSRacecar,
    UdeSRacecarDyn,
)
from minilink.blocks import Demux, Gain
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

V_CRUISE = 3.0  # [m/s] the speed the plan asks for
TF = 8.0  # [s] a little more than one lap

# The circuit: a rounded rectangle 18 m round, a lane 0.6 m either side of its line,
# and one cone on the far straight

path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
scene = Scene(obstacles=[Sphere((0.0, 2.0), 0.15)])

# The model the plan runs on: the kinematic bicycle, ẋ = v cos ψ, ẏ = v sin ψ,
# ψ̇ = v tan δ / L. Its two inputs are a speed and a steering angle

model = UdeSRacecar()
model.inputs["u"].lower = np.array([0.0, -0.52])
model.inputs["u"].upper = np.array([5.0, 0.52])

# The cost: cruise at V_CRUISE with little steering, stay in the lane, keep 0.2 m
# clear of the cone

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

# The start: on the near straight, 0.1 m beside the line

heading = np.arctan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
x0 = np.array(
    [path[0, 0] - 0.1 * np.sin(heading), path[0, 1] + 0.1 * np.cos(heading), heading]
)

# The outer loop: a plan 1 s ahead, solved again every 0.1 s from the measured pose

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(model, x0=x0, cost=cost, U=model.input_box, horizon=1.0),
    n_steps=8,
    backend="jax",
    method="slsqp",
    options={"maxiter": 40, "ftol": 0.01},
)
mpc = ModelPredictiveController(planner, delay=False)

# The car the plan drives: the dynamic racecar, with its tires, its drivetrain and its
# steering servo. Its inputs are a drive power and a steering command, not a speed

car = UdeSRacecarDyn(named_ports=True)
car.x0[0:3] = x0
car.skin = racecar_skin_3d

# The inner loop: a PI on the speed turns the plan's speed into a drive power; the
# plan's steering angle goes to the servo; the car's pose goes back to the plan

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
inner_loop.plot_diagram()

# The whole loop: the plan on its 10 Hz clock, closed on the inner loop

closed_loop = (mpc % 0.1) @ inner_loop
traj = closed_loop.compute_trajectory(tf=TF, dt=0.01)

# The lap: the car's path in its lane and round the cone, then the car itself

lap = traj.of("plant")
bounds = ((-4.5, 4.5), (-3.5, 3.5))

fig, ax = track.plot(bounds=bounds, waypoint_color="none", show=False)
scene.plot(bounds=bounds, ax=ax, show=False)
ax.plot(lap.x[0], lap.x[1], label="the car")
ax.legend()
plt.show()

inner_loop.animate(lap, renderer="meshcat")
