"""Pure pursuit, Stanley and line of sight on the same car and circuit: the paths and the cross-track error."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import (
    DiagramSystem,
    HeadingController,
    LineOfSight,
    PurePursuit,
    Stanley,
)
from minilink.catalog import UdeSRacecarDyn
from minilink.core.geometry import circuit_waypoints, from_waypoints

# The circuit: a rounded rectangle, 36.6 m round

width = 8.0  # [m]

waypoints = circuit_waypoints(length=12.0, width=width, radius=2.0)
path = from_waypoints(waypoints, closed=True)

# The car on a floor of friction μ, its power held at p_cruise: the open P_cmd port
# reads its nominal value

mu = 0.6
p_cruise = 10.0  # [W] about 3 m/s, just under what a 2 m corner holds at μ = 0.6

car = UdeSRacecarDyn()
car.params["mu"] = mu
car.inputs["P_cmd"].nominal_value = np.array([p_cruise])
a = car.params["a"]
b = car.params["b"]
delta_max = car.params["delta_max"]

# It sets off at v_0 halfway along the near straight, 0.15 m beside the line, and each
# law pulls it in

v_0 = 3.0  # [m/s]

first = int(np.argmin(np.abs(waypoints[:, 0]) + np.abs(waypoints[:, 1] + 0.5 * width)))
step = waypoints[first + 1] - waypoints[first]
heading = np.arctan2(step[1], step[0])
start = waypoints[first] + 0.15 * np.array([-np.sin(heading), np.cos(heading)])
car.x0 = np.array(
    [
        start[0],
        start[1],
        heading,
        v_0,
        0.0,
        0.0,
        v_0 / car.params["r_r"],
        0.0,
        p_cruise,
    ]
)

# The three laws at their defaults, each written at its own point of the car

pursuit = PurePursuit(path, wheelbase=a + b, delta_max=delta_max, rear_offset=b)
stanley = Stanley(path, delta_max=delta_max, front_offset=a)
guidance = LineOfSight(path, lookahead=1.0)
heading_loop = HeadingController(wheelbase=a + b, delta_max=delta_max)

# Pure pursuit and Stanley steer the car directly

closed_loops = {}
for name, law in (("pure pursuit", pursuit), ("Stanley", stanley)):
    closed_loop = DiagramSystem()
    closed_loop.add_subsystem(law, "controller")
    closed_loop.add_subsystem(car, "plant")
    closed_loop.connect("plant", "y", "controller", "y")
    closed_loop.connect("controller", "u", "plant", "delta_cmd")
    closed_loops[name] = closed_loop

# Line of sight gives a heading, and the heading loop the steer

closed_loop = DiagramSystem()
closed_loop.add_subsystem(guidance, "guidance")
closed_loop.add_subsystem(heading_loop, "controller")
closed_loop.add_subsystem(car, "plant")
closed_loop.connect("plant", "y", "guidance", "y")
closed_loop.connect("guidance", "psi_ref", "controller", "r")
closed_loop.connect("plant", "y", "controller", "y")
closed_loop.connect("controller", "u", "plant", "delta_cmd")
closed_loops["line of sight"] = closed_loop

# Each lap, a little more than one round, and the car's distance from the line along it

fig, (ax_path, ax_error) = plt.subplots(1, 2, figsize=(13, 5))
circuit = np.vstack([waypoints, waypoints[:1]])
ax_path.plot(circuit[:, 0], circuit[:, 1], "k--", lw=0.8, label="path")
for name, closed_loop in closed_loops.items():
    run = closed_loop.compute_trajectory(tf=12.0, dt=0.002).of("plant")
    error = np.array([path.distance(p) for p in run.x[0:2].T])
    rms = np.sqrt(np.mean(error**2))
    print(f"{name:14s} cross-track error: max {error.max():.3f} m, RMS {rms:.3f} m")
    ax_path.plot(run.x[0], run.x[1], label=name)
    ax_error.plot(run.t, error, label=name)

ax_path.set_aspect("equal")
ax_path.set_xlabel("x [m]")
ax_path.set_ylabel("y [m]")
ax_path.legend()
ax_error.set_xlabel("t [s]")
ax_error.set_ylabel("distance from the path [m]")
ax_error.legend()
plt.show()
