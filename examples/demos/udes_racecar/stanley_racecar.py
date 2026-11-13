"""Stanley round the closed circuit of the MPC demo, at a constant power."""

import numpy as np

from minilink import DiagramSystem, Stanley
from minilink.catalog import UdeSRacecarDyn
from minilink.core.geometry import circuit_waypoints, from_waypoints
from minilink.graphical.catalog import (
    racecar_skin_2d,  # noqa: F401  (used by the commented 2-D look below)
    racecar_skin_3d,
)
from minilink.planning import ReferenceTrack, TrackCorridorOverlay

# The circuit: a rounded rectangle, 36.6 m round, and a lane 0.6 m either side of its line

width = 8.0  # [m]

waypoints = circuit_waypoints(length=12.0, width=width, radius=2.0)
path = from_waypoints(waypoints, closed=True)
track = ReferenceTrack(path, half_width=0.6)

# The car on a floor of friction μ, its power held at p_cruise: the open P_cmd port
# reads its nominal value

mu = 0.6
p_cruise = 10.0  # [W] about 3 m/s, just under what a 2 m corner holds at μ = 0.6

car = UdeSRacecarDyn(named_ports=True)
car.params["terrain"]["friction"] = mu
car.inputs["P_cmd"].nominal_value = np.array([p_cruise])
car.camera_scale = 2.0

# It sets off at v_0 halfway along the near straight, 0.15 m beside the line, and the
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

# The law steers the front axle, a ahead of the state's centre of gravity: the heading
# error plus arctan(−k e / (k_s + |vx|)) on the cross-track error e

k = 2.0  # [1/s]
k_s = 0.5  # [m/s] keeps the correction finite at rest

stanley = Stanley(
    path,
    gain=k,
    softening=k_s,
    delta_max=car.params["delta_max"],
    front_offset=car.params["a"],
)

# The loop: the state into the law, its steer into the car (the car has two commands,
# so the wiring is explicit)

closed_loop = DiagramSystem()
closed_loop.add_subsystem(stanley, "controller")
closed_loop.add_subsystem(car, "plant")
closed_loop.connect("plant", "y", "controller", "y")
closed_loop.connect("controller", "u", "plant", "delta_cmd")

closed_loop.plot_diagram()

result = closed_loop.compute_trajectory(tf=12.0, dt=0.002)  # a little more than one lap
# closed_loop.plot_trajectory()


# car.skin = racecar_skin_2d
# closed_loop.animate(
#     result.resample(n_samples=250),
#     overlays=[TrackCorridorOverlay(track)],
# )


car.skin = racecar_skin_3d
closed_loop.animate(
    result.resample(n_samples=250),
    overlays=[TrackCorridorOverlay(track)],
    renderer="meshcat",
    native=False,
)
