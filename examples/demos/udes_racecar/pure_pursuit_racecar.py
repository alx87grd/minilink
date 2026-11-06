"""Pure pursuit round the closed circuit of the MPC demo, at a constant power."""

import numpy as np

from minilink import DiagramSystem, PurePursuit
from minilink.catalog import UdeSRacecarDyn
from minilink.graphical.catalog.racecar_skin import (
    racecar_skin_2d,  # noqa: F401  (used by the commented 2-D look below)
    racecar_skin_3d,
)
from minilink.planning import (
    ReferenceTrack,
    TrackCorridorOverlay,
    circuit_waypoints,
    from_waypoints,
)

LENGTH, WIDTH, RADIUS = 12.0, 8.0, 2.0  # [m] the circuit: a rounded rectangle, 36.6 m
CORRIDOR_HALF_WIDTH = 0.6  # [m] the lane drawn round the line
P_CRUISE = 10.0  # [W] about 3 m/s, just under what a 2 m corner holds at μ = 0.6
V_START = 3.0  # [m/s]
TF_SIM = 12.0  # [s] a little more than one lap
SIM_DT = 0.002  # [s] plant step
N_FRAMES = 250  # animation frames

LOOKAHEAD = 0.6  # [m] at standstill
LOOKAHEAD_GAIN = 0.25  # [s] added lookahead per m/s of speed
PLANT_MU = 0.6

LATERAL_START = 0.15  # [m] the car sets off beside the line, and the law pulls it in

path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS)
track = ReferenceTrack(from_waypoints(path), half_width=CORRIDOR_HALF_WIDTH)

# The car, its power held at P_CRUISE: the open P_cmd port reads its nominal value
car = UdeSRacecarDyn()
car.params["mu"] = PLANT_MU
car.inputs["P_cmd"].nominal_value = np.array([P_CRUISE])
car.camera_scale = 2.0

# Set off halfway along the near straight, beside the line
first = int(np.argmin(np.abs(path[:, 0]) + np.abs(path[:, 1] + 0.5 * WIDTH)))
step = path[(first + 1) % path.shape[0]] - path[first]
heading = np.arctan2(step[1], step[0])
start = path[first] + LATERAL_START * np.array([-np.sin(heading), np.cos(heading)])
car.x0 = np.array(
    [
        start[0],
        start[1],
        heading,
        V_START,
        0.0,
        0.0,
        V_START / car.params["r_r"],
        0.0,
        P_CRUISE,
    ]
)

# The law steers the rear axle, which sits b behind the state's centre of gravity
pursuit = PurePursuit(
    path,
    wheelbase=car.params["a"] + car.params["b"],
    lookahead=LOOKAHEAD,
    lookahead_gain=LOOKAHEAD_GAIN,
    delta_max=car.params["delta_max"],
    rear_offset=car.params["b"],
)

# The loop: the state into the law, its steer into the car (the car has two commands,
# so the wiring is explicit)
lap = DiagramSystem()
lap.add_subsystem(pursuit, "controller")
lap.add_subsystem(car, "plant")
lap.connect("plant", "y", "controller", "y")
lap.connect("controller", "u", "plant", "delta_cmd")
# lap.plot_diagram()
result = lap.compute_trajectory(tf=TF_SIM, dt=SIM_DT)
# lap.plot_trajectory()


# car.skin = racecar_skin_2d
# lap.animate(
#     result.resample(n_samples=N_FRAMES),
#     overlays=[TrackCorridorOverlay(track)],
# )


car.skin = racecar_skin_3d
lap.animate(
    result.resample(n_samples=N_FRAMES),
    overlays=[TrackCorridorOverlay(track)],
    renderer="meshcat",
    native=False,
)
