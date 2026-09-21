"""Pure pursuit around a closed circuit, over a rate-limited speed loop."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import DiagramSystem, Source
from minilink.blocks import RateLimiter
from minilink.catalog import UdeSRacecarDyn
from minilink.control import PI, PurePursuit
from minilink.planning import (
    ReferenceTrack,
    TrackCorridorOverlay,
    circuit_waypoints,
    from_waypoints,
)

LENGTH, WIDTH, RADIUS = 14.0, 9.0, 2.5  # [m] the circuit: a rounded rectangle
SPACING = 0.25  # [m] between waypoints
HALF_WIDTH = 0.5  # [m] the lane drawn round the waypoints in the animation
V_REF = 3.0  # [m/s] held all the way round
LOOKAHEAD = 0.5  # [m] at standstill
LOOKAHEAD_GAIN = 0.15  # [s] added lookahead per unit speed
KP, KI = 60.0, 40.0  # [W per m/s, W per m] speed loop on the drive power
P_CRUISE = 8.0  # [W] what holds V_REF on the straight, and where the throttle starts
THROTTLE_RATE = 200.0  # [W/s] how fast the drive may take or give up power
TF = 15.0  # [s], a little more than one lap
N_STEPS = 3001
N_FRAMES = 151

path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS, spacing=SPACING)
start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])

car = UdeSRacecarDyn()
wheelbase = car.params["a"] + car.params["b"]
car.x0 = np.array(
    [
        start[0],
        start[1],
        heading,
        V_REF,
        0.0,
        0.0,
        V_REF / car.params["r_r"],
        0.0,
        P_CRUISE,
    ]
)
car.camera_scale = None  # frame the whole scene, circuit included

# The same waypoint array on the planning side: the tracker takes it raw (RULES 3.2
# keeps control from importing planning), and it draws the lane in the animation.
lane = TrackCorridorOverlay(
    ReferenceTrack(from_waypoints(np.vstack([path, path[:1]])), half_width=HALF_WIDTH),
    n_samples=4 * len(path),
)

# Two loops, both on the plant's own sensor ports: a PI holding the speed through the
# power port, and the geometric tracker holding the line through the steer port. The
# derivative term has nothing to act on here — power to speed is a first-order lag, and
# a D term only feeds the corner-entry deceleration back as lift-off — so the speed loop
# is a PI, with the drive's limits as its anti-windup bounds.
lap = DiagramSystem()
lap.add_subsystem(Source(1), "cruise")
lap.add_subsystem(
    PI(
        Kp=KP,
        Ki=KI,
        ports="reference",
        u_min=-car.params["P_max"],
        u_max=car.params["P_max"],
    ),
    "speed",
)
lap.add_subsystem(
    RateLimiter(
        rate_max=THROTTLE_RATE,
        tau=0.02,
        lower=-car.params["P_max"],
        upper=car.params["P_max"],
        x0=P_CRUISE,
    ),
    "throttle",
)
lap.add_subsystem(
    PurePursuit(
        path,
        wheelbase=wheelbase,
        lookahead=LOOKAHEAD,
        lookahead_gain=LOOKAHEAD_GAIN,
        delta_max=car.params["delta_max"],
        rear_offset=car.params["b"],
    ),
    "tracker",
)
lap.add_subsystem(car, "car")
lap.subsystems["cruise"].params["value"] = np.array([V_REF])

lap.connect("cruise", "y", "speed", "r")
lap.connect("car", "speed", "speed", "y")
lap.connect("speed", "u", "throttle", "u")
lap.connect("throttle", "y", "car", "P_cmd")
lap.connect("car", "y", "tracker", "y")
lap.connect("tracker", "u", "car", "delta_cmd")

lap.plot_diagram()

traj = lap.compute_trajectory(tf=TF, n_steps=N_STEPS, verbose=True)


run = lap.trajectory_of(car, traj)
lap.plot_trajectory(traj, signals=("car:speed", "tracker:u", "car:grip"), show=False)

car.animate(
    run.resample(n_samples=N_FRAMES),
    overlays=[lane],
)
