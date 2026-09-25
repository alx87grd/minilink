"""A lap of the circuit: a GIF, and an interactive 3-D page."""

import pathlib

import numpy as np

from minilink import DiagramSystem, Source
from minilink.blocks import RateLimiter
from minilink.catalog import UdeSRacecarDyn3D
from minilink.control import PID, PurePursuit
from minilink.graphical.catalog.racecar_skin import racecar_skin_2d, racecar_skin_3d
from minilink.planning import circuit_waypoints

LENGTH, WIDTH, RADIUS = 14.0, 9.0, 2.5  # [m] circuit: a rounded rectangle
V_REF = 3.0  # [m/s]
LOOKAHEAD = 0.6  # [m] at standstill
LOOKAHEAD_GAIN = 0.25  # [s] added lookahead per unit speed
KP, KI, TAU_FILTER = 25.0, 15.0, 0.05  # speed loop on the drive power
THROTTLE_RATE = 200.0  # [W/s] how fast the drive may take power
TF = 14.0  # [s] one lap: the circuit is 41.7 m long
N_STEPS = 2801
N_FRAMES = 101  # animation frames; the GIF is written at 250 dpi, so keep it short
VIDEO_SPEED = 2.0  # play the lap twice as fast as it happens
FLOOR = (-8.0, 8.0, -5.5, 5.5)  # [m] tiled ground around the circuit
TILE = 0.5  # [m]
HERE = pathlib.Path(__file__).parent
GIF = str(HERE / "racecar_lap_3d")  # ".gif" is appended by the writer
PAGE = HERE / "racecar_lap_3d_meshcat.html"

path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS)

start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])

car = UdeSRacecarDyn3D()
car.x0[0:3] = [start[0], start[1], heading]
car.x0[3] = V_REF
car.x0[6] = V_REF / car.params["r_r"]

# the drawing-only scene: tiled ground, the line the tracker is given, no walls
car.scene_bounds = FLOOR
car.scene_tile = TILE
car.scene_path = path
car.camera_follow_frame = "body"
car.camera_scale = 1.2  # [m] half-width of the chase view

lap = DiagramSystem()
lap.add_subsystem(Source(1), "cruise")
lap.add_subsystem(PID(Kp=KP, Ki=KI, tau=TAU_FILTER, ports="reference"), "speed")
lap.add_subsystem(RateLimiter(rate_max=THROTTLE_RATE, tau=0.02), "throttle")
lap.add_subsystem(
    PurePursuit(
        path,
        wheelbase=car.params["a"] + car.params["b"],
        lookahead=LOOKAHEAD,
        lookahead_gain=LOOKAHEAD_GAIN,
        delta_max=car.params["delta_max"],
        rear_offset=car.params["b"],  # the state is at the centre of gravity
        state_dim=car.n,
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

traj = lap.compute_trajectory(tf=TF, n_steps=N_STEPS, verbose=True)
run = lap.trajectory_of(car, traj).resample(n_samples=N_FRAMES)

# the last two states are the wheel rolling angles: a frame is a pure function of the
# state, so a wheel that turns in the animation needs its angle carried by the plant
car.plot_trajectory(run, show=False)

# GIF: the stock matplotlib renderer with the top view.
# A saved animation is drawn flat, so the 3-D look would come out as a wireframe.
car.skin = racecar_skin_2d
car.animate(
    run,
    renderer="matplotlib",
    html=False,
    show=False,
    save=True,
    file_name=GIF,
    time_factor_video=VIDEO_SPEED,
)

# the same lap in 3-D: lit solids, orbit and zoom in a browser
car.skin = racecar_skin_3d
car.animate(
    run,
    renderer="meshcat",
    is_3d=True,
    html=False,
    show=False,
    save=True,
    file_name=str(PAGE),
)
