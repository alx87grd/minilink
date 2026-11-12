"""Drive the 3-D car over rolling hills: the keyboard or a gamepad on its power and its steering, in real time."""

from minilink import OffRoadCar3D
from minilink.dynamics.catalog.vehicles.terrain import Bumps
from minilink.simulation.realtime import PygameInput

# The car: an off-road side-by-side on four sprung corners, tall and softly sprung

car = OffRoadCar3D(terrain=Bumps(), wheel_angles=True)

# The ground: rolling hills 1.6 m from crest to trough and 30 m long, drawn over 120 m
# around the start

car.params["terrain"]["amplitude"] = 0.8  # [m]
car.params["terrain"]["wavelength"] = 30.0  # [m]
car.scene_bounds = (-60.0, 60.0, -60.0, 60.0)

# The game: UP and DOWN command the power, LEFT and RIGHT the steering (a gamepad's
# left stick does the same); the wheel loads and the rollover margin are plotted live

car.game(
    renderer="meshcat",
    is_3d=True,
    plot=("loads", "rollover"),
    plot_renderer="plotly",
    input=PygameInput(
        key_axes=[("up", "down"), ("left", "right")],
        joystick_axes=[(1, 1.0), (0, -1.0)],
    ),
)
