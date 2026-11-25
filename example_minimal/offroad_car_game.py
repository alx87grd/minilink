"""Drive the 3-D car over rolling hills: the keyboard or a gamepad on its power and its steering, in real time."""

from minilink import OffRoadCar3D
from minilink.dynamics.catalog.vehicles.terrain import Bumps
from minilink.simulation.realtime import PygameInput

########################################################
### Tunables
########################################################

AMPLITUDE = 0.8  # [m]
WAVELENGTH = 30.0  # [m]
SCENE_HALF = 60.0  # [m]

########################################################
### The car
########################################################

car = OffRoadCar3D(terrain=Bumps(), wheel_angles=True)

########################################################
### Terrain
########################################################

car.params["terrain"]["amplitude"] = AMPLITUDE
car.params["terrain"]["wavelength"] = WAVELENGTH
car.scene_bounds = (-SCENE_HALF, SCENE_HALF, -SCENE_HALF, SCENE_HALF)

########################################################
### Real-time game
########################################################

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
