"""Drive the off-road side-by-side over rolling hills — keyboard or gamepad, MeshCat."""

from minilink import OffRoadCar3D
from minilink.dynamics.catalog.vehicles.terrain import Bumps
from minilink.simulation.realtime import PygameInput

car = OffRoadCar3D(terrain=Bumps(), wheel_angles=True)
car.params["terrain"]["amplitude"] = 0.8  # [m] hills, crest to trough 1.6 m
car.params["terrain"]["wavelength"] = 30.0  # [m]
car.params["terrain"]["friction"] = 1.0  # [-] road grip: 1 dry, 0.7 wet, 0.1 ice

# The hills drawn around the start: 120 m by 120 m
car.scene_bounds = (-60.0, 60.0, -60.0, 60.0)

car.game(
    renderer="meshcat",
    is_3d=True,
    plot=("loads", "rollover"),
    plot_renderer="plotly",
    # UP/DOWN the power; LEFT steers left (a positive steering angle), RIGHT right
    input=PygameInput(
        key_axes=[("up", "down"), ("left", "right")],
        joystick_axes=[(1, 1.0), (0, -1.0)],
    ),
    backend="jax",
)
