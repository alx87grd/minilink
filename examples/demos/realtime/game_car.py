"""Drive the 1/10 racecar with a gamepad — windowless joystick + MeshCat + Plotly."""

from minilink import UdeSRacecarDyn3D

sys = UdeSRacecarDyn3D()

sys.game(
    renderer="meshcat",
    is_3d=True,
    plot=("slip", "grip"),
    plot_backend="plotly",
    joystick_axes=[(1, 1.0), (0, -1.0)],
)
