"""Drive the 1/10 racecar with a gamepad — windowless joystick + MeshCat + Plotly."""

from minilink import UdeSRacecarDyn
from minilink.graphical.catalog import racecar_skin_3d

sys = UdeSRacecarDyn(wheel_angles=True)
sys.skin = racecar_skin_3d

sys.game(
    renderer="meshcat",
    is_3d=True,
    plot=("slip", "grip"),
    plot_renderer="plotly",
    joystick_axes=[(1, 1.0), (0, -1.0)],
)
