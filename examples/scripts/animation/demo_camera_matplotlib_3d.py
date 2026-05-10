"""Initial matplotlib 3D view: ``is_3d=True`` plus ``attach_standard_camera`` (opening frame only)."""

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import attach_standard_camera

sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0
sys.compute_trajectory(tf=10.0)

attach_standard_camera(sys, plot_axes=(0, 2), scale=12.0)
sys.animate(is_3d=True)
