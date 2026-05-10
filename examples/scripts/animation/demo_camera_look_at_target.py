"""Look-at center: ``attach_standard_camera(..., target=...)`` (``T[:3, 3]``)."""

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import attach_standard_camera

sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0
sys.compute_trajectory(tf=10.0)

attach_standard_camera(sys, target=(2.0, 0.5, 0.0), plot_axes=(0, 1), scale=8.0)
sys.animate()
