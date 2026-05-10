import math

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Explicit camera basis (columns = camera X, Y, Z in world); overrides camera_plot_axes.
theta = math.radians(28.0)
c, s = math.cos(theta), math.sin(theta)
R = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_R = R
sys.camera_scale = 10.0
sys.animate()
