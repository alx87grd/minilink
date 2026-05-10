import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import attach_standard_camera

# Explicit camera basis R (columns = camera X, Y, Z in world coordinates)
theta = np.deg2rad(28.0)
c, s = np.cos(theta), np.sin(theta)
R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

attach_standard_camera(sys, R=R, scale=10.0)
sys.animate()
