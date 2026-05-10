import math

import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum




def R_about_x(theta) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def R_about_y(theta) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def R_about_z(theta) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Example tilts (radians). Edit angles or swap the composition order if you want
# intrinsic vs fixed-axis conventions.
yaw_z = math.radians(90.0)
pitch_y = math.radians(-35.0)
roll_x = math.radians(15.0)

R = R_about_z(yaw_z) @ R_about_y(pitch_y) @ R_about_x(roll_x)


sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

# Explicit camera basis (columns = camera X, Y, Z in world); overrides camera_plot_axes.
sys.camera_R = R
sys.camera_scale = 10.0
sys.animate()
sys.animate(is_3d=True)
sys.animate(renderer="meshcat")