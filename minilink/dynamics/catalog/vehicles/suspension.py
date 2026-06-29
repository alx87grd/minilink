import numpy as np

from minilink.core.kinematics import identity, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    CustomLine,
    follow_xy_camera,
    line_between_transform,
    spring_line,
)


class QuarterCarOnRoughTerrain(DynamicSystem):
    """Quarter-car suspension moving over prescribed rough terrain."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Quarter Car On Rough Terrain"
        self.params = {
            "mass": 1.0,
            "b": 1.0,
            "k": 1.0,
            "vx": 1.0,
            # Road profile: superposed sinusoids (amplitude, spatial freq, phase),
            # used by the dynamics forcing and to draw the terrain line.
            "a": np.array([0.5, 0.3, 0.7, 0.2, 0.2, 0.1]),
            "w": np.array([0.2, 0.4, 0.5, 1.0, 2.0, 3.0]),
            "phi": np.array([3.0, 2.0, 0.0, 0.0, 0.0, 0.0]),
        }
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def z(self, x, params=None):
        params = self.params if params is None else params
        a = params["a"]
        w = params["w"]
        phi = params["phi"]

        # ground height: superposed sinusoidal road profile
        return np.sum(a * np.sin(w * (x - phi)))

    def dz(self, x, params=None):
        params = self.params if params is None else params
        a = params["a"]
        w = params["w"]
        phi = params["phi"]

        # ground slope: spatial derivative of the road profile
        return np.sum(a * w * np.cos(w * (x - phi)))

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        b = params["b"]
        k = params["k"]
        vx = params["vx"]

        dy, y, x_pos = x
        ground = self.z(x_pos, params)
        ground_slope = self.dz(x_pos, params)

        # mass y'' = u - k (y - z) - b (y' - z'): sprung mass over the road
        acceleration = (u[0] - k * (y - ground) - b * (dy - ground_slope)) / mass

        return np.array([acceleration, dy, vx])

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[2], x[1], self.camera_scale)

    # The spring stays the unit ``spring_line`` posed by a per-frame stretch
    # transform (``line_between_transform``) carried in ``tf`` — the spring is
    # meant to stretch, and there is no honest fixed-length primitive for it.

    def get_kinematic_geometry(self):
        xs = np.linspace(-5.0, 15.0, 240)
        terrain = np.column_stack([xs, [self.z(x) for x in xs], np.zeros_like(xs)])
        return {
            "world": [CustomLine(terrain, color="black", linewidth=1)],
            "spring": [spring_line(color="black")],
            "body": [
                Box(
                    length_x=0.8, length_y=0.35, length_z=0.2, color="blue", opacity=0.9
                )
            ],
        }

    def tf(self, x, u, t=0, params=None):
        ground = self.z(x[2])
        mass_y = x[1]
        return {
            "spring": line_between_transform([x[2], ground], [x[2], mass_y - 0.2]),
            "body": translation(x[2], mass_y, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "body": [
                Arrow(
                    base=(0.5, 0.0),
                    vector=(0.0, u[0]),
                    scale=0.2,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    sys = QuarterCarOnRoughTerrain()
    sys.x0 = np.array([0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([5.0])
    sys.compute_trajectory(tf=8.0)
    sys.animate()
