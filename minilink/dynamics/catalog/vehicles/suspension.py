"""Quarter-car suspension riding over a prescribed rough terrain."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    CustomLine,
)
from minilink.graphical.catalog.shapes import spring_between

###############################################################################
# Public API
###############################################################################


class QuarterCar(DynamicSystem):
    """Quarter-car suspension moving over prescribed rough terrain."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)

        # Parameters
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

        # Signal labels and units
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def z(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        a = params["a"]
        w = params["w"]
        phi = params["phi"]

        # Ground height, the superposed sinusoidal road profile:
        # z = Σᵢ aᵢ sin(wᵢ (x − φᵢ))
        z = xp.sum(a * xp.sin(w * (x - phi)))

        return z

    def dz(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        a = params["a"]
        w = params["w"]
        phi = params["phi"]

        # Ground slope, the road profile's spatial derivative:
        # dz/dx = Σᵢ aᵢ wᵢ cos(wᵢ (x − φᵢ))
        dz = xp.sum(a * w * xp.cos(w * (x - phi)))

        return dz

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        mass = params["mass"]
        b = params["b"]
        k = params["k"]
        vx = params["vx"]

        dy, y, x_pos = x
        ground = self.z(x_pos, params)
        ground_slope = self.dz(x_pos, params)
        force = u[0]

        # The road rises under the wheel at ż = (dz/dx) ẋ, with ẋ = vx
        ground_velocity = ground_slope * vx

        # The sprung mass over the road: m ÿ = u − k (y − z) − b (ẏ − ż)
        acceleration = (force - k * (y - ground) - b * (dy - ground_velocity)) / mass

        # The state [ẏ, y, x] moves at [ÿ, ẏ, vx]
        dx = xp.array([acceleration, dy, vx])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        # The terrain line over the drawn stretch of road
        xs = np.linspace(-5.0, 15.0, 240)
        terrain = np.column_stack([xs, [self.z(x) for x in xs], np.zeros_like(xs)])

        return {
            "world": [CustomLine(terrain, color="black", linewidth=1)],
            "body": [
                Box(
                    length_x=0.8, length_y=0.35, length_z=0.2, color="blue", opacity=0.9
                )
            ],
        }

    def tf(self, x, u, t=0, params=None):
        mass_y = x[1]

        return {"body": translation(x[2], mass_y, 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        ground = self.z(x[2], params)
        mass_y = x[1]

        return {
            "world": [
                spring_between([x[2], ground], [x[2], mass_y - 0.2]),
            ],
            "body": [
                Arrow(
                    base=(0.5, 0.0),
                    vector=(0.0, u[0]),
                    scale=0.2,
                    color="red",
                    linewidth=2,
                )
            ],
        }


if __name__ == "__main__":
    sys = QuarterCar()
    sys.x0 = np.array([0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=8.0)
    sys.animate()
