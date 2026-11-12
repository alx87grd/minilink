"""Kinematic steering models: the bicycle and the holonomic point robot."""

from functools import partial

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
)
from minilink.graphical.catalog.skins import car_skin_2d

###############################################################################
# Public API
###############################################################################


class KinematicBicycle(DynamicSystem):
    """Kinematic bicycle model with speed and steering-angle inputs."""

    def __init__(self):
        super().__init__(state_dim=3, input_dim=2, output_dim=3, expose_state=True)

        # Parameters: the CG-to-axle distances a and b, the wheelbase L = a + b (the
        # frame convention of
        # :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.DynamicBicycle`)
        self.params = {"a": 1.0, "b": 1.0}

        # Signal labels and units
        self.state.labels = ["x", "y", "psi"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["speed", "steering"]
        self.inputs["u"].units = ["m/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the 2-D centerline look shared with :class:`DynamicBicycle`; the
        # track and the wheel radius place the front wheels for a four-wheel look
        # (``car.skin = car_skin_3d``)
        self.wheel_len = 0.76
        self.wheel_width = 0.27
        self.track = 1.6
        self.wheel_radius = 0.3
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"
        self.skin = partial(car_skin_2d, color="#1a1a1a")

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        a = params["a"]
        b = params["b"]

        length = a + b
        speed, steering = u
        psi = x[2]

        # Kinematic bicycle: ẋ = v cos ψ, ẏ = v sin ψ, ψ̇ = v tan δ / L
        dx = xp.array(
            [
                speed * xp.cos(psi),
                speed * xp.sin(psi),
                speed * xp.tan(steering) / length,
            ]
        )

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        a = params["a"]
        # Drawing sizes, with the car looks' defaults for a plant that borrows these
        # frames and sets none
        track = getattr(self, "track", 1.6)
        r = getattr(self, "wheel_radius", 0.3)

        steering = u[1]

        # The car's frames: the body, the front axle steered at its distance a
        # ahead, and the two front wheels at their hubs, steered
        T_wb = SE2(x[0], x[1], x[2])
        T_front = T_wb @ SE2(a, 0.0, steering)
        R_steer = SE2(0.0, 0.0, steering)
        T_fl = T_wb @ translation(a, 0.5 * track, r) @ R_steer
        T_fr = T_wb @ translation(a, -0.5 * track, r) @ R_steer

        return {"body": T_wb, "axle_front": T_front, "wheel_fl": T_fl, "wheel_fr": T_fr}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


class HolonomicMobileRobot(DynamicSystem):
    """Holonomic 2D point robot."""

    def __init__(self):
        super().__init__(state_dim=2, input_dim=2, output_dim=2, expose_state=True)

        # Signal labels and units
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the EoM)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(u)

        # Holonomic point: the velocity command integrates straight to the position
        # ṗ = u
        dx = xp.asarray(u)

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {
            "body": [
                Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True)
            ]
        }

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "body": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(u[0], u[1]),
                    scale=0.4,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    sys = KinematicBicycle()
    sys.x0 = np.array([0.0, 0.0, 0.0])
    sys.compute_trajectory(
        u=lambda t: np.array([1.0, 0.25 * np.sin(t)]),
        tf=5.0,
        n_samples=160,
        show=True,
        verbose=False,
    )
    sys.animate()
