from functools import partial

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    wheel_box,
)
from minilink.graphical.catalog.skins import car_skin_2d


class KinematicBicycle(DynamicSystem):
    """Kinematic bicycle model with speed and steering-angle inputs."""

    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3, expose_state=True)
        self.name = "Kinematic Bicycle"
        # CG-centered wheelbase: ``length = a + b`` (same frame convention as
        # :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.DynamicBicycle`).
        self.params = {"a": 1.0, "b": 1.0, "length": 2.0}
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["speed", "steering"]
        self.inputs["u"].units = ["m/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics-only (2-D centerline look shared with :class:`DynamicBicycle`)
        self.wheel_len = 0.76
        self.wheel_width = 0.27
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"
        self.skin = partial(car_skin_2d, color="#1a1a1a")

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        length = params["a"] + params["b"]
        speed, steering = u
        theta = x[2]
        xp = array_module(x, u)

        # kinematic bicycle: heading turns at speed * tan(steering) / wheelbase
        return xp.array(
            [
                speed * xp.cos(theta),
                speed * xp.sin(theta),
                speed * xp.tan(steering) / length,
            ]
        )

    def h(self, x, u, t=0.0, params=None):
        return x

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        a = params["a"]
        steering = u[1]
        T_wb = SE2(x[0], x[1], x[2])
        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, steering),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


class KinematicCar(KinematicBicycle):
    """Kinematic bicycle parameterized as a full-size car with a four-wheel skin."""

    def __init__(self):
        super().__init__()
        self.name = "Kinematic Car"
        self.a = 2.0
        self.b = 3.0
        self.params["a"] = self.a
        self.params["b"] = self.b
        self.params["length"] = self.a + self.b

        # Graphic parameters (display only; the EoM use only ``length``). The skin
        # is a rectangular body filling the ``length x width`` collision footprint
        # plus four wheels (the front pair steering) -- replacing the bicycle's
        # pointed outline and two in-line wheels so it reads as a car.
        self.width = 2.0
        self.body_width_ratio = 0.74  # tub narrower than track, so wheels show
        self.visual_wheelbase_ratio = 0.64  # axle separation as a fraction of length
        self.tire_length = 0.95
        self.tire_width = 0.34
        self.camera_scale = 2.0 * self.params["length"]

    def get_kinematic_geometry(self):
        length = self.params["length"]
        body = Box(
            length_x=length,
            length_y=self.body_width_ratio * self.width,
            length_z=0.4,
            color="#4c72b0",
            opacity=0.9,
        )
        axle = 0.5 * self.visual_wheelbase_ratio * length
        half_track = 0.5 * self.width - 0.5 * self.tire_width
        wheel_rl = wheel_box(self.tire_length, self.tire_width)
        wheel_rr = wheel_box(self.tire_length, self.tire_width)
        wheel_rl.local_transform = SE2(-axle, half_track, 0.0)
        wheel_rr.local_transform = SE2(-axle, -half_track, 0.0)
        return {
            "body": [body, wheel_rl, wheel_rr],
            "wheel_fl": [wheel_box(self.tire_length, self.tire_width)],
            "wheel_fr": [wheel_box(self.tire_length, self.tire_width)],
        }

    def tf(self, x, u, t=0, params=None):
        length = self.params["length"]
        steering = u[1]
        axle = 0.5 * self.visual_wheelbase_ratio * length
        half_track = 0.5 * self.width - 0.5 * self.tire_width
        T_body = SE2(x[0], x[1], x[2])
        R_steer = SE2(0.0, 0.0, steering)
        return {
            "body": T_body,
            "wheel_fl": T_body @ SE2(axle, half_track, 0.0) @ R_steer,
            "wheel_fr": T_body @ SE2(axle, -half_track, 0.0) @ R_steer,
        }


class HolonomicMobileRobot(DynamicSystem):
    """Holonomic 2D point robot."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Holonomic Mobile Robot"
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters (not part of the EoM)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        # holonomic point: velocity command integrates straight to position
        return array_module(u).asarray(u)

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
    sys.compute_forced(
        lambda t: np.array([1.0, 0.25 * np.sin(t)]),
        tf=5.0,
        n_steps=160,
        show=True,
        verbose=False,
    )
    sys.animate()
