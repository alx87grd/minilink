from functools import partial

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    Sphere,
    wheel_box,
)
from minilink.graphical.catalog.skins import car_skin_2d


class KinematicBicycle(DynamicSystem):
    """Kinematic bicycle model with speed and steering-angle inputs.

    See :class:`JaxKinematicBicycle` and :class:`JaxKinematicBicycleRateInputs`
    for JAX-traceable variants.
    """

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

        # kinematic bicycle: heading turns at speed * tan(steering) / wheelbase
        return np.array(
            [
                speed * np.cos(theta),
                speed * np.sin(theta),
                speed * np.tan(steering) / length,
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


class ConstantSpeedKinematicCar(DynamicSystem):
    """Kinematic car with constant speed and steering-angle input."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Kinematic Car"
        self.a = 2.0
        self.b = 3.0
        self.params = {
            "speed": 2.0,
            "a": self.a,
            "b": self.b,
            "length": self.a + self.b,
        }
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["steering"]
        self.inputs["u"].units = ["rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        self.wheel_len = 0.76
        self.wheel_width = 0.27
        self.camera_scale = 2.0 * self.params["length"]
        self.camera_follow_frame = "body"
        self.skin = partial(car_skin_2d, color="#1a1a1a")

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        speed = params["speed"]
        length = params["length"]
        theta = x[2]
        steering = u[0]

        # kinematic bicycle driven at fixed forward speed
        return np.array(
            [
                speed * np.cos(theta),
                speed * np.sin(theta),
                speed * np.tan(steering) / length,
            ]
        )

    def h(self, x, u, t=0.0, params=None):
        return x

    def tf(self, x, u, t=0, params=None):
        full_u = np.array([self.params["speed"], u[0]])
        return KinematicBicycle.tf(self, x, full_u, t)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        full_u = np.array([self.params["speed"], u[0]])
        return KinematicBicycle.get_dynamic_geometry(self, x, full_u, t)


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
        return np.asarray(u)

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


class HolonomicMobileRobot3D(DynamicSystem):
    """Holonomic 3D point robot."""

    def __init__(self):
        super().__init__(n=3, input_dim=3, output_dim=3, expose_state=True)
        self.name = "Holonomic 3D Mobile Robot"
        self.state.labels = ["x", "y", "z"]
        self.state.units = ["m", "m", "m"]
        self.inputs["u"].labels = ["vx", "vy", "vz"]
        self.inputs["u"].units = ["m/s", "m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters (not part of the EoM)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        # holonomic point in 3D: velocity command integrates straight to position
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {"body": [Sphere(radius=0.25, color="blue", opacity=0.9)]}

    def tf(self, x, u, t=0, params=None):
        return {
            "body": translation(x[0], x[1], x[2]),
            "arrows": translation(x[0], x[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "arrows": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(u[0], u[1]),
                    scale=0.4,
                    color="red",
                    linewidth=2,
                )
            ]
        }


class UdeSRacecar(KinematicCar):
    """Small kinematic car with UdeS racecar-scale parameters."""

    def __init__(self):
        super().__init__()
        self.name = "UdeS Racecar"
        self.a = 0.17
        self.b = 0.17
        self.params["a"] = self.a
        self.params["b"] = self.b
        self.params["length"] = self.a + self.b

        # Graphic parameters (not part of the EoM)
        self.width = 0.17
        self.tire_length = 0.04
        self.tire_width = 0.015
        self.camera_scale = 2.0 * self.params["length"]


# Same equations as :class:`KinematicBicycle`, but written so the dynamics ``f``
# trace through ``jax.numpy`` for gradient-based trajectory optimization.


class JaxKinematicBicycle(KinematicBicycle):
    """JAX-traceable :class:`KinematicBicycle`.

    Inherits the geometry and visualization contract from
    :class:`KinematicBicycle` and only overrides the equations of motion so that
    ``f(x, u, t)`` traces through ``jax.numpy``.
    """

    def __init__(self):
        super().__init__()
        self.name = "JAX Kinematic Bicycle"

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        length = params["a"] + params["b"]
        speed = u[0]
        steering = u[1]
        theta = x[2]

        # kinematic bicycle: heading turns at speed * tan(steering) / wheelbase
        return jnp.array(
            [
                speed * jnp.cos(theta),
                speed * jnp.sin(theta),
                speed * jnp.tan(steering) / length,
            ]
        )

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(x)


class JaxKinematicBicycleRateInputs(JaxKinematicBicycle):
    """JAX-traceable :class:`KinematicBicycle` with rate inputs.

    State ``x = [x, y, theta, speed, steering]``; inputs are ``speed_dot`` and
    ``steering_dot``. Useful with trajectory optimization run with
    ``compile_backend="jax"``.
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "JAX Kinematic Bicycle (rate inputs)"

        self.n = 5

        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        self.state.labels = ["x", "y", "theta", "speed", "steering"]
        self.state.units = ["m", "m", "rad", "m/s", "rad"]

        self.inputs = {}
        self.add_input_port(
            "speed_dot",
            nominal_value=0.0,
            labels=["speed_dot"],
            units=["m/s^2"],
        )
        self.add_input_port(
            "steering_dot",
            nominal_value=0.0,
            labels=["steering_dot"],
            units=["rad/s"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        length = params["a"] + params["b"]
        speed = x[3]
        steering = x[4]
        theta = x[2]

        pose_dot = jnp.array(
            [
                speed * jnp.cos(theta),
                speed * jnp.sin(theta),
                speed * jnp.tan(steering) / length,
            ]
        )
        return jnp.concatenate([pose_dot, u])

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        a = params["a"]
        steering = x[4]
        T_wb = SE2(x[0], x[1], x[2])
        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, steering),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


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
