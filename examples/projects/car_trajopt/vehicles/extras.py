"""Vehicle variants outside the teaching ladder: a constant-speed car and two holonomic robots."""

from functools import partial

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import translation
from minilink.core.system import DynamicSystem

# The kinematic UdeS racecar is the catalog's own; the project imports it from here
from minilink.dynamics.catalog.vehicles.racecar import UdeSRacecar  # noqa: F401
from minilink.dynamics.catalog.vehicles.steering import KinematicBicycle
from minilink.graphical.animation.primitives import Arrow, Circle, Sphere
from minilink.graphical.catalog.skins import car_skin_2d

###############################################################################
# Public API
###############################################################################


class ConstantSpeedKinematicCar(DynamicSystem):
    """Kinematic car with constant speed and steering-angle input."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Kinematic Car"

        # Parameters: a full-size wheelbase, driven at a fixed forward speed
        self.a = 2.0
        self.b = 3.0
        self.params = {
            "speed": 2.0,
            "a": self.a,
            "b": self.b,
            "length": self.a + self.b,
        }

        # Signal labels and units
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["steering"]
        self.inputs["u"].units = ["rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the kinematic bicycle's 2-D look
        self.wheel_len = 0.76
        self.wheel_width = 0.27
        self.camera_scale = 2.0 * self.params["length"]
        self.camera_follow_frame = "body"
        self.skin = partial(car_skin_2d, color="#1a1a1a")

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        speed = params["speed"]
        length = params["length"]

        theta = x[2]
        steering = u[0]

        # Kinematic bicycle at a fixed speed: ẋ = v cos θ, ẏ = v sin θ, θ̇ = v tan δ / L
        dx = xp.array(
            [
                speed * xp.cos(theta),
                speed * xp.sin(theta),
                speed * xp.tan(steering) / length,
            ]
        )

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def tf(self, x, u, t=0, params=None):
        full_u = self.bicycle_input(u)

        return KinematicBicycle.tf(self, x, full_u, t)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        full_u = self.bicycle_input(u)

        return KinematicBicycle.get_dynamic_geometry(self, x, full_u, t)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def bicycle_input(self, u):
        """The kinematic bicycle's input ``[speed, steering]``, at the fixed speed."""
        speed = self.params["speed"]

        steering = u[0]
        full_u = np.array([speed, steering])

        return full_u


class DynamicHolonomicMobileRobot(DynamicSystem):
    """Holonomic 2D point with acceleration inputs.

    State ``x = [x, y, vx, vy]``; input ``u = [ax, ay]``.
    """

    def __init__(self):
        super().__init__(n=4, input_dim=2, output_dim=4, expose_state=True)
        self.name = "Dynamic Holonomic Mobile Robot"

        # Signal labels and units
        self.state.labels = ["x", "y", "vx", "vy"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["ax", "ay"]
        self.inputs["u"].units = ["m/s^2", "m/s^2"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the EoM)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(x)

        # Double integrator in the plane: the position integrates the velocity, the
        # velocity the acceleration, ṗ = v and v̇ = a
        dx = xp.array([x[2], x[3], u[0], u[1]])

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
                    vector=(x[2], x[3]),
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

        # Signal labels and units
        self.state.labels = ["x", "y", "z"]
        self.state.units = ["m", "m", "m"]
        self.inputs["u"].labels = ["vx", "vy", "vz"]
        self.inputs["u"].units = ["m/s", "m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the EoM)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(u)

        # Holonomic point in 3D: the velocity command integrates straight to the
        # position, ṗ = u
        dx = xp.asarray(u)

        return dx

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
