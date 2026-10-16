"""Longitudinal front-wheel-drive car with wheel slip or wheel torque as the input."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    ground_line,
    vehicle_body,
    wheel_box,
)

###############################################################################
# Public API
###############################################################################


class LongitudinalFrontWheelDriveCarWithWheelSlipInput(DynamicSystem):
    """Longitudinal front-wheel-drive car with wheel slip as input."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.name = "Front Wheel Drive Car With Slip Input"

        # Parameters
        self.params = {
            "length": 2.0,
            "xc": 1.0,
            "yc": 0.5,
            "mass": 1500.0,
            "gravity": 9.81,
            "rho": 1.225,
            "cdA": 0.3 * 2.0,
            "mu_max": 1.0,
            "mu_slope": 70.0,
        }

        # Signal labels and units
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["slip"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a camera framing the car (not part of the EoM)
        self.camera_scale = 2.0 * self.params["length"]
        self.camera_follow_frame = "body"

    def slip2force(self, slip, params=None):
        params = self.params if params is None else params
        xp = array_module(slip)

        mu_max = params["mu_max"]
        mu_slope = params["mu_slope"]

        # Sigmoid traction curve, the friction ratio μ = |F_x / F_z| against the slip s:
        # μ = μ_max (2 / (1 + e^(−k s)) − 1), with k the slope
        mu = mu_max * (2.0 / (1.0 + xp.exp(-mu_slope * slip)) - 1.0)

        return mu

    def acceleration(self, speed, slip, params=None):
        params = self.params if params is None else params
        xp = array_module(speed)

        length = params["length"]
        xc = params["xc"]
        yc = params["yc"]
        mass = params["mass"]
        gravity = params["gravity"]
        rho = params["rho"]
        cdA = params["cdA"]

        rr = xc / length  # Ground share behind the c.g. (the load on the driven wheels)
        ry = yc / length  # The c.g. height ratio driving the load transfer
        mu = self.slip2force(slip, params)

        # Aero drag ½ ρ c_d A v |v|
        drag = 0.5 * rho * cdA * speed * xp.abs(speed)

        # Longitudinal acceleration with dynamic load transfer onto the wheels:
        # a = (μ m g r_x − drag) / (m (1 + μ r_y))
        acceleration = (mu * mass * gravity * rr - drag) / (mass * (1.0 + mu * ry))

        return acceleration

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(x, u)

        speed = x[1]
        slip = u[0]

        # ẋ = v, v̇ = a(v, s)
        acceleration = self.acceleration(speed, slip, params)
        dx = xp.array([speed, acceleration])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        length = self.params["length"]

        # The body on its two wheels, above the ground line
        wheel_rear = wheel_box(length=0.35, width=0.18)
        wheel_front = wheel_box(length=0.35, width=0.18)
        wheel_rear.local_transform = translation(-0.4 * length, -0.45, 0.0)
        wheel_front.local_transform = translation(0.4 * length, -0.45, 0.0)

        return {
            "world": [ground_line(length=12.0, y=-0.45)],
            "body": [
                vehicle_body(length=length, width=0.7, color="blue"),
                wheel_rear,
                wheel_front,
            ],
        }

    def tf(self, x, u, t=0, params=None):
        return {
            "body": translation(x[0], 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        force = self.slip2force(u[0]) if self.m == 1 else 0.0
        length = self.params["length"]

        return {
            "body": [
                Arrow(
                    base=(0.5 * length, 0.0),
                    vector=(force, 0.0),
                    scale=1.0,
                    color="red",
                    linewidth=2,
                )
            ]
        }


class LongitudinalFrontWheelDriveCarWithTorqueInput(
    LongitudinalFrontWheelDriveCarWithWheelSlipInput
):
    """Longitudinal front-wheel-drive car with wheel torque input."""

    def __init__(self):
        DynamicSystem.__init__(
            self,
            n=4,
            input_dim=1,
            output_dim=1,
            expose_state=True,
            y_dependencies=(),
        )
        self.name = "Front Wheel Drive Car With Torque Input"

        # Parameters: the slip-input car's, plus the driven wheel's radius and inertia
        self.params = {
            "length": 2.0,
            "xc": 1.0,
            "yc": 0.5,
            "mass": 1500.0,
            "gravity": 9.81,
            "rho": 1.225,
            "cdA": 0.3 * 2.0,
            "mu_max": 1.0,
            "mu_slope": 70.0,
            "wheel_radius": 0.3,
            "wheel_inertia": 1.5,
        }

        # Signal labels and units: the output is the slip ratio
        self.state.labels = ["x", "dx", "wheel_speed", "wheel_angle"]
        self.state.units = ["m", "m/s", "rad/s", "rad"]
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = ["slip"]

        # Initial condition: the car already rolling
        self.x0 = np.array([0.0, 0.01, 0.0, 0.0])

        # Graphics: a camera that frames the car and follows its body (not in the EoM)
        self.camera_scale = 2.0 * self.params["length"]
        self.camera_follow_frame = "body"

    def slip_ratio(self, speed, wheel_speed, params=None):
        params = self.params if params is None else params
        xp = array_module(speed, wheel_speed)

        wheel_radius = params["wheel_radius"]

        # Tire slip ratio, the contact-point speed against the ground speed, clipped:
        # s = clip((r ω − v) / (|v| + 10⁻⁶), −0.5, 0.5)
        denominator = xp.abs(speed) + 1e-6
        slip = xp.clip((wheel_radius * wheel_speed - speed) / denominator, -0.5, 0.5)

        return slip

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        mass = params["mass"]
        rho = params["rho"]
        cdA = params["cdA"]
        wheel_radius = params["wheel_radius"]
        wheel_inertia = params["wheel_inertia"]

        speed = x[1]
        wheel_speed = x[2]
        torque = u[0]
        slip = self.slip_ratio(speed, wheel_speed, params)

        # The car: v̇ = a(v, s), and the aero drag ½ ρ c_d A v |v|
        acceleration = self.acceleration(speed, slip, params)
        drag = 0.5 * rho * cdA * speed * xp.abs(speed)

        # Wheel spin-up: the drive torque less the traction and drag reaction torque,
        # J_w ω̇ = τ − r (m a + drag)
        wheel_acceleration = (
            torque - wheel_radius * (mass * acceleration + drag)
        ) / wheel_inertia

        # The state x = [x, v, ω, φ]: ẋ = v, v̇ = a, ω̇, φ̇ = ω
        dx = xp.array([speed, acceleration, wheel_acceleration, wheel_speed])

        return dx

    def h(self, x, u, t=0.0, params=None):
        xp = array_module(x)

        speed = x[1]
        wheel_speed = x[2]

        # The measured output is the slip ratio
        slip = self.slip_ratio(speed, wheel_speed, params)
        y = xp.array([slip])

        return y

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        slip = self.slip_ratio(x[1], x[2])
        u_slip = np.array([slip])

        return super().get_dynamic_geometry(x, u_slip, t)


if __name__ == "__main__":
    sys = LongitudinalFrontWheelDriveCarWithWheelSlipInput()
    sys.x0 = np.array([0.0, 20.0])
    sys.inputs["u"].nominal_value = np.array([-0.1])
    sys.compute_trajectory(tf=10.0)
    sys.plot_phase_plane()
    sys.animate()
