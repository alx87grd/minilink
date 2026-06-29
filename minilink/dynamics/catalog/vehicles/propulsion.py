import numpy as np

from minilink.core.kinematics import identity, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    follow_xy_camera,
    ground_line,
    vehicle_body,
    wheel_box,
)


class LongitudinalFrontWheelDriveCarWithWheelSlipInput(DynamicSystem):
    """Longitudinal front-wheel-drive car with wheel slip as input."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.name = "Front Wheel Drive Car With Slip Input"
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

        # graphic camera framing the car (not part of the EoM)
        self.camera_scale = 2.0 * self.params["length"]

        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["slip"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def slip2force(self, slip, params=None):
        params = self.params if params is None else params
        mu_max = params["mu_max"]
        mu_slope = params["mu_slope"]

        # sigmoid traction curve: friction ratio mu = |Fx / Fz| vs slip
        return mu_max * (2.0 / (1.0 + np.exp(-mu_slope * slip)) - 1.0)

    def acceleration(self, speed, slip, params=None):
        params = self.params if params is None else params
        length = params["length"]
        xc = params["xc"]
        yc = params["yc"]
        mass = params["mass"]
        gravity = params["gravity"]
        rho = params["rho"]
        cdA = params["cdA"]

        rr = xc / length  # ground share behind the c.g. (load on driven wheels)
        ry = yc / length  # c.g. height ratio driving the load transfer
        mu = self.slip2force(slip, params)
        drag = 0.5 * rho * cdA * speed * abs(speed)

        # longitudinal acceleration with dynamic load transfer onto the wheels
        return (mu * mass * gravity * rr - drag) / (mass * (1.0 + mu * ry))

    def f(self, x, u, t=0.0, params=None):
        speed = x[1]
        acceleration = self.acceleration(speed, u[0], params)

        # state derivative: position rate is speed, speed rate is acceleration
        return np.array([speed, acceleration])

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        length = self.params["length"]
        return {
            "world": [ground_line(length=12.0, y=-0.45)],
            "body": [vehicle_body(length=length, width=0.7, color="blue")],
            "wheel_rear": [wheel_box(length=0.35, width=0.18)],
            "wheel_front": [wheel_box(length=0.35, width=0.18)],
        }

    def tf(self, x, u, t=0, params=None):
        car_x = x[0]
        length = self.params["length"]
        return {
            "world": identity(),
            "body": translation(car_x, 0.0, 0.0),
            "wheel_rear": translation(car_x - 0.4 * length, -0.45, 0.0),
            "wheel_front": translation(car_x + 0.4 * length, -0.45, 0.0),
            "arrows": translation(car_x + 0.5 * length, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        force = self.slip2force(u[0]) if self.m == 1 else 0.0
        return {
            "arrows": [
                Arrow(
                    base=(0.0, 0.0),
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

        # graphic camera framing the car (not part of the EoM)
        self.camera_scale = 2.0 * self.params["length"]

        self.state.labels = ["x", "dx", "wheel_speed", "wheel_angle"]
        self.state.units = ["m", "m/s", "rad/s", "rad"]
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = ["slip"]
        self.x0 = np.array([0.0, 0.01, 0.0, 0.0])
        # v2 camera hint: this variant tracks the car (the slip-input base uses
        # the inherited fixed camera).
        self.camera_follow_frame = "body"

    def _slip(self, speed, wheel_speed, params=None):
        params = self.params if params is None else params
        wheel_radius = params["wheel_radius"]

        # tire slip ratio, clipped: contact-point speed vs ground speed
        denominator = abs(speed) + 1e-6
        return np.clip((wheel_radius * wheel_speed - speed) / denominator, -0.5, 0.5)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        rho = params["rho"]
        cdA = params["cdA"]
        wheel_radius = params["wheel_radius"]
        wheel_inertia = params["wheel_inertia"]

        speed = x[1]
        wheel_speed = x[2]
        torque = u[0]
        slip = self._slip(speed, wheel_speed, params)
        acceleration = self.acceleration(speed, slip, params)
        drag = 0.5 * rho * cdA * speed * abs(speed)

        # wheel spin-up: drive torque minus the traction + drag reaction torque
        wheel_acceleration = (
            torque - wheel_radius * (mass * acceleration + drag)
        ) / wheel_inertia

        return np.array([speed, acceleration, wheel_acceleration, wheel_speed])

    def h(self, x, u, t=0.0, params=None):
        return np.array([self._slip(x[1], x[2], params)])

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], 0.0, self.camera_scale)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        slip = self._slip(x[1], x[2])
        return super().get_dynamic_geometry(x, np.array([slip]), t)


if __name__ == "__main__":
    sys = LongitudinalFrontWheelDriveCarWithWheelSlipInput()
    sys.x0 = np.array([0.0, 20.0])
    sys.inputs["u"].nominal_value = np.array([-0.1])
    sys.compute_trajectory(tf=10.0)
    sys.plot_phase_plane()
    sys.animate()
