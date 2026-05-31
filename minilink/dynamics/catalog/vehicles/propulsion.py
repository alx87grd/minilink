import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog._graphics import (Arrow, follow_xy_camera,
                                                 ground_line, identity_matrix,
                                                 scale_pose2d_matrix,
                                                 translation_matrix,
                                                 vehicle_body, wheel_box)


class LongitudinalFrontWheelDriveCarWithWheelSlipInput(DynamicSystem):
    """Longitudinal front-wheel-drive car with wheel slip as input.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.name = "Front Wheel Drive Car With Slip Input"
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["slip"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self._set_physical_defaults()

    def _set_physical_defaults(self):
        self.length = 2.0
        self.xc = 1.0
        self.yc = 0.5
        self.mass = 1500.0
        self.gravity = 9.81
        self.rho = 1.225
        self.cdA = 0.3 * 2.0
        self.mu_max = 1.0
        self.mu_slope = 70.0
        self.camera_scale = 2.0 * self.length

    def ratios(self):
        ry = self.yc / self.length
        rr = self.xc / self.length
        rf = 1.0 - rr
        return ry, rr, rf

    def slip2force(self, slip):
        return self.mu_max * (2.0 / (1.0 + np.exp(-self.mu_slope * slip)) - 1.0)

    def acceleration(self, speed, slip):
        ry, rr, _ = self.ratios()
        drag = 0.5 * self.rho * self.cdA * speed * abs(speed)
        mu = self.slip2force(slip)
        return (mu * self.mass * self.gravity * rr - drag) / (
            self.mass * (1.0 + mu * ry)
        )

    def f(self, x, u, t=0.0, params=None):
        speed = x[1]
        return np.array([speed, self.acceleration(speed, u[0])])

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return [
            ground_line(length=12.0, y=-0.45),
            vehicle_body(length=self.length, width=0.7, color="blue"),
            wheel_box(length=0.35, width=0.18),
            wheel_box(length=0.35, width=0.18),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        car_x = x[0]
        force = self.slip2force(u[0]) if self.m == 1 else 0.0
        return [
            identity_matrix(),
            translation_matrix(car_x, 0.0, 0.0),
            translation_matrix(car_x - 0.4 * self.length, -0.45, 0.0),
            translation_matrix(car_x + 0.4 * self.length, -0.45, 0.0),
            scale_pose2d_matrix(
                car_x + 0.5 * self.length,
                0.0,
                0.0 if force >= 0.0 else np.pi,
                abs(force),
            ),
        ]


class LongitudinalFrontWheelDriveCarWithTorqueInput(
    LongitudinalFrontWheelDriveCarWithWheelSlipInput
):
    """Longitudinal front-wheel-drive car with wheel torque input.

    TRL: 1 - ready for user review.
    """

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
        self.state.labels = ["x", "dx", "wheel_speed", "wheel_angle"]
        self.state.units = ["m", "m/s", "rad/s", "rad"]
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = ["slip"]
        self._set_physical_defaults()
        self.wheel_radius = 0.3
        self.wheel_inertia = 1.5
        self.x0 = np.array([0.0, 0.01, 0.0, 0.0])

    def _slip(self, speed, wheel_speed):
        denominator = abs(speed) + 1e-6
        return np.clip(
            (self.wheel_radius * wheel_speed - speed) / denominator,
            -0.5,
            0.5,
        )

    def f(self, x, u, t=0.0, params=None):
        speed = x[1]
        wheel_speed = x[2]
        torque = u[0]
        slip = self._slip(speed, wheel_speed)
        acceleration = self.acceleration(speed, slip)
        drag = 0.5 * self.rho * self.cdA * speed * abs(speed)
        wheel_acceleration = (
            torque - self.wheel_radius * (self.mass * acceleration + drag)
        ) / self.wheel_inertia
        return np.array([speed, acceleration, wheel_acceleration, wheel_speed])

    def h(self, x, u, t=0.0, params=None):
        return np.array([self._slip(x[1], x[2])])

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], 0.0, self.camera_scale)

    def get_kinematic_transforms(self, x, u, t):
        slip = self._slip(x[1], x[2])
        return super().get_kinematic_transforms(x, np.array([slip]), t)


if __name__ == "__main__":
    system = LongitudinalFrontWheelDriveCarWithWheelSlipInput()
    system.compute_forced(
        lambda t: np.array([0.05]),
        tf=5.0,
        n_steps=160,
        show=True,
        verbose=False,
    )
