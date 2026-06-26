import numpy as np

from minilink.core.kinematics import identity_matrix, pose2d_matrix, translation_matrix
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.legacy import legacy_arrow_vector, legacy_body_arrow
from minilink.graphical.animation.primitives import Box, Circle, Sphere, vehicle_body, wheel_box


class KinematicBicycle(DynamicSystem):
    """Kinematic bicycle model with speed and steering-angle inputs."""

    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3, expose_state=True)
        self.name = "Kinematic Bicycle"
        self.params = {"length": 1.0}
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["speed", "steering"]
        self.inputs["u"].units = ["m/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters (not part of the EoM)
        self.width = 0.35
        self.tire_length = 0.25
        self.tire_width = 0.08
        self.camera_follow_frame = "body"
        self.camera_scale = 10.0

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        length = params["length"]
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

    def get_kinematic_geometry(self):
        length = self.params["length"]
        return {
            "body": [vehicle_body(length=length, width=self.width, color="blue")],
            "axle_rear": [wheel_box(self.tire_length, self.tire_width)],
            "axle_front": [wheel_box(self.tire_length, self.tire_width)],
        }

    def tf(self, x, u, t=0, params=None):
        length = self.params["length"]
        steering = u[1]
        rear_x = -0.5 * length
        front_x = 0.5 * length
        t_body = pose2d_matrix(x[0], x[1], x[2])
        return {
            "world": identity_matrix(x),
            "body": t_body,
            "axle_rear": t_body @ pose2d_matrix(rear_x, 0.0, 0.0),
            "axle_front": t_body @ pose2d_matrix(front_x, 0.0, steering),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        speed = u[0]
        t_body = pose2d_matrix(x[0], x[1], x[2])
        return {
            "world": [
                legacy_body_arrow(
                    t_body, 0.0, 0.0, 0.0, 0.4 * abs(speed), color="red", linewidth=2
                )
            ],
        }


class KinematicCar(KinematicBicycle):
    """Kinematic bicycle parameterized as a full-size car with a four-wheel skin."""

    def __init__(self):
        super().__init__()
        self.name = "Kinematic Car"
        self.a = 2.0
        self.b = 3.0
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
        wheel = wheel_box(self.tire_length, self.tire_width)
        return {
            "body": [body],
            "wheel_rl": [wheel],
            "wheel_rr": [wheel],
            "wheel_fl": [wheel],
            "wheel_fr": [wheel],
        }

    def tf(self, x, u, t=0, params=None):
        length = self.params["length"]
        steering = u[1]
        axle = 0.5 * self.visual_wheelbase_ratio * length
        half_track = 0.5 * self.width - 0.5 * self.tire_width
        t_body = pose2d_matrix(x[0], x[1], x[2])
        r_steer = pose2d_matrix(0.0, 0.0, steering)
        return {
            "body": t_body,
            "wheel_rl": t_body @ pose2d_matrix(-axle, half_track, 0.0),
            "wheel_rr": t_body @ pose2d_matrix(-axle, -half_track, 0.0),
            "wheel_fl": t_body @ pose2d_matrix(axle, half_track, 0.0) @ r_steer,
            "wheel_fr": t_body @ pose2d_matrix(axle, -half_track, 0.0) @ r_steer,
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        speed = u[0]
        t_body = pose2d_matrix(x[0], x[1], x[2])
        return {
            "world": [
                legacy_body_arrow(
                    t_body, 0.0, 0.0, 0.0, 0.4 * abs(speed), color="red", linewidth=2
                )
            ],
        }


class ConstantSpeedKinematicCar(DynamicSystem):
    """Kinematic car with constant speed and steering-angle input."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Kinematic Car"
        self.a = 2.0
        self.b = 3.0
        self.params = {"speed": 2.0, "length": self.a + self.b}
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["steering"]
        self.inputs["u"].units = ["rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters (not part of the EoM)
        self.width = 2.0
        self.tire_length = 0.25
        self.tire_width = 0.08
        self.camera_follow_frame = "body"
        self.camera_scale = 2.0 * self.params["length"]

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

    def get_kinematic_geometry(self):
        return KinematicBicycle.get_kinematic_geometry(self)

    def tf(self, x, u, t=0, params=None):
        steering = u[0]
        full_u = np.array([self.params["speed"], steering])
        return KinematicBicycle.tf(self, x, full_u, t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        steering = u[0]
        full_u = np.array([self.params["speed"], steering])
        return KinematicBicycle.get_dynamic_geometry(self, x, full_u, t, params)


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
        self.camera_follow_frame = "body"
        self.camera_scale = 10.0

    def f(self, x, u, t=0.0, params=None):
        # holonomic point: velocity command integrates straight to position
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {"body": [Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True)]}

    def tf(self, x, u, t=0, params=None):
        return {
            "world": identity_matrix(x),
            "body": translation_matrix(x[0], x[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "world": [
                legacy_arrow_vector(
                    x[0], x[1], u[0], u[1], scale=0.4, color="red", linewidth=2
                )
            ],
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
        self.camera_follow_frame = "body"
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 10.0

    def f(self, x, u, t=0.0, params=None):
        # holonomic point in 3D: velocity command integrates straight to position
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {"body": [Sphere(radius=0.25, color="blue", opacity=0.9)]}

    def tf(self, x, u, t=0, params=None):
        return {
            "world": identity_matrix(x),
            "body": translation_matrix(x[0], x[1], x[2]),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "world": [
                legacy_arrow_vector(
                    x[0], x[1], u[0], u[1], scale=0.4, color="red", linewidth=2
                )
            ],
        }


class UdeSRacecar(KinematicCar):
    """Small kinematic car with UdeS racecar-scale parameters."""

    def __init__(self):
        super().__init__()
        self.name = "UdeS Racecar"
        self.a = 0.17
        self.b = 0.17
        self.params["length"] = self.a + self.b

        # Graphic parameters (not part of the EoM)
        self.width = 0.17
        self.tire_length = 0.04
        self.tire_width = 0.015
        self.camera_scale = 2.0 * self.params["length"]


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
