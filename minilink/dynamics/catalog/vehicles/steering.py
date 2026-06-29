import numpy as np

from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    Sphere,
    arrow_transform,
    camera_matrix,
    follow_xy_camera,
    pose2d_matrix,
    scale_pose2d_matrix,
    translation_matrix,
    vehicle_body,
    wheel_box,
)
from minilink.graphical.animation.shapes_v2 import ArrowV2


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
        self.camera_scale = 10.0
        # v2 camera hint: track the body frame (read by ``Animator2`` only).
        self.camera_follow_frame = "body"

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

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        length = self.params["length"]
        return [
            vehicle_body(length=length, width=self.width, color="blue"),
            wheel_box(self.tire_length, self.tire_width),
            wheel_box(self.tire_length, self.tire_width),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        length = self.params["length"]
        speed, steering = u[0], u[1]
        rear_x = -0.5 * length
        front_x = 0.5 * length
        T_body = pose2d_matrix(x[0], x[1], x[2])
        return [
            T_body,
            T_body @ pose2d_matrix(rear_x, 0.0, 0.0),
            T_body @ pose2d_matrix(front_x, 0.0, steering),
            T_body @ scale_pose2d_matrix(0.0, 0.0, 0.0, 0.4 * abs(speed)),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        length = self.params["length"]
        return {
            "body": [vehicle_body(length=length, width=self.width, color="blue")],
            "axle_rear": [wheel_box(self.tire_length, self.tire_width)],
            "axle_front": [wheel_box(self.tire_length, self.tire_width)],
        }

    def tf_v2(self, x, u, t=0, params=None):
        length = self.params["length"]
        steering = u[1]
        rear_x = -0.5 * length
        front_x = 0.5 * length
        T_body = SE2(x[0], x[1], x[2])
        return {
            "body": T_body,
            "axle_rear": T_body @ SE2(rear_x, 0.0, 0.0),
            "axle_front": T_body @ SE2(front_x, 0.0, steering),
            "arrows": T_body,
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        speed = u[0]
        return {
            "arrows": [
                ArrowV2(
                    base=(0.0, 0.0),
                    vector=(1.0, 0.0),
                    scale=0.4 * abs(speed),
                    color="red",
                    linewidth=2,
                )
            ]
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
        return [
            body,
            wheel_box(self.tire_length, self.tire_width),  # rear-left
            wheel_box(self.tire_length, self.tire_width),  # rear-right
            wheel_box(self.tire_length, self.tire_width),  # front-left (steers)
            wheel_box(self.tire_length, self.tire_width),  # front-right (steers)
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        length = self.params["length"]
        speed, steering = u[0], u[1]
        axle = 0.5 * self.visual_wheelbase_ratio * length
        half_track = (
            0.5 * self.width - 0.5 * self.tire_width
        )  # tire flush with the side
        T_body = pose2d_matrix(x[0], x[1], x[2])
        R_steer = pose2d_matrix(0.0, 0.0, steering)
        return [
            T_body,
            T_body @ pose2d_matrix(-axle, half_track, 0.0),
            T_body @ pose2d_matrix(-axle, -half_track, 0.0),
            T_body @ pose2d_matrix(axle, half_track, 0.0) @ R_steer,
            T_body @ pose2d_matrix(axle, -half_track, 0.0) @ R_steer,
            T_body @ scale_pose2d_matrix(0.0, 0.0, 0.0, 0.4 * abs(speed)),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        length = self.params["length"]
        body = Box(
            length_x=length,
            length_y=self.body_width_ratio * self.width,
            length_z=0.4,
            color="#4c72b0",
            opacity=0.9,
        )
        return {
            "body": [body],
            "wheel_rl": [wheel_box(self.tire_length, self.tire_width)],
            "wheel_rr": [wheel_box(self.tire_length, self.tire_width)],
            "wheel_fl": [wheel_box(self.tire_length, self.tire_width)],
            "wheel_fr": [wheel_box(self.tire_length, self.tire_width)],
        }

    def tf_v2(self, x, u, t=0, params=None):
        length = self.params["length"]
        steering = u[1]
        axle = 0.5 * self.visual_wheelbase_ratio * length
        half_track = 0.5 * self.width - 0.5 * self.tire_width
        T_body = SE2(x[0], x[1], x[2])
        R_steer = SE2(0.0, 0.0, steering)
        return {
            "body": T_body,
            "wheel_rl": T_body @ SE2(-axle, half_track, 0.0),
            "wheel_rr": T_body @ SE2(-axle, -half_track, 0.0),
            "wheel_fl": T_body @ SE2(axle, half_track, 0.0) @ R_steer,
            "wheel_fr": T_body @ SE2(axle, -half_track, 0.0) @ R_steer,
            "arrows": T_body,
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
        self.camera_scale = 2.0 * self.params["length"]
        self.camera_follow_frame = "body"

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

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return KinematicBicycle.get_kinematic_geometry(self)

    def get_kinematic_transforms(self, x, u, t):
        steering = u[0]
        full_u = np.array([self.params["speed"], steering])
        return KinematicBicycle.get_kinematic_transforms(self, x, full_u, t)

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return KinematicBicycle.get_kinematic_geometry_v2(self)

    def tf_v2(self, x, u, t=0, params=None):
        full_u = np.array([self.params["speed"], u[0]])
        return KinematicBicycle.tf_v2(self, x, full_u, t)

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        full_u = np.array([self.params["speed"], u[0]])
        return KinematicBicycle.get_dynamic_geometry_v2(self, x, full_u, t)


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

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return [
            Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        return [
            translation_matrix(x[0], x[1], 0.0),
            arrow_transform(x[0], x[1], u[0], u[1], scale=0.4),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {
            "body": [
                Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True)
            ]
        }

    def tf_v2(self, x, u, t=0, params=None):
        T = translation(x[0], x[1], 0.0)
        return {"body": T, "arrows": T}

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        return {
            "arrows": [
                ArrowV2(
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

    def get_camera_transform(self, x, u, t):
        return camera_matrix(
            target=(x[0], x[1], x[2]),
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )

    def get_kinematic_geometry(self):
        return [
            Sphere(radius=0.25, color="blue", opacity=0.9),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        return [
            translation_matrix(x[0], x[1], x[2]),
            arrow_transform(x[0], x[1], u[0], u[1], scale=0.4),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {"body": [Sphere(radius=0.25, color="blue", opacity=0.9)]}

    def tf_v2(self, x, u, t=0, params=None):
        return {
            "body": translation(x[0], x[1], x[2]),
            "arrows": translation(x[0], x[1], 0.0),
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        return {
            "arrows": [
                ArrowV2(
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
