import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.dynamics.catalog._graphics import (Arrow, Box, Circle, Point,
                                                 arrow_transform, drone_body,
                                                 follow_xy_camera, ground_line,
                                                 identity_matrix,
                                                 pose2d_matrix,
                                                 scale_pose2d_matrix,
                                                 translation_matrix)


class Drone2D(MechanicalSystem):
    """Planar drone with two vertical body-frame thrusters.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Drone"
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["T1", "T2"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 1.0
        self.inertia = 0.1
        self.thruster_offset = 0.5
        self.gravity = 9.81
        self.cda = 0.1
        self.width = 1.0
        self.height = 0.2
        self.camera_scale = 3.0

    def H(self, q, params=None):
        return np.diag([self.mass, self.mass, self.inertia])

    def C(self, q, dq, params=None):
        return np.zeros((3, 3))

    def B(self, q, params=None):
        theta = q[2]
        return np.array(
            [
                [-np.sin(theta), -np.sin(theta)],
                [np.cos(theta), np.cos(theta)],
                [-self.thruster_offset, self.thruster_offset],
            ]
        )

    def g(self, q, params=None):
        return np.array([0.0, self.mass * self.gravity, 0.0])

    def d(self, q, dq, u=None, t=0.0, params=None):
        return np.array(
            [
                self.cda * dq[0] * abs(dq[0]) + 0.01 * dq[0],
                self.cda * dq[1] * abs(dq[1]) + 0.01 * dq[1],
                0.01 * dq[2],
            ]
        )

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return [
            ground_line(length=20.0),
            drone_body(width=1.2, height=0.18),
            Point(color="blue", marker="o", size=5),
            Arrow(color="red", linewidth=2, origin="base"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[:3]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        scale = 0.08
        return [
            identity_matrix(),
            T_body,
            pose2d_matrix(q[0], q[1], 0.0),
            T_body
            @ scale_pose2d_matrix(
                -self.thruster_offset,
                0.0,
                np.pi / 2.0,
                scale * u[0],
            ),
            T_body
            @ scale_pose2d_matrix(
                self.thruster_offset,
                0.0,
                np.pi / 2.0,
                scale * u[1],
            ),
        ]


class Drone2DWithSideThruster(Drone2D):
    """Planar drone with an added lateral body-frame thruster.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__()
        self.name = "Planar Drone With Side Thruster"
        self.m = 0
        self.inputs.clear()
        self.add_input_port(
            "u",
            dim=3,
            labels=["T1", "T2", "TL"],
            units=["N", "N", "N"],
        )

    def B(self, q, params=None):
        B = np.zeros((3, 3))
        B[:, :2] = super().B(q, params)
        B[0, 2] = np.cos(q[2])
        B[1, 2] = np.sin(q[2])
        return B

    def get_kinematic_geometry(self):
        return super().get_kinematic_geometry() + [
            Arrow(color="orange", linewidth=2, origin="base")
        ]

    def get_kinematic_transforms(self, x, u, t):
        transforms = super().get_kinematic_transforms(x, u[:2], t)
        q = x[:3]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        transforms.append(T_body @ scale_pose2d_matrix(0.0, 0.0, 0.0, 0.08 * u[2]))
        return transforms


class SpeedControlledDrone2D(DynamicSystem):
    """Planar drone abstraction with velocity inputs.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Speed Controlled Planar Drone"
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.camera_scale = 10.0

    def f(self, x, u, t=0.0, params=None):
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return [
            drone_body(width=1.0, height=0.18),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        return [
            translation_matrix(x[0], x[1], 0.0),
            arrow_transform(x[0], x[1], u[0], u[1], scale=0.25),
        ]


class ConstantSpeedHelicopterTunnel(DynamicSystem):
    """Constant-speed tunnel helicopter with vertical force input.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Helicopter Tunnel"
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 1.0
        self.vx = 10.0
        self.camera_scale = 12.0

    def f(self, x, u, t=0.0, params=None):
        return np.array([u[0] / self.mass, x[0], self.vx])

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[2], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return [
            Box(length_x=1.0, length_y=0.35, length_z=0.2, color="blue", opacity=0.9),
            Circle(radius=0.08, center=[-0.3, -0.2, 0.0], color="black", fill=True),
            Circle(radius=0.08, center=[0.3, -0.2, 0.0], color="black", fill=True),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        return [
            translation_matrix(x[2], x[1], 0.0),
            translation_matrix(x[2], x[1], 0.0),
            translation_matrix(x[2], x[1], 0.0),
            arrow_transform(x[2] + 0.6, x[1], 0.0, u[0], scale=0.2),
        ]


if __name__ == "__main__":

    system = Drone2D()

    system.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])

    system.compute_forced(
        lambda t: np.array([5.0, 6.0]),
        tf=2.0,
        n_steps=80,
        show=True,
        verbose=False,
    )

    system.animate()
