import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.dynamics.catalog._graphics import (Arrow, Point,
                                                 follow_xy_camera, ground_line,
                                                 identity_matrix,
                                                 pose2d_matrix, rocket_body,
                                                 scale_pose2d_matrix)


class Rocket(MechanicalSystem):
    """Planar rocket with thrust magnitude and gimbal angle inputs.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Rocket"
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["thrust", "delta"]
        self.inputs["u"].units = ["N", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 1000.0
        self.inertia = 100.0
        self.ycg = 1.0
        self.gravity = 9.8
        self.cda = 1.0
        self.width = 0.4
        self.height = 2.0
        self.dynamic_range = 10.0
        self.camera_scale = self.dynamic_range

    def H(self, q, params=None):
        return np.diag([self.mass, self.mass, self.inertia])

    def C(self, q, dq, params=None):
        return np.zeros((3, 3))

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

    def generalized_force(self, q, dq, u, t=0.0, params=None):
        thrust, delta = u
        theta = q[2]
        return thrust * np.array(
            [
                -np.sin(theta + delta),
                np.cos(theta + delta),
                -self.ycg * np.sin(delta),
            ]
        )

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        return [
            rocket_body(width=self.width, height=self.height),
            Point(color="black", marker="o", size=5),
            ground_line(length=200.0, y=0.0, color="black", style="--"),
            Arrow(color="red", linewidth=2, origin="tip"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[:3]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        return [
            T_body,
            pose2d_matrix(q[0], q[1], 0.0),
            identity_matrix(),
            T_body
            @ scale_pose2d_matrix(
                0.0,
                -1.0,
                np.pi / 2.0 + u[1],
                0.0002 * u[0],
            ),
        ]


if __name__ == "__main__":

    sys = Rocket()

    sys.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([15000.0, 0.01])

    sys.compute_trajectory(tf=3.0)
    sys.animate()
