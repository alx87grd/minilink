import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Point,
    follow_xy_camera,
    ground_line,
    identity_matrix,
    pose2d_matrix,
    scale_pose2d_matrix,
    translation_matrix,
)
from minilink.graphical.animation.shapes_v2 import ArrowV2


class Rocket(MechanicalSystem):
    """Planar rocket with thrust magnitude and gimbal angle inputs."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Rocket"
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["thrust", "delta"]
        self.inputs["u"].units = ["N", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.params = {
            "mass": 1000.0,
            "inertia": 100.0,
            "ycg": 1.0,
            "gravity": 9.8,
            "cda": 1.0,
        }

        # Graphic parameters
        self.width = 0.4
        self.height = 2.0
        self.dynamic_range = 10.0
        self.camera_scale = self.dynamic_range

    def H(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        return np.diag([mass, mass, inertia])

    def C(self, q, dq, params=None):
        return np.zeros((3, 3))

    def g(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        gravity = params["gravity"]

        # weight pulls along +y (d sits on the left side of the EoM)
        return np.array([0.0, mass * gravity, 0.0])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        cda = params["cda"]

        # quadratic aerodynamic drag plus a small linear damping term
        return np.array(
            [
                cda * dq[0] * abs(dq[0]) + 0.01 * dq[0],
                cda * dq[1] * abs(dq[1]) + 0.01 * dq[1],
                0.01 * dq[2],
            ]
        )

    def generalized_force(self, q, dq, u, t=0.0, params=None):
        params = self.params if params is None else params
        ycg = params["ycg"]
        thrust, delta = u
        theta = q[2]

        # gimballed thrust: force along the nozzle axis, torque about the c.g.
        return thrust * np.array(
            [
                -np.sin(theta + delta),
                np.cos(theta + delta),
                -ycg * np.sin(delta),
            ]
        )

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def body_shape(self):
        """Side-view rocket silhouette with the c.g. at the local origin."""
        w = self.width
        h = self.height
        pts = np.array(
            [
                [-0.5 * w, -0.5 * h, 0.0],
                [-0.5 * w, 0.35 * h, 0.0],
                [0.0, 0.5 * h, 0.0],
                [0.5 * w, 0.35 * h, 0.0],
                [0.5 * w, -0.5 * h, 0.0],
                [-0.5 * w, -0.5 * h, 0.0],
            ]
        )
        return CustomLine(pts, color="blue", linewidth=2)

    def get_kinematic_geometry(self):
        return [
            self.body_shape(),
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

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {
            "body": [self.body_shape()],
            "center": [Point(color="black", marker="o", size=5)],
            "world": [ground_line(length=200.0, y=0.0, color="black", style="--")],
        }

    def tf_v2(self, x, u, t=0, params=None):
        q = x[:3]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        return {
            "world": identity_matrix(),
            "body": T_body,
            "center": pose2d_matrix(q[0], q[1], 0.0),
            "thrust": T_body @ translation_matrix(0.0, -1.0, 0.0),
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        length = 0.0002 * u[0]
        angle = np.pi / 2.0 + u[1]
        d = np.array([np.cos(angle), np.sin(angle)])
        return {
            "thrust": [
                ArrowV2(
                    base=-length * d,
                    vector=d,
                    scale=length,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    sys = Rocket()

    sys.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([15000.0, 0.01])

    sys.compute_trajectory(tf=3.0)
    sys.animate()
