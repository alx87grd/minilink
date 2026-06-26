import numpy as np

from minilink.core.kinematics import identity_matrix, pose2d_matrix, translation_matrix
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.legacy import legacy_arrow_vector, legacy_body_arrow
from minilink.graphical.animation.primitives import CustomLine, Point, ground_line


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
        self.camera_follow_frame = "body"
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

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=200.0, y=0.0, color="black", style="--")],
            "body": [self.body_shape()],
            "cg": [Point(color="black", marker="o", size=5)],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]
        t_body = pose2d_matrix(q[0], q[1], q[2])
        return {
            "world": identity_matrix(x),
            "body": t_body,
            "cg": pose2d_matrix(q[0], q[1], 0.0),
            "engine": t_body @ translation_matrix(0.0, -1.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        q = x[:3]
        t_body = pose2d_matrix(q[0], q[1], q[2])
        delta = u[1]
        thrust = u[0]
        return {
            "world": [
                legacy_body_arrow(
                    t_body,
                    0.0,
                    -1.0,
                    np.pi / 2.0 + delta,
                    0.0002 * thrust,
                    color="red",
                    linewidth=2,
                )
            ],
        }

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


if __name__ == "__main__":
    sys = Rocket()

    sys.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([15000.0, 0.01])

    sys.compute_trajectory(tf=3.0)
    sys.animate()
