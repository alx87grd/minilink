import numpy as np

from minilink.dynamics.abstraction.generalized_mechanical import \
    GeneralizedMechanicalSystem
from minilink.dynamics.catalog._graphics import (Arrow, Point, TorqueArrow,
                                                 arrow_transform, boat_body,
                                                 follow_xy_camera,
                                                 pose2d_matrix,
                                                 scale_pose2d_matrix,
                                                 torque_pose2d_matrix)


class Boat2D(GeneralizedMechanicalSystem):
    """Three-DoF planar boat with surge/sway force inputs.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=3, pos=3, actuators=2)
        self.name = "Planar Boat"
        self.state.labels = ["x", "y", "theta", "surge", "sway", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["Tx", "Ty"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 10000.0
        self.inertia = 10000.0
        self.l_t = 3.0
        self.damping_coef = np.array([10.0, 10.0, 100.0])
        self.Cx_max = 0.5
        self.Cy_max = 0.6
        self.Cm_max = 0.01
        self.N_max = 0.1
        self.rho = 1000.0
        self.Alc = self.l_t * 2.0
        self.Afc = 0.25 * self.Alc
        self.loa = self.l_t * 2.0
        self.camera_scale = 3.0 * self.loa
        self.show_hydrodynamic_forces = False

    def M(self, q, params=None):
        return np.diag([self.mass, self.mass, self.inertia])

    def C(self, q, v, params=None):
        yaw_rate = v[2]
        return np.array(
            [
                [0.0, -self.mass * yaw_rate, 0.0],
                [self.mass * yaw_rate, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ]
        )

    def N(self, q, params=None):
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def B(self, q, params=None):
        return np.array([[1.0, 0.0], [0.0, 1.0], [0.0, -self.l_t]])

    def current_coefficients(self, alpha):
        Cx = -self.Cx_max * np.cos(alpha) * np.abs(np.cos(alpha))
        Cy = self.Cy_max * np.sin(alpha) * np.abs(np.sin(alpha))
        Cm = self.Cm_max * np.sin(2.0 * alpha)
        return Cx, Cy, Cm

    def damping(self, relative_velocity):
        d_linear = relative_velocity * self.damping_coef
        speed_squared = relative_velocity[0] ** 2 + relative_velocity[1] ** 2
        alpha = -np.arctan2(relative_velocity[1], relative_velocity[0])
        Cx, Cy, Cm = self.current_coefficients(alpha)
        fx = -0.5 * self.rho * self.Afc * Cx * speed_squared
        fy = -0.5 * self.rho * self.Alc * Cy * speed_squared
        mz = -0.5 * self.rho * self.Alc * self.loa * Cm * speed_squared
        mz += (
            self.N_max
            * self.rho
            * self.Alc
            * self.loa
            * np.abs(relative_velocity[2])
            * relative_velocity[2]
        )
        return d_linear + np.array([fx, fy, mz])

    def d(self, q, v, u=None, t=0.0, params=None):
        return self.damping(v)

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        geometry = [
            boat_body(length=2.0 * self.l_t, width=self.Afc),
            Point(color="blue", marker="o", size=5),
            Arrow(color="red", linewidth=2, origin="tip"),
        ]
        if self.show_hydrodynamic_forces:
            geometry.extend(
                [
                    Arrow(color="black", linewidth=2, style="--", origin="base"),
                    TorqueArrow(
                        radius=self.loa / 5.0,
                        head_ratio=0.4,
                        color="black",
                        linewidth=2,
                        style="--",
                    ),
                ]
            )
        return geometry

    def get_kinematic_transforms(self, x, u, t):
        q = x[:3]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        force_scale = 0.0002
        transforms = [
            T_body,
            pose2d_matrix(q[0], q[1], 0.0),
            T_body
            @ scale_pose2d_matrix(
                -self.l_t,
                0.0,
                np.arctan2(u[1], u[0]),
                force_scale * np.hypot(u[0], u[1]),
            ),
        ]
        if self.show_hydrodynamic_forces:
            hydro_force = -self.d(q, x[3:], u, t)
            torque_max = abs(0.5 * self.rho * self.Alc * self.loa * self.Cm_max * 12.0)
            transforms.extend(
                [
                    T_body
                    @ scale_pose2d_matrix(
                        0.0,
                        0.0,
                        np.arctan2(hydro_force[1], hydro_force[0]),
                        force_scale * np.hypot(hydro_force[0], hydro_force[1]),
                    ),
                    torque_pose2d_matrix(
                        q[0],
                        q[1],
                        q[2] - np.pi / 2.0,
                        hydro_force[2] * (2.0 * np.pi) / torque_max,
                    ),
                ]
            )
        return transforms


class Boat2DWithCurrent(Boat2D):
    """Planar boat with constant current velocity in world frame.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__()
        self.name = "Planar Boat With Current"
        self.current_velocity = np.array([-1.0, -1.0])

    def d(self, q, v, u=None, t=0.0, params=None):
        world_current = np.array(
            [self.current_velocity[0], self.current_velocity[1], 0.0]
        )
        body_current = self.N(q).T @ world_current
        return self.damping(v - body_current)

    def get_kinematic_geometry(self):
        return super().get_kinematic_geometry() + [
            Arrow(color="green", linewidth=2, origin="tip")
        ]

    def get_kinematic_transforms(self, x, u, t):
        transforms = super().get_kinematic_transforms(x, u, t)
        transforms.append(
            arrow_transform(
                x[0] - self.loa,
                x[1] + self.loa,
                self.current_velocity[0],
                self.current_velocity[1],
                scale=0.5 * self.loa,
            )
        )
        return transforms


if __name__ == "__main__":
    system = Boat2D()
    system.compute_forced(
        lambda t: np.array([1000.0, 50.0 * np.sin(0.5 * t)]),
        tf=10.0,
        n_steps=240,
        show=True,
        verbose=False,
    )
