import numpy as np

from minilink.core.backends import array_module
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Point,
    arrow_transform,
    follow_xy_camera,
    ground_line,
    pose2d_matrix,
    scale_pose2d_matrix,
)


class Plane2D(MechanicalSystem):
    """Planar aircraft with thrust and elevator inputs."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Aircraft"
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["thrust", "delta"]
        self.inputs["u"].units = ["N", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.params = {
            "mass": 2.0,
            "inertia": 0.1,
            "gravity": 9.8,
            "rho": 1.29,
            "S_w": 0.2,
            "S_t": 0.05,
            "l_w": 0.0,
            "l_t": 1.0,
            "Cd0": 0.02,
            "AR": 5.0,
            "e_factor": 0.8,
            "Cl_alpha": 4.0,
            "Cm0": 0.0,
            "alpha_stall": np.pi / 12.0,
        }

        # Graphic parameters
        self.length = 2.0
        self.l_cg = 0.6 * self.length
        self.width = self.length / 10.0
        self.dynamic_range = self.length
        self.camera_scale = self.dynamic_range

    def velocity_vector(self, q, dq):
        xp = array_module(dq)
        speed = xp.sqrt(dq[0] ** 2 + dq[1] ** 2)
        gamma = xp.arctan2(dq[1], dq[0])
        alpha = q[2] - gamma
        return speed, gamma, alpha

    def Cl(self, alpha, params=None):
        params = self.params if params is None else params
        xp = array_module(alpha)
        alpha_stall = params["alpha_stall"]
        Cl_alpha = params["Cl_alpha"]

        cl = xp.sin(2.0 * alpha)
        cl = xp.where(xp.abs(alpha) < alpha_stall, cl + Cl_alpha * alpha, cl)
        return cl

    def Cd(self, alpha, params=None):
        params = self.params if params is None else params
        xp = array_module(alpha)
        alpha_stall = params["alpha_stall"]
        Cd0 = params["Cd0"]
        e = params["e_factor"]
        AR = params["AR"]

        cd = Cd0 + (1.0 - xp.cos(2.0 * alpha))
        cd = xp.where(
            xp.abs(alpha) < alpha_stall,
            cd + self.Cl(alpha, params) ** 2 / (xp.pi * e * AR),
            cd,
        )
        return cd

    def Cm(self, alpha, params=None):
        params = self.params if params is None else params
        return params["Cm0"]

    def aerodynamic_forces(self, speed, alpha, delta, params=None):
        params = self.params if params is None else params
        xp = array_module(speed)
        rho = params["rho"]
        S_w = params["S_w"]
        S_t = params["S_t"]
        AR = params["AR"]

        q_dyn = 0.5 * rho * speed**2
        chord_w = xp.sqrt(S_w / AR)
        chord_t = xp.sqrt(S_t / AR)

        L_w = q_dyn * S_w * self.Cl(alpha, params)
        D_w = q_dyn * S_w * self.Cd(alpha, params)
        M_w = q_dyn * S_w * chord_w * self.Cm(alpha, params)
        L_t = q_dyn * S_t * self.Cl(alpha + delta, params)
        D_t = q_dyn * S_t * self.Cd(alpha + delta, params)
        M_t = q_dyn * S_t * chord_t * self.Cm(alpha + delta, params)
        return L_w, D_w, M_w, L_t, D_t, M_t

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)
        mass = params["mass"]
        inertia = params["inertia"]

        return xp.diag(xp.array([mass, mass, inertia]))

    def C(self, q, dq, params=None):
        xp = array_module(q)
        return xp.zeros((3, 3))

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)
        mass = params["mass"]
        gravity = params["gravity"]

        return xp.array([0.0, mass * gravity, 0.0])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q)
        l_w = params["l_w"]
        l_t = params["l_t"]

        speed, gamma, alpha = self.velocity_vector(q, dq)
        delta = u[1]
        L_w, D_w, M_w, L_t, D_t, M_t = self.aerodynamic_forces(
            speed, alpha, delta, params
        )

        c_alpha, s_alpha = xp.cos(alpha), xp.sin(alpha)
        lift = L_w + L_t
        drag = D_w + D_t
        moment = (
            M_w
            + M_t
            - l_w * (L_w * c_alpha + D_w * s_alpha)
            - l_t * (L_t * c_alpha + D_t * s_alpha)
        )
        wind_load = xp.array([-drag, lift, moment])

        c_gamma, s_gamma = xp.cos(gamma), xp.sin(gamma)
        R = xp.array(
            [
                [c_gamma, -s_gamma, 0.0],
                [s_gamma, c_gamma, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        return -(R @ wind_load)

    def generalized_force(self, q, dq, u, t=0.0, params=None):
        xp = array_module(q)
        thrust = u[0]
        theta = q[2]

        return thrust * xp.array([xp.cos(theta), xp.sin(theta), 0.0])

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[0], x[1], self.camera_scale)

    def body_shape(self):
        """Side-view fuselage silhouette with the c.g. at the local origin.

        The fuselage spans ``length`` along local +X with a tail fin at the
        rear; the polygon is shifted by ``l_cg`` (tail-to-c.g. distance) so the
        c.g. sits at the origin used by the body-frame pose.
        """
        l = self.length
        w = self.width
        l_cg = self.l_cg
        pts = np.array(
            [
                [-l_cg, -0.5 * w, 0.0],
                [l - l_cg, -0.5 * w, 0.0],
                [l - w - l_cg, 0.5 * w, 0.0],
                [2.0 * w - l_cg, 0.5 * w, 0.0],
                [w - l_cg, 2.5 * w, 0.0],
                [-l_cg, 2.5 * w, 0.0],
                [-l_cg, -0.5 * w, 0.0],
            ]
        )
        return CustomLine(pts, color="blue", linewidth=2)

    def chord_line(self):
        """Unit chord segment along local +X (scaled per surface to the wing/tail chord)."""
        return CustomLine([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], color="blue", linewidth=2)

    def get_kinematic_geometry(self):
        return [
            self.body_shape(),
            Point(color="black", marker="o", size=5),
            self.chord_line(),
            ground_line(length=200.0, y=0.0, color="black", style="--"),
            Arrow(color="red", linewidth=2, origin="tip"),
            self.chord_line(),
            Arrow(color="black", linewidth=2, origin="base"),
            Arrow(color="blue", linewidth=2, origin="base"),
            Arrow(color="red", linewidth=2, origin="base"),
            Arrow(color="blue", linewidth=2, origin="base"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[:3]
        dq = x[3:]
        params = self.params
        speed, gamma, alpha = self.velocity_vector(q, dq)
        delta = u[1]
        L_w, D_w, _, L_t, D_t, _ = self.aerodynamic_forces(speed, alpha, delta)
        force_scale = self.length / 10.0
        chord_w = np.sqrt(params["S_w"] / params["AR"])
        chord_t = np.sqrt(params["S_t"] / params["AR"])
        l_w = params["l_w"]
        l_t = params["l_t"]
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)
        wing = np.array([q[0] - l_w * c, q[1] - l_w * s])
        tail = np.array([q[0] - l_t * c, q[1] - l_t * s])
        speed_len = min(speed * self.length / 30.0, self.length)
        return [
            pose2d_matrix(q[0], q[1], q[2]),
            pose2d_matrix(q[0], q[1], 0.0),
            pose2d_matrix(wing[0], wing[1], theta)
            @ scale_pose2d_matrix(-chord_w, 0.0, 0.0, 2.0 * chord_w),
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(q[0], q[1], q[2])
            @ scale_pose2d_matrix(-self.l_cg, 0.0, 0.0, force_scale * u[0]),
            pose2d_matrix(tail[0], tail[1], theta + delta)
            @ scale_pose2d_matrix(-chord_t, 0.0, 0.0, 2.0 * chord_t),
            scale_pose2d_matrix(q[0], q[1], gamma, speed_len),
            arrow_transform(
                wing[0],
                wing[1],
                -L_w * np.sin(gamma),
                L_w * np.cos(gamma),
                scale=force_scale,
            ),
            arrow_transform(
                wing[0],
                wing[1],
                -D_w * np.cos(gamma),
                -D_w * np.sin(gamma),
                scale=force_scale,
            ),
            arrow_transform(
                tail[0],
                tail[1],
                -L_t * np.sin(gamma),
                L_t * np.cos(gamma),
                scale=force_scale,
            ),
            arrow_transform(
                tail[0],
                tail[1],
                -D_t * np.cos(gamma),
                -D_t * np.sin(gamma),
                scale=force_scale,
            ),
        ]


if __name__ == "__main__":
    sys = Plane2D()
    sys.x0 = np.array([0.0, 0.0, 0.0, 12.0, 0.0, 0.0])

    sys.compute_forced(lambda t: np.array([2 * t, -0.12 * t]), tf=10.0)

    sys.animate(time_factor_video=0.5)
