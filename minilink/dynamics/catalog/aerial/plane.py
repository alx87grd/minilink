import numpy as np

from minilink.core.backends import array_module
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
        # static-framed primitives (the chord/force geometry is in the dynamic
        # hook so the legacy draw order survives the static/dynamic merge)
        return {
            "body": [self.body_shape()],
            "center": [Point(color="black", marker="o", size=5)],
            "wingchord": [self.chord_line()],
            "world": [ground_line(length=200.0, y=0.0, color="black", style="--")],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]
        params = self.params
        chord_w = np.sqrt(params["S_w"] / params["AR"])
        chord_t = np.sqrt(params["S_t"] / params["AR"])
        l_w = params["l_w"]
        l_t = params["l_t"]
        theta = q[2]
        delta = u[1]
        c, s = np.cos(theta), np.sin(theta)
        wing = np.array([q[0] - l_w * c, q[1] - l_w * s])
        tail = np.array([q[0] - l_t * c, q[1] - l_t * s])
        return {
            "world": identity_matrix(),
            "body": pose2d_matrix(q[0], q[1], q[2]),
            "center": pose2d_matrix(q[0], q[1], 0.0),
            "wingchord": pose2d_matrix(wing[0], wing[1], theta)
            @ scale_pose2d_matrix(-chord_w, 0.0, 0.0, 2.0 * chord_w),
            "thrust": pose2d_matrix(q[0], q[1], q[2])
            @ translation_matrix(-self.l_cg, 0.0, 0.0),
            "tailchord": pose2d_matrix(tail[0], tail[1], theta + delta)
            @ scale_pose2d_matrix(-chord_t, 0.0, 0.0, 2.0 * chord_t),
            "speed": translation_matrix(q[0], q[1], 0.0),
            "wingpt": translation_matrix(wing[0], wing[1], 0.0),
            "tailpt": translation_matrix(tail[0], tail[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        q = x[:3]
        dq = x[3:]
        speed, gamma, alpha = self.velocity_vector(q, dq)
        delta = u[1]
        L_w, D_w, _, L_t, D_t, _ = self.aerodynamic_forces(speed, alpha, delta)
        force_scale = self.length / 10.0
        thrust_len = force_scale * u[0]
        speed_len = min(speed * self.length / 30.0, self.length)
        cg, sg = np.cos(gamma), np.sin(gamma)
        return {
            "thrust": [
                Arrow(
                    base=(-thrust_len, 0.0),
                    vector=(1.0, 0.0),
                    scale=thrust_len,
                    color="red",
                    linewidth=2,
                )
            ],
            "tailchord": [self.chord_line()],
            "speed": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(cg, sg),
                    scale=speed_len,
                    color="black",
                    linewidth=2,
                )
            ],
            "wingpt": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(-L_w * sg, L_w * cg),
                    scale=force_scale,
                    color="blue",
                    linewidth=2,
                ),
                Arrow(
                    base=(0.0, 0.0),
                    vector=(-D_w * cg, -D_w * sg),
                    scale=force_scale,
                    color="red",
                    linewidth=2,
                ),
            ],
            "tailpt": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(-L_t * sg, L_t * cg),
                    scale=force_scale,
                    color="blue",
                    linewidth=2,
                ),
                Arrow(
                    base=(0.0, 0.0),
                    vector=(-D_t * cg, -D_t * sg),
                    scale=force_scale,
                    color="red",
                    linewidth=2,
                ),
            ],
        }


if __name__ == "__main__":
    sys = Plane2D()
    sys.x0 = np.array([0.0, 0.0, 0.0, 12.0, 0.0, 0.0])

    sys.compute_forced(lambda t: np.array([2 * t, -0.12 * t]), tf=10.0)

    sys.animate(time_factor_video=0.5)
