import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, SE3, Rx, Ry, Rz, translation
from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Point,
    ground_line,
)
from minilink.graphical.catalog.shapes import segment_pose_2d
from minilink.graphical.catalog.skins import plane_skin_3d


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
        self.camera_follow_frame = "body"

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
        # Static-framed primitives; chord/force geometry lives in the dynamic hook
        # so draw order matches the pre-merge layout.
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
        dir_w = np.array([c, s])
        dir_t = np.array([np.cos(theta + delta), np.sin(theta + delta)])
        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
            "wingchord": segment_pose_2d(
                wing - chord_w * dir_w, wing + chord_w * dir_w
            ),
            "tailchord": segment_pose_2d(
                tail - chord_t * dir_t, tail + chord_t * dir_t
            ),
            "speed": translation(q[0], q[1], 0.0),
            "wingpt": translation(wing[0], wing[1], 0.0),
            "tailpt": translation(tail[0], tail[1], 0.0),
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
        thrust = Arrow(
            base=(-thrust_len, 0.0),
            vector=(1.0, 0.0),
            scale=thrust_len,
            color="red",
            linewidth=2,
        )
        thrust.local_transform = translation(-self.l_cg, 0.0, 0.0)
        return {
            "body": [thrust],
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


class Plane3D(GeneralizedMechanicalSystem):
    """Six-DoF aircraft with thrust, elevator, aileron, and rudder.

    Configuration ``q = [x, y, z, phi, theta, psi]`` is world position (``z`` up)
    plus ZYX Euler angles. Generalized velocity
    ``v = [u, v, w, p, q, r]`` is the body-frame linear and angular rate
    (aircraft body: ``+x`` forward, ``+y`` right, ``+z`` down).

    The body-to-world rotation is ``R_wb = Rz(psi) @ Ry(theta) @ Rx(phi) @ Rx(pi)``
    so that zero angles place the aircraft level with belly toward world ``-z``.
    With this map, a horizontal flight path at angle of attack ``alpha`` uses
    pitch ``theta ≈ -alpha``. Aerodynamics reuse the planar ``Cl`` / ``Cd`` /
    ``Cm`` laws on the wing and horizontal tail, plus a vertical fin (rudder)
    and a pure aileron roll moment.
    """

    def __init__(self):
        super().__init__(dof=6, pos=6, actuators=4)
        self.name = "Aircraft 3D"
        self.state.labels = [
            "x",
            "y",
            "z",
            "phi",
            "theta",
            "psi",
            "u",
            "v",
            "w",
            "p",
            "q",
            "r",
        ]
        self.state.units = [
            "m",
            "m",
            "m",
            "rad",
            "rad",
            "rad",
            "m/s",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad/s",
        ]
        self.inputs["u"].labels = ["thrust", "delta_e", "delta_a", "delta_r"]
        self.inputs["u"].units = ["N", "rad", "rad", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.params = {
            # Light-GA scale: heavier + larger inertias → slower angular response.
            "mass": 80.0,
            "inertia_xx": 80.0,
            "inertia_yy": 180.0,
            "inertia_zz": 220.0,
            "gravity": 9.8,
            "rho": 1.29,
            # Large wing and generous tail volumes for natural static stability.
            "S_w": 12.0,
            "S_t": 2.8,
            "S_v": 2.2,
            "l_w": 0.0,
            "l_t": 4.5,
            "l_v": 4.5,
            "b_w": 10.0,
            "Cd0": 0.025,
            "AR": 8.0,
            "e_factor": 0.85,
            "Cl_alpha": 3.0,
            # Nose-up bias so neutral stick trims near a lifting α (1-g cruise).
            "Cm0": 0.22,
            "Cl_da": 0.06,
            "alpha_stall": np.pi / 10.0,
        }

        self.length = 8.0
        self.l_cg = 0.55 * self.length
        self.width = self.length / 12.0
        self.span = self.params["b_w"]
        self.dynamic_range = self.length
        self.camera_scale = 4.0 * self.length
        self.camera_follow_frame = "body"
        self.ground_plane_size = 200.0
        self.skin = plane_skin_3d

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

    def R_wb(self, q):
        """Body-to-world rotation for configuration ``q``."""
        xp = array_module(q)
        phi, theta, psi = q[3], q[4], q[5]
        return Rz(psi) @ Ry(theta) @ Rx(phi) @ Rx(xp.pi)

    def velocity_angles(self, v):
        """Airspeed, angle of attack, and sideslip from body velocity."""
        xp = array_module(v)
        u, vb, w = v[0], v[1], v[2]
        speed = xp.sqrt(u**2 + vb**2 + w**2)
        alpha = xp.arctan2(w, u)
        beta = xp.arctan2(vb, xp.sqrt(u**2 + w**2))
        return speed, alpha, beta

    def aerodynamic_forces(
        self, speed, alpha, beta, delta_e, delta_a, delta_r, params=None
    ):
        """Body-frame force and moment from the simple multi-surface aero model."""
        params = self.params if params is None else params
        xp = array_module(speed)
        rho = params["rho"]
        S_w = params["S_w"]
        S_t = params["S_t"]
        S_v = params["S_v"]
        AR = params["AR"]
        l_w = params["l_w"]
        l_t = params["l_t"]
        l_v = params["l_v"]
        b_w = params["b_w"]
        Cl_da = params["Cl_da"]

        q_dyn = 0.5 * rho * speed**2
        chord_w = xp.sqrt(S_w / AR)
        chord_t = xp.sqrt(S_t / AR)

        # Wing + horizontal tail (longitudinal), same Cl/Cd/Cm as Plane2D.
        L_w = q_dyn * S_w * self.Cl(alpha, params)
        D_w = q_dyn * S_w * self.Cd(alpha, params)
        M_w = q_dyn * S_w * chord_w * self.Cm(alpha, params)
        L_t = q_dyn * S_t * self.Cl(alpha + delta_e, params)
        D_t = q_dyn * S_t * self.Cd(alpha + delta_e, params)
        M_t = q_dyn * S_t * chord_t * self.Cm(alpha + delta_e, params)

        c_a, s_a = xp.cos(alpha), xp.sin(alpha)
        # Wind-axis [-D, 0, -L] rotated by alpha about body y into the body frame.
        F_w = xp.array([-c_a * D_w - s_a * L_w, 0.0, s_a * D_w - c_a * L_w])
        F_t = xp.array([-c_a * D_t - s_a * L_t, 0.0, s_a * D_t - c_a * L_t])

        # Vertical fin + rudder (lateral): Cl/Cd only — no Cm0 (that trims pitch).
        L_v = q_dyn * S_v * self.Cl(beta + delta_r, params)
        D_v = q_dyn * S_v * self.Cd(beta + delta_r, params)
        c_b, s_b = xp.cos(beta), xp.sin(beta)
        F_v = xp.array([-c_b * D_v + s_b * L_v, -s_b * D_v - c_b * L_v, 0.0])

        force = F_w + F_t + F_v

        # Moments: surface Cm plus r × F for stations on the body -x axis.
        moment_y = M_w + M_t + l_w * F_w[2] + l_t * F_t[2]
        moment_z = -l_v * F_v[1]
        moment_x = q_dyn * S_w * b_w * Cl_da * delta_a
        moment = xp.array([moment_x, moment_y, moment_z])
        return force, moment

    def M(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)
        mass = params["mass"]
        Ixx = params["inertia_xx"]
        Iyy = params["inertia_yy"]
        Izz = params["inertia_zz"]
        return xp.diag(xp.array([mass, mass, mass, Ixx, Iyy, Izz]))

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)
        mass = params["mass"]
        Ixx = params["inertia_xx"]
        Iyy = params["inertia_yy"]
        Izz = params["inertia_zz"]
        p, q_rate, r = v[3], v[4], v[5]

        # Rigid-body Coriolis in body axes: m ω×v_lin and ω×(Iω).
        # fmt: off
        return xp.array([
            [        0.0, -mass * r,  mass * q_rate, 0.0, 0.0, 0.0],
            [  mass * r,        0.0, -mass * p, 0.0, 0.0, 0.0],
            [-mass * q_rate,  mass * p,        0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0,          0.0,   Izz * r, -Iyy * q_rate],
            [0.0, 0.0, 0.0,   -Izz * r,        0.0,  Ixx * p],
            [0.0, 0.0, 0.0, Iyy * q_rate, -Ixx * p,        0.0],
        ])
        # fmt: on

    def N(self, q, params=None):
        xp = array_module(q)
        phi, theta = q[3], q[4]
        R = self.R_wb(q)

        s_phi, c_phi = xp.sin(phi), xp.cos(phi)
        s_th, c_th = xp.sin(theta), xp.cos(theta)
        t_th = s_th / c_th
        # Euler rates from the intermediate (pre-Rx(pi)) rates [p, -q, -r].
        # fmt: off
        E = xp.array([
            [1.0, s_phi * t_th,  c_phi * t_th],
            [0.0,        c_phi,        -s_phi],
            [0.0, s_phi / c_th, c_phi / c_th],
        ])
        S = xp.diag(xp.array([1.0, -1.0, -1.0]))
        # fmt: on
        E_body = E @ S

        z03 = xp.zeros((3, 3))
        top = xp.concatenate([R, z03], axis=1)
        bottom = xp.concatenate([z03, E_body], axis=1)
        return xp.concatenate([top, bottom], axis=0)

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)
        mass = params["mass"]
        gravity = params["gravity"]
        R = self.R_wb(q)
        # Left-side potential gradient (same convention as Plane2D): world +z up
        # so g_world = [0, 0, m g], then express in body axes.
        g_world = xp.array([0.0, 0.0, mass * gravity])
        g_body = R.T @ g_world
        return xp.concatenate([g_body, xp.zeros(3)])

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(v)
        speed, alpha, beta = self.velocity_angles(v)
        delta_e = u[1]
        delta_a = u[2]
        delta_r = u[3]
        force, moment = self.aerodynamic_forces(
            speed, alpha, beta, delta_e, delta_a, delta_r, params
        )
        wrench = xp.concatenate([force, moment])
        return -wrench

    def generalized_force(self, q, v, u, t=0.0, params=None):
        xp = array_module(q)
        thrust = u[0]
        return xp.array([thrust, 0.0, 0.0, 0.0, 0.0, 0.0])

    def tf(self, x, u, t=0, params=None):
        q = x[:6]
        p = q[:3]
        R = self.R_wb(q)
        R_np = np.asarray(R)
        p_np = np.asarray(p)
        return {"body": SE3(R_np, p_np)}


if __name__ == "__main__":
    from minilink.simulation.realtime import PygameInput, RealtimeSimulator

    sys = Plane3D()
    # Neutral-stick 1-g cruise trim (de=da=dr=0): α≈0.058, V≈16.9 m/s, T≈198 N.
    # Face −x (psi=π) so the default meshcat view sits behind the tail.
    # Hold W for trim thrust — release cuts power (hold mode → 0).
    alpha = 0.0577
    u_b = 16.92
    T_trim = 198.1
    T_max = 1000.0  # W commands exact cruise thrust
    sys.x0 = np.array(
        [
            50.0,
            0.0,
            20.0,
            0.0,
            -alpha,
            np.pi,
            u_b,
            0.0,
            u_b * np.tan(alpha),
            0.0,
            0.0,
            0.0,
        ]
    )

    # Absolute (hold) commands: pressed key → port bound, release → 0.
    # W/S thrust, UP/DOWN elevator, LEFT/RIGHT aileron, A/D rudder. ESC quits.
    u = sys.inputs["u"]
    u.lower_bound[:] = [0.0, -0.12, -0.12, -0.12]
    u.upper_bound[:] = [T_max, 0.12, 0.12, 0.12]
    u.set_nominal_value(np.array([T_trim, 0.0, 0.0, 0.0]))

    sys.compute_trajectory()

    sys.camera_plot_axes = (0, 1)
    sys.animate()

    RealtimeSimulator(
        sys,
        frame_dt=1 / 30,
        renderer="meshcat",
        is_3d=True,
        input=PygameInput(
            mode="hold",
            key_axes=[
                ("w", "s"),
                ("up", "down"),
                ("right", "left"),
                ("d", "a"),
            ],
        ),
    ).run()
