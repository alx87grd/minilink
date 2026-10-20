"""Aircraft: the planar plane with an elevator, and the six-DoF plane with elevator, aileron and rudder."""

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

###############################################################################
# Public API
###############################################################################


class Plane2D(MechanicalSystem):
    """Planar aircraft with thrust and elevator inputs."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Aircraft"

        # Parameters
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

        # Signal labels and units
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["thrust", "delta"]
        self.inputs["u"].units = ["N", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the fuselage length, its c.g. at 60 % of it from the tail, and a
        # camera that follows the body
        self.length = 2.0
        self.l_cg = 0.6 * self.length
        self.width = self.length / 10.0
        self.dynamic_range = self.length
        self.camera_scale = self.dynamic_range
        self.camera_follow_frame = "body"

    def velocity_vector(self, q, dq):
        """Airspeed ``V``, flight-path angle ``γ`` and angle of attack ``α = θ − γ``."""
        xp = array_module(dq)

        theta = q[2]
        vx = dq[0]
        vy = dq[1]

        # V = √(ẋ² + ẏ²), γ = atan2(ẏ, ẋ), α = θ − γ
        speed = xp.sqrt(vx**2 + vy**2)
        gamma = xp.arctan2(vy, vx)
        alpha = theta - gamma

        return speed, gamma, alpha

    def Cl(self, alpha, params=None):
        """Lift coefficient: ``sin 2α``, plus ``Cl_alpha α`` below stall."""
        params = self.params if params is None else params
        xp = array_module(alpha)

        alpha_stall = params["alpha_stall"]
        Cl_alpha = params["Cl_alpha"]

        # C_L = sin 2α + C_Lα α while |α| < α_stall, and sin 2α past the stall
        cl = xp.sin(2.0 * alpha)
        cl = xp.where(xp.abs(alpha) < alpha_stall, cl + Cl_alpha * alpha, cl)

        return cl

    def Cd(self, alpha, params=None):
        """Drag coefficient: the parasitic drag, plus the induced drag below stall."""
        params = self.params if params is None else params
        xp = array_module(alpha)

        alpha_stall = params["alpha_stall"]
        Cd0 = params["Cd0"]
        e = params["e_factor"]
        AR = params["AR"]

        Cl = self.Cl(alpha, params)

        # C_D = C_D0 + (1 − cos 2α) + C_L² / (π e AR) while |α| < α_stall,
        # and without the induced term C_L² / (π e AR) past the stall
        cd = Cd0 + (1.0 - xp.cos(2.0 * alpha))
        cd = xp.where(xp.abs(alpha) < alpha_stall, cd + Cl**2 / (xp.pi * e * AR), cd)

        return cd

    def Cm(self, alpha, params=None):
        """Pitch moment coefficient: the constant ``Cm0``, at every angle of attack."""
        params = self.params if params is None else params

        Cm0 = params["Cm0"]

        return Cm0

    def aerodynamic_forces(self, speed, alpha, delta, params=None):
        """Lift, drag and moment of the wing, and of the tail deflected by ``delta``."""
        params = self.params if params is None else params
        xp = array_module(speed)

        rho = params["rho"]
        S_w = params["S_w"]
        S_t = params["S_t"]
        AR = params["AR"]

        # The wing at α, the tail at α + δ
        Cl_w = self.Cl(alpha, params)
        Cd_w = self.Cd(alpha, params)
        Cm_w = self.Cm(alpha, params)
        Cl_t = self.Cl(alpha + delta, params)
        Cd_t = self.Cd(alpha + delta, params)
        Cm_t = self.Cm(alpha + delta, params)

        # Dynamic pressure q̄ = ½ ρ V², and each chord c = √(S / AR)
        q_dyn = 0.5 * rho * speed**2
        chord_w = xp.sqrt(S_w / AR)
        chord_t = xp.sqrt(S_t / AR)

        # L = q̄ S C_L, D = q̄ S C_D, M = q̄ S c C_M, on each surface
        L_w = q_dyn * S_w * Cl_w
        D_w = q_dyn * S_w * Cd_w
        M_w = q_dyn * S_w * chord_w * Cm_w
        L_t = q_dyn * S_t * Cl_t
        D_t = q_dyn * S_t * Cd_t
        M_t = q_dyn * S_t * chord_t * Cm_t

        return L_w, D_w, M_w, L_t, D_t, M_t

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        inertia = params["inertia"]

        H = xp.diag(xp.array([mass, mass, inertia]))

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # A constant inertia matrix: no Coriolis or centrifugal terms
        C = xp.zeros((3, 3))

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        gravity = params["gravity"]

        # The weight along +y (g sits on the left side of the equation of motion)
        g = xp.array([0.0, mass * gravity, 0.0])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l_w = params["l_w"]
        l_t = params["l_t"]

        # Without u, the control surfaces read the input port's nominal value
        u = self.get_u_from_input_ports() if u is None else u
        speed, gamma, alpha = self.velocity_vector(q, dq)
        delta = u[1]
        L_w, D_w, M_w, L_t, D_t, M_t = self.aerodynamic_forces(
            speed, alpha, delta, params
        )
        c_alpha = xp.cos(alpha)
        s_alpha = xp.sin(alpha)

        # The wind-axis load [−D, L, M]: the drag against the velocity, the lift across
        # it, and the surfaces' moments plus their lift and drag about the c.g.
        lift = L_w + L_t
        drag = D_w + D_t
        moment = (
            M_w
            + M_t
            - l_w * (L_w * c_alpha + D_w * s_alpha)
            - l_t * (L_t * c_alpha + D_t * s_alpha)
        )
        wind_load = xp.array([-drag, lift, moment])

        # The wind axes turned by the flight-path angle γ into the world axes
        c_gamma = xp.cos(gamma)
        s_gamma = xp.sin(gamma)
        # fmt: off
        R = xp.array([
            [c_gamma, -s_gamma, 0.0],
            [s_gamma,  c_gamma, 0.0],
            [    0.0,      0.0, 1.0],
        ])
        # fmt: on

        # d = −R(γ) [−D, L, M]ᵀ, on the left side of the equation of motion
        tau_d = -(R @ wind_load)

        return tau_d

    def generalized_force(self, q, dq, u, t=0.0, params=None):
        xp = array_module(q)

        thrust = u[0]
        theta = q[2]

        # The thrust along the body axis: τ = T [cos θ, sin θ, 0]
        tau = thrust * xp.array([xp.cos(theta), xp.sin(theta), 0.0])

        return tau

    def get_kinematic_geometry(self):
        # The primitives on fixed frames; the tail chord and the force arrows come
        # from get_dynamic_geometry, which keeps their draw order
        return {
            "body": [self.body_shape()],
            "center": [Point(color="black", marker="o", size=5)],
            "wingchord": [self.chord_line()],
            "world": [ground_line(length=200.0, y=0.0, color="black", style="--")],
        }

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        S_w = params["S_w"]
        AR = params["AR"]
        S_t = params["S_t"]
        l_w = params["l_w"]
        l_t = params["l_t"]

        q = x[:3]
        theta = q[2]
        delta = u[1]
        chord_w = xp.sqrt(S_w / AR)
        chord_t = xp.sqrt(S_t / AR)
        c = xp.cos(theta)
        s = xp.sin(theta)

        # The wing and tail stations on the body axis
        wing = xp.array([q[0] - l_w * c, q[1] - l_w * s])
        tail = xp.array([q[0] - l_t * c, q[1] - l_t * s])

        # Each chord along its surface: the wing's on the body axis, the tail's
        # deflected by δ
        dir_w = xp.array([c, s])
        dir_t = xp.array([xp.cos(theta + delta), xp.sin(theta + delta)])
        wing_aft = wing - chord_w * dir_w
        wing_fore = wing + chord_w * dir_w
        tail_aft = tail - chord_t * dir_t
        tail_fore = tail + chord_t * dir_t

        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
            "wingchord": segment_pose_2d(wing_aft, wing_fore),
            "tailchord": segment_pose_2d(tail_aft, tail_fore),
            "speed": translation(q[0], q[1], 0.0),
            "wingpt": translation(wing[0], wing[1], 0.0),
            "tailpt": translation(tail[0], tail[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        length = self.length
        l_cg = self.l_cg

        q = x[:3]
        dq = x[3:]
        speed, gamma, alpha = self.velocity_vector(q, dq)
        delta = u[1]
        L_w, D_w, _, L_t, D_t, _ = self.aerodynamic_forces(speed, alpha, delta, params)

        # Arrow lengths: a tenth of the body length per newton, and the speed capped
        # at the body length
        force_scale = length / 10.0
        thrust_len = force_scale * u[0]
        speed_len = min(speed * length / 30.0, length)

        # The lift across the velocity and the drag against it, at each surface
        cg = np.cos(gamma)
        sg = np.sin(gamma)
        lift_w = (-L_w * sg, L_w * cg)
        drag_w = (-D_w * cg, -D_w * sg)
        lift_t = (-L_t * sg, L_t * cg)
        drag_t = (-D_t * cg, -D_t * sg)

        # The thrust arrow ends at the tail, along the body axis
        thrust = Arrow(
            base=(-thrust_len, 0.0),
            vector=(1.0, 0.0),
            scale=thrust_len,
            color="red",
            linewidth=2,
        )
        thrust.local_transform = translation(-l_cg, 0.0, 0.0)

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
                    vector=lift_w,
                    scale=force_scale,
                    color="blue",
                    linewidth=2,
                ),
                Arrow(
                    base=(0.0, 0.0),
                    vector=drag_w,
                    scale=force_scale,
                    color="red",
                    linewidth=2,
                ),
            ],
            "tailpt": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=lift_t,
                    scale=force_scale,
                    color="blue",
                    linewidth=2,
                ),
                Arrow(
                    base=(0.0, 0.0),
                    vector=drag_t,
                    scale=force_scale,
                    color="red",
                    linewidth=2,
                ),
            ],
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

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

        # Parameters
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

        # Signal labels and units
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

        # Graphics: the airframe's size and span, a camera that follows the body,
        # and the 3-D skin
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
        """Lift coefficient: ``sin 2α``, plus ``Cl_alpha α`` below stall."""
        params = self.params if params is None else params
        xp = array_module(alpha)

        alpha_stall = params["alpha_stall"]
        Cl_alpha = params["Cl_alpha"]

        # C_L = sin 2α + C_Lα α while |α| < α_stall, and sin 2α past the stall
        cl = xp.sin(2.0 * alpha)
        cl = xp.where(xp.abs(alpha) < alpha_stall, cl + Cl_alpha * alpha, cl)

        return cl

    def Cd(self, alpha, params=None):
        """Drag coefficient: the parasitic drag, plus the induced drag below stall."""
        params = self.params if params is None else params
        xp = array_module(alpha)

        alpha_stall = params["alpha_stall"]
        Cd0 = params["Cd0"]
        e = params["e_factor"]
        AR = params["AR"]

        Cl = self.Cl(alpha, params)

        # C_D = C_D0 + (1 − cos 2α) + C_L² / (π e AR) while |α| < α_stall,
        # and without the induced term C_L² / (π e AR) past the stall
        cd = Cd0 + (1.0 - xp.cos(2.0 * alpha))
        cd = xp.where(xp.abs(alpha) < alpha_stall, cd + Cl**2 / (xp.pi * e * AR), cd)

        return cd

    def Cm(self, alpha, params=None):
        """Pitch moment coefficient: the constant ``Cm0``, at every angle of attack."""
        params = self.params if params is None else params

        Cm0 = params["Cm0"]

        return Cm0

    def R_wb(self, q):
        """Body-to-world rotation for configuration ``q``."""
        xp = array_module(q)

        phi = q[3]
        theta = q[4]
        psi = q[5]

        # R_wb = Rz(ψ) Ry(θ) Rx(φ) Rx(π): the yaw, the pitch, the roll, then the flip
        # from the body's z-down convention
        R = Rz(psi) @ Ry(theta) @ Rx(phi) @ Rx(xp.pi)

        return R

    def velocity_angles(self, v):
        """Airspeed, angle of attack, and sideslip from body velocity."""
        xp = array_module(v)

        u = v[0]
        vb = v[1]
        w = v[2]

        # V = √(u² + v² + w²), α = atan2(w, u), β = atan2(v, √(u² + w²))
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

        # Dynamic pressure q̄ = ½ ρ V², and each chord c = √(S / AR)
        q_dyn = 0.5 * rho * speed**2
        chord_w = xp.sqrt(S_w / AR)
        chord_t = xp.sqrt(S_t / AR)

        # Wing and horizontal tail (longitudinal), with the planar aircraft's C_L, C_D
        # and C_M: L = q̄ S C_L, D = q̄ S C_D, M = q̄ S c C_M, the tail at α + δ_e
        Cl_w = self.Cl(alpha, params)
        Cd_w = self.Cd(alpha, params)
        Cm_w = self.Cm(alpha, params)
        Cl_t = self.Cl(alpha + delta_e, params)
        Cd_t = self.Cd(alpha + delta_e, params)
        Cm_t = self.Cm(alpha + delta_e, params)
        L_w = q_dyn * S_w * Cl_w
        D_w = q_dyn * S_w * Cd_w
        M_w = q_dyn * S_w * chord_w * Cm_w
        L_t = q_dyn * S_t * Cl_t
        D_t = q_dyn * S_t * Cd_t
        M_t = q_dyn * S_t * chord_t * Cm_t

        # The wind-axis load [−D, 0, −L] turned by α about the body y axis
        c_a = xp.cos(alpha)
        s_a = xp.sin(alpha)
        F_w = xp.array([-c_a * D_w - s_a * L_w, 0.0, s_a * D_w - c_a * L_w])
        F_t = xp.array([-c_a * D_t - s_a * L_t, 0.0, s_a * D_t - c_a * L_t])

        # Vertical fin and rudder (lateral), at β + δ_r: C_L and C_D only, no C_M0
        # (it trims the pitch)
        Cl_v = self.Cl(beta + delta_r, params)
        Cd_v = self.Cd(beta + delta_r, params)
        L_v = q_dyn * S_v * Cl_v
        D_v = q_dyn * S_v * Cd_v
        c_b = xp.cos(beta)
        s_b = xp.sin(beta)
        F_v = xp.array([-c_b * D_v + s_b * L_v, -s_b * D_v - c_b * L_v, 0.0])

        # F = F_w + F_t + F_v
        force = F_w + F_t + F_v

        # Moments: each surface's C_M plus r × F at its station on the body −x axis,
        # and the aileron's pure roll moment q̄ S_w b_w C_lδa δ_a
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

        M = xp.diag(xp.array([mass, mass, mass, Ixx, Iyy, Izz]))

        return M

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        mass = params["mass"]
        Ixx = params["inertia_xx"]
        Iyy = params["inertia_yy"]
        Izz = params["inertia_zz"]

        p = v[3]
        q_rate = v[4]
        r = v[5]

        # Rigid-body Coriolis in body axes: m ω × v_lin and ω × (I ω)
        # fmt: off
        C = xp.array([
            [           0.0, -mass * r, mass * q_rate, 0.0, 0.0, 0.0],
            [      mass * r,       0.0,     -mass * p, 0.0, 0.0, 0.0],
            [-mass * q_rate,  mass * p,           0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0,          0.0,  Izz * r, -Iyy * q_rate],
            [0.0, 0.0, 0.0,     -Izz * r,      0.0,       Ixx * p],
            [0.0, 0.0, 0.0, Iyy * q_rate, -Ixx * p,           0.0],
        ])
        # fmt: on

        return C

    def N(self, q, params=None):
        xp = array_module(q)

        phi = q[3]
        theta = q[4]
        R = self.R_wb(q)
        s_phi = xp.sin(phi)
        c_phi = xp.cos(phi)
        s_th = xp.sin(theta)
        c_th = xp.cos(theta)
        t_th = s_th / c_th

        # Euler rates from the intermediate (pre-Rx(π)) rates [p, −q, −r]: E_body = E S
        # fmt: off
        E = xp.array([
            [1.0, s_phi * t_th, c_phi * t_th],
            [0.0,        c_phi,       -s_phi],
            [0.0, s_phi / c_th, c_phi / c_th],
        ])
        # fmt: on
        S = xp.diag(xp.array([1.0, -1.0, -1.0]))
        E_body = E @ S

        # q̇ = N v, with N = [[R_wb, 0], [0, E_body]]
        z03 = xp.zeros((3, 3))
        top = xp.concatenate([R, z03], axis=1)
        bottom = xp.concatenate([z03, E_body], axis=1)
        N = xp.concatenate([top, bottom], axis=0)

        return N

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        gravity = params["gravity"]

        R = self.R_wb(q)

        # Left-side potential gradient (the planar aircraft's convention): world +z
        # up, so g_world = [0, 0, m g], then g = [R_wbᵀ g_world, 0] in body axes
        g_world = xp.array([0.0, 0.0, mass * gravity])
        g_body = R.T @ g_world
        g = xp.concatenate([g_body, xp.zeros(3)])

        return g

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        # Without u, the control surfaces read the input port's nominal value
        u = self.get_u_from_input_ports() if u is None else u
        speed, alpha, beta = self.velocity_angles(v)
        delta_e = u[1]
        delta_a = u[2]
        delta_r = u[3]
        force, moment = self.aerodynamic_forces(
            speed, alpha, beta, delta_e, delta_a, delta_r, params
        )

        # d = −[F; M]: the aerodynamic wrench on the left side of the equation of motion
        wrench = xp.concatenate([force, moment])
        tau_d = -wrench

        return tau_d

    def generalized_force(self, q, v, u, t=0.0, params=None):
        xp = array_module(q)

        thrust = u[0]

        # The thrust along the body x axis: τ = [T, 0, 0, 0, 0, 0]
        tau = xp.array([thrust, 0.0, 0.0, 0.0, 0.0, 0.0])

        return tau

    def tf(self, x, u, t=0, params=None):
        q = x[:6]
        p = q[:3]
        R = self.R_wb(q)

        # The pose in NumPy, whatever the state's backend
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

    # sys.compute_trajectory()

    # sys.camera_plot_axes = (0, 1)
    # sys.animate()

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
            joystick_axes=[
                (5, 1.0),
                (4, 1.0),
                (3, 1.0),
                (0, 1.0),
            ],
        ),
    ).run()
