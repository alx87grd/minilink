"""Planar boat: surge, sway and yaw with surge/sway force inputs, with or without a current."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)
from minilink.graphical.animation.primitives import Arrow, CustomLine, TorqueArrow
from minilink.graphical.catalog.boat_skin import boat_skin_2d

###############################################################################
# Public API
###############################################################################


class Boat2D(GeneralizedMechanicalSystem):
    """Three-DoF planar boat with surge/sway force inputs.

    Low-speed manoeuvring model with linear plus quadratic hydrodynamic
    damping, following Fossen, *Handbook of Marine Craft Hydrodynamics and
    Motion Control* (2nd ed.), section 6.7."""

    def __init__(self):
        super().__init__(dof=3, pos=3, actuators=2)
        self.name = "Planar Boat"

        # Parameters: the hull's areas and length scale with the thruster lever arm
        l_t = 3.0
        self.params = {
            "mass": 10000.0,
            "inertia": 10000.0,
            "l_t": l_t,
            "damping_coef": np.array([10.0, 10.0, 100.0]),
            "Cx_max": 0.5,
            "Cy_max": 0.6,
            "Cm_max": 0.01,
            "N_max": 0.1,
            "rho": 1000.0,
            "Alc": 2.0 * l_t,
            "Afc": 0.5 * l_t,
            "loa": 2.0 * l_t,
        }

        # Signal labels and units
        self.state.labels = ["x", "y", "theta", "surge", "sway", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["Tx", "Ty"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the equations of motion): the hull's size, a camera
        # that follows it, and the top-view skin (``boat_skin_3d`` is the hull on the
        # water)
        self.body_length = 2.0 * l_t
        self.body_width = self.params["Afc"]
        self.camera_scale = 3.0 * self.params["loa"]
        self.camera_follow_frame = "body"
        self.show_hydrodynamic_forces = False
        self.skin = boat_skin_2d

    def M(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        inertia = params["inertia"]

        # Rigid-body inertia M = diag(m, m, I_z): equal surge and sway mass
        M = xp.diag(xp.array([mass, mass, inertia]))

        return M

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        xp = array_module(q, v)

        mass = params["mass"]

        yaw_rate = v[2]

        # Coriolis coupling of the body-frame yaw rate r: C = [0 −m r 0; m r 0 0; 0 0 0]
        # fmt: off
        C = xp.array([
            [            0.0, -mass * yaw_rate, 0.0],
            [mass * yaw_rate,              0.0, 0.0],
            [            0.0,              0.0, 0.0],
        ])
        # fmt: on

        return C

    def N(self, q, params=None):
        xp = array_module(q)

        theta = q[2]
        c = xp.cos(theta)
        s = xp.sin(theta)

        # The body-to-world rotation R(θ), mapping body velocities to world rates
        # fmt: off
        N = xp.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        # fmt: on

        return N

    def B(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l_t = params["l_t"]

        # The thrust acts l_t aft of the c.g., adding the yaw moment −l_t T_y
        # fmt: off
        B = xp.array([
            [1.0,  0.0],
            [0.0,  1.0],
            [0.0, -l_t],
        ])
        # fmt: on

        return B

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params

        # In still water the flow sees the body velocity itself
        d = self.damping(v, params)

        return d

    def damping(self, relative_velocity, params=None):
        """Hydrodynamic damping of a water-relative body velocity ``[u_r, v_r, r]``."""
        params = self.params if params is None else params
        xp = array_module(relative_velocity)

        damping_coef = params["damping_coef"]
        rho = params["rho"]
        Afc = params["Afc"]
        Alc = params["Alc"]
        loa = params["loa"]
        N_max = params["N_max"]

        # The relative flow: its squared speed V² and attack angle α
        vr = relative_velocity
        speed_squared = vr[0] ** 2 + vr[1] ** 2
        alpha = -xp.arctan2(vr[1], vr[0])
        Cx, Cy, Cm = self.current_coefficients(alpha, params)

        # Linear damping D ν_r, plus the quadratic current loads (Fossen 6.7):
        # X = −½ ρ A_FC C_X V², Y = −½ ρ A_LC C_Y V², and
        # N = −½ ρ A_LC L_oa C_N V² + N_max ρ A_LC L_oa |r| r
        d_linear = damping_coef * vr
        fx = -0.5 * rho * Afc * Cx * speed_squared
        fy = -0.5 * rho * Alc * Cy * speed_squared
        mz = -0.5 * rho * Alc * loa * Cm * speed_squared
        mz += N_max * rho * Alc * loa * xp.abs(vr[2]) * vr[2]
        d = d_linear + xp.array([fx, fy, mz])

        return d

    def current_coefficients(self, alpha, params=None):
        """The current coefficients ``(Cx, Cy, Cm)`` at the attack angle ``alpha``."""
        params = self.params if params is None else params
        xp = array_module(alpha)

        Cx_max = params["Cx_max"]
        Cy_max = params["Cy_max"]
        Cm_max = params["Cm_max"]

        # Quadratic hydrodynamic coefficients against the attack angle (Fossen
        # Fig. 6.11): C_X = −C_X,max cos α |cos α|, C_Y = C_Y,max sin α |sin α|,
        # C_N = C_N,max sin 2α
        Cx = -Cx_max * xp.cos(alpha) * xp.abs(xp.cos(alpha))
        Cy = Cy_max * xp.sin(alpha) * xp.abs(xp.sin(alpha))
        Cm = Cm_max * xp.sin(2.0 * alpha)

        return Cx, Cy, Cm

    def body_shape(self):
        """Top-view hull silhouette with the c.g. at the local origin.

        The hull spans ``body_length`` along local +X with a pointed bow and is
        centred so the body-frame origin (c.g.) sits at mid-hull.
        """
        l = self.body_length
        w = self.body_width

        pts = np.array(
            [
                [-0.5 * l, -0.5 * w, 0.0],
                [0.35 * l, -0.5 * w, 0.0],
                [0.5 * l, 0.0, 0.0],
                [0.35 * l, 0.5 * w, 0.0],
                [-0.5 * l, 0.5 * w, 0.0],
                [-0.5 * l, -0.5 * w, 0.0],
            ]
        )

        return CustomLine(pts, color="blue", linewidth=2)

    def tf(self, x, u, t=0, params=None):
        q = x[:3]

        # The yaw-moment arrow turns from the body's lateral axis: θ − π/2
        theta_torque = q[2] - np.pi / 2.0

        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
            "hydrotorque": SE2(q[0], q[1], theta_torque),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        # Arrow lengths: 2·10⁻⁴ m per newton
        force_scale = 0.0002

        # The thrust arrow along the thrust, ending at the thruster l_t aft of the c.g.
        thrust_len = force_scale * np.hypot(u[0], u[1])
        a = np.arctan2(u[1], u[0])
        thrust_dir = np.array([np.cos(a), np.sin(a)])
        l_t = params["l_t"]
        thrust = Arrow(
            base=-thrust_len * thrust_dir,
            vector=thrust_dir,
            scale=thrust_len,
            color="red",
            linewidth=2,
        )
        thrust.local_transform = translation(-l_t, 0.0, 0.0)
        dynamic = {"body": [thrust]}

        # On request, the hydrodynamic force at the c.g. and its yaw moment
        if self.show_hydrodynamic_forces:
            q = x[:3]
            rho = params["rho"]
            Alc = params["Alc"]
            loa = params["loa"]
            Cm_max = params["Cm_max"]

            # The force the water applies: −d, since d sits on the left side
            d = self.d(q, x[3:], u, t, params)
            hydro_force = -d

            # A full turn of the torque arrow is the current moment at V² = 12 m²/s²,
            # ½ ρ A_LC L_oa C_N,max 12
            torque_max = abs(0.5 * rho * Alc * loa * Cm_max * 12.0)
            hf_len = force_scale * np.hypot(hydro_force[0], hydro_force[1])
            hf_a = np.arctan2(hydro_force[1], hydro_force[0])
            hf_d = np.array([np.cos(hf_a), np.sin(hf_a)])
            dynamic["body"].append(
                Arrow(
                    base=(0.0, 0.0),
                    vector=hf_d,
                    scale=hf_len,
                    color="black",
                    linewidth=2,
                    style="--",
                )
            )
            dynamic["hydrotorque"] = [
                TorqueArrow(
                    sweep=hydro_force[2] * (2.0 * np.pi) / torque_max,
                    radius=loa / 5.0,
                    head_ratio=0.4,
                    color="black",
                    linewidth=2,
                    style="--",
                )
            ]

        return dynamic


class Boat2DWithCurrent(Boat2D):
    """Planar boat with constant current velocity in world frame."""

    def __init__(self):
        super().__init__()
        self.name = "Planar Boat With Current"

        # Parameters: the current's velocity in the world frame
        self.params["current_velocity"] = np.array([-1.0, -1.0])

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q, v)

        current_velocity = params["current_velocity"]

        N = self.N(q, params)

        # The current in body axes: ν_c = Nᵀ [V_x, V_y, 0]
        world_current = xp.array([current_velocity[0], current_velocity[1], 0.0])
        body_current = N.T @ world_current

        # The damping acts on the water-relative velocity ν_r = ν − ν_c
        relative_velocity = v - body_current
        d = self.damping(relative_velocity, params)

        return d

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        frames = super().tf(x, u, t, params)
        loa = params["loa"]

        # The current arrow, one hull length up and to the left of the boat
        frames["current"] = translation(x[0] - loa, x[1] + loa, 0.0)

        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        dynamic = super().get_dynamic_geometry(x, u, t, params)
        current_velocity = params["current_velocity"]
        loa = params["loa"]

        # The current arrow along the current, half a hull length per m/s
        length = 0.5 * loa * np.hypot(current_velocity[0], current_velocity[1])
        a = np.arctan2(current_velocity[1], current_velocity[0])
        current_dir = np.array([np.cos(a), np.sin(a)])
        dynamic["current"] = [
            Arrow(
                base=-length * current_dir,
                vector=current_dir,
                scale=length,
                color="green",
                linewidth=2,
            )
        ]

        return dynamic


if __name__ == "__main__":
    sys = Boat2D()
    sys = Boat2DWithCurrent()
    sys.show_hydrodynamic_forces = True

    sys.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    sys.compute_forced(
        lambda t: np.array([10000.0, 5000.0 * np.sin(0.5 * t)]),
        tf=10.0,
    )
    sys.animate()
