"""
Dynamic bicycle (planar rigid body + tire forces), ported minimally from pyro
``vehicle_dynamic.DynamicBicycle``.

State ``x = [x, y, theta, vx, vy, yaw_rate]``: world pose and body velocities
(surge, sway, yaw rate). Inputs are two named ports ``w_rear`` (rear wheel spin
rate [rad/s]) and ``delta`` (steer angle [rad]) so diagrams can wire each
command independently.

:class:`DynamicBicycleCar3D` subclasses this model with identical dynamics and richer 3D graphics.
JAX-traceable plants live in
:mod:`~minilink.dynamics.catalog.vehicles.jax_vehicles`.
"""

from functools import partial

import numpy as np

from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
)
from minilink.graphical.catalog.skins import car_skin_2d, car_skin_3d

# Tire model — pure functions; coefficients live in the plant ``params``.


def tire_slip(vx, vy, w, R, v_min_epsilon):
    """Slip angle ``alpha`` [rad] and slip ratio ``kappa`` [-] at one contact."""
    vx_adj = abs(vx) + v_min_epsilon
    alpha = -np.arctan(vy / vx_adj)
    kappa = (w * R - vx) / vx_adj
    return alpha, kappa


def linear_tire_forces(vx, vy, w, R, Fz, Ca, Ck, mu, v_min_epsilon):
    """Linear-slip tire forces with friction-circle saturation (pyro ``LinearTire``).

    ``Fx = Ck kappa`` and ``Fy = Ca alpha``, scaled back onto ``|F| <= mu Fz``.
    """
    alpha, kappa = tire_slip(vx, vy, w, R, v_min_epsilon)
    Fx = Ck * kappa
    Fy = Ca * alpha
    F_max = mu * Fz
    F_total = np.sqrt(Fx**2 + Fy**2)
    if F_total > F_max:
        ratio = F_max / F_total
        Fx *= ratio
        Fy *= ratio
    return Fx, Fy


def _wheel_rectangle_pts(wl, ww):
    """Closed polyline in wheel frame (x forward, y lateral)."""
    h, w = wl / 2, ww / 2
    return np.array(
        [
            [h, w, 0.0],
            [h, -w, 0.0],
            [-h, -w, 0.0],
            [-h, w, 0.0],
            [h, w, 0.0],
        ]
    )


class DynamicBicycle(DynamicSystem):
    """
    Dynamic bicycle with rear wheel speed and steer inputs.

    Inputs
    ------
    w_rear : rear wheel angular rate [rad/s]
    delta  : front steer angle [rad]
    """

    def __init__(self):
        super().__init__(n=6)

        self.name = "Dynamic Bicycle"

        self.state.labels = ["x", "y", "theta", "vx", "vy", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]

        self.add_input_port(
            "w_rear", nominal_value=0.0, labels=["w_rear"], units=["rad/s"]
        )
        self.add_input_port("delta", nominal_value=0.0, labels=["delta"], units=["rad"])

        self.add_output_port("y", dim=6, function=self.h, dependencies=())

        # EoM parameters: CG-to-axle distances a/b [m], wheel radii r_f/r_r [m],
        # mass [kg], yaw inertia [kg m^2], gravity [m/s^2], air density rho
        # [kg/m^3], drag area CdA = Cd * A [m^2], and linear-slip tire
        # coefficients shared by both axles: cornering stiffness Ca [N/rad],
        # longitudinal stiffness Ck [N], friction mu [-], and slip
        # regularization speed v_min_epsilon [m/s] (same keys as the JAX
        # vehicle ladder).
        self.params = {
            "a": 1.0,
            "b": 1.0,
            "r_f": 0.3,
            "r_r": 0.3,
            "mass": 1500.0,
            "inertia": 2500.0,
            "gravity": 9.81,
            "rho": 1.225,
            "CdA": 0.3 * 2.2,
            "Ca": 60000.0,
            "Ck": 100000.0,
            "mu": 1.0,
            "v_min_epsilon": 0.1,
        }

        # Graphics-only attributes (2-D centerline look)
        self.wheel_len = 0.76
        self.wheel_width = 0.27

        # Graphics-only attributes for the 3-D four-wheel look (read by
        # ``car_skin_3d`` / ``tf``). They live on the base plant so the 3-D
        # look is just ``skin = car_skin_3d`` — no bespoke subclass needed. They
        # do not affect the default 2-D centerline skin.
        self.track = 1.92
        self.body_height = 0.22
        self.body_width_ratio = 0.72
        self.body_length_overhang = 0.26
        self.body_ground_clearance = 0.003
        self.ground_plane_size = 120.0
        self._visual_wheel_width = 0.2
        self._visual_tire_radius_ratio = 0.58

        # Default 2-D skin (black centerline chassis) and a camera that tracks
        # the body frame.
        self.skin = partial(car_skin_2d, color="#1a1a1a")
        self.camera_follow_frame = "body"

    def M(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        return np.diag(np.array([mass, mass, inertia], dtype=float))

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        mass = params["mass"]

        w = v[2]
        C = np.zeros((3, 3), dtype=float)
        C[1, 0] = mass * w
        C[0, 1] = -mass * w
        return C

    def N(self, q, params=None):
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)

        # World-frame velocity kinematics: dq = N(q) v
        # fmt: off
        return np.array(
            [
                [c, -s, 0.0],
                [s,  c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        # fmt: on

    def compute_wheel_velocities(self, v_body, u_inputs, params=None):
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]

        vx = v_body[0]
        vy = v_body[1]
        r = v_body[2]
        delta = u_inputs[1]

        # Contact-point velocities in the body frame, then in each wheel frame.
        vx_f_b = vx
        vy_f_b = vy + a * r
        vx_r_b = vx
        vy_r_b = vy - b * r

        c_d, s_d = np.cos(delta), np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        # Rear wheel rate is the input; the front wheel rolls freely.
        w_r = u_inputs[0]
        w_f = vx_f / r_f

        return vx_f, vy_f, w_f, vx_r, vy_r, w_r

    def compute_tire_physics(self, v_body, u_inputs, params=None):
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]
        r_r = params["r_r"]
        mass = params["mass"]
        gravity = params["gravity"]
        Ca = params["Ca"]
        Ck = params["Ck"]
        mu = params["mu"]
        v_min_epsilon = params["v_min_epsilon"]
        L = a + b

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs, params
        )

        # Static normal-load split between front and rear axles.
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        Fx_f, Fy_f = linear_tire_forces(
            vx_f, vy_f, w_f, r_f, Fz_f, Ca, Ck, mu, v_min_epsilon
        )
        Fx_r, Fy_r = linear_tire_forces(
            vx_r, vy_r, w_r, r_r, Fz_r, Ca, Ck, mu, v_min_epsilon
        )
        return Fx_f, Fy_f, Fx_r, Fy_r

    def generalized_d(self, q, v, u_in, params=None):
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        rho = params["rho"]
        CdA = params["CdA"]

        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v, u_in, params)
        delta = u_in[1]
        c_d, s_d = np.cos(delta), np.sin(delta)

        # Tire forces rotated into the body frame, summed with aero drag.
        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d
        Fx_r_b = Fx_r
        Fy_r_b = Fy_r
        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = a * Fy_f_b - b * Fy_r_b
        F_aero = 0.5 * rho * CdA * v[0] * abs(v[0])
        Sum_Fx -= F_aero
        F_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz], dtype=float)
        return -F_ext

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        u_in = np.array([w_rear[0], delta[0]])

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Rigid-body EoM in body frame: M dv + C v + d = 0; dq = N v
        dv = np.linalg.solve(M, -C @ v - d)
        dq = N @ v
        return np.concatenate([dq, dv])

    def h(self, x, u, t=0.0, params=None):
        return x.copy()

    def _u_in(self, x, u):
        """``[w_rear, delta]`` port values (overridden by the rate variant)."""
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        xp = np.asarray
        return xp([w_rear[0], delta[0]])

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        a = params["a"]
        r_f = params["r_f"]
        tr = self.track

        delta = self._u_in(x, u)[1]
        T_wb = SE2(x[0], x[1], x[2])
        R_steer = SE2(0.0, 0.0, delta)
        return {
            # body frame: 2-D chassis, 3-D body box, rear wheels, and contact arrows.
            "body": T_wb,
            # 2-D front axle (steered); rear axle wheels ride on ``body`` via
            # ``local_transform`` in the skin.
            "axle_front": T_wb @ SE2(a, 0.0, delta),
            # 3-D steered front wheels (lift + steer still need per-frame frames).
            "wheel_fl": T_wb @ translation(a, 0.5 * tr, r_f) @ R_steer,
            "wheel_fr": T_wb @ translation(a, -0.5 * tr, r_f) @ R_steer,
        }

    def _contact_fields(self, x, u):
        """Body-frame per-axle velocities and tire forces for the arrow geometry.

        Returns ``(v_r_loc, v_f_loc, F_r_loc, F_f_loc)`` — the rear/front contact
        velocity and tire force in the body frame. The ``body`` ``tf`` rotates them
        to world, so no manual ``cos``/``sin`` here.
        """
        params = self.params
        a = params["a"]
        b = params["b"]
        vb = x[3:6]
        u_in = self._u_in(x, u)
        delta = u_in[1]

        uu, vv, wr = vb[0], vb[1], vb[2]
        v_f_loc = np.array([uu, vv + a * wr])
        v_r_loc = np.array([uu, vv - b * wr])

        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(vb, u_in)
        cd, sd = np.cos(delta), np.sin(delta)
        # Front tire force rotated steer->body; rear is already body-frame.
        F_f_loc = np.array([Fx_f * cd - Fy_f * sd, Fx_f * sd + Fy_f * cd])
        F_r_loc = np.array([Fx_r, Fy_r])
        return v_r_loc, v_f_loc, F_r_loc, F_f_loc

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """2-D centerline look: one velocity and one force arrow per axle."""
        b = self.params["b"]
        a = self.params["a"]
        v_r_loc, v_f_loc, F_r_loc, F_f_loc = self._contact_fields(x, u)
        return {
            "body": [
                Arrow(
                    base=(-b, 0.0),
                    vector=v_r_loc,
                    scale=0.2,
                    color="blue",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(a, 0.0),
                    vector=v_f_loc,
                    scale=0.2,
                    color="blue",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(-b, 0.0),
                    vector=F_r_loc,
                    scale=0.001,
                    color="red",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(a, 0.0),
                    vector=F_f_loc,
                    scale=0.001,
                    color="red",
                    linewidth=1.25,
                ),
            ]
        }


class DynamicBicycleCar3D(DynamicBicycle):
    """
    Same dynamics as :class:`DynamicBicycle`; overrides only kinematic graphics.

    Body box, four wheel :class:`~minilink.graphical.animation.primitives.Rod` primitives,
    and duplicated velocity/force arrows at each side (bicycle model vectors on L/R).
    """

    def __init__(self):
        super().__init__()
        self.name = "Dynamic Bicycle (3D car)"
        # Visual-only: wide stance, low body, narrow tub between exposed wheels (racecar-like).
        self.track = 1.92
        self.body_height = 0.22
        # Wider tub → less lateral gap between body sides and tires.
        self.body_width_ratio = 0.72
        self.body_length_overhang = 0.26
        # Smaller vertical gap between body underside and tire crown (display).
        self.body_ground_clearance = 0.003
        self.ground_plane_size = 120.0
        # Larger tires (display only; wheel centers still use r_f / r_r).
        self._visual_wheel_width = 0.2
        self._visual_tire_radius_ratio = 0.58

        # This subclass's look is the 3-D skin; dynamic arrows stay a per-class
        # override because four corner force/velocity arrows are not skin-driven.
        self.skin = car_skin_3d

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """3-D look: velocity and force arrows duplicated at all four wheels.

        Dynamic geometry is not a skin concern, so the four-wheel arrow set is a
        manual override here rather than something driven by ``car_skin_3d``. The
        arrows share the ``body`` frame (``T_wb``); each bakes its wheel
        corner as its ``base`` and lifts to the hub height via ``local_transform``.
        """
        a = self.params["a"]
        b = self.params["b"]
        r_f = self.params["r_f"]
        r_r = self.params["r_r"]
        tr = self.track
        v_r_loc, v_f_loc, F_r_loc, F_f_loc = self._contact_fields(x, u)

        def _arrow(vec, scale, color, bx, by, bz):
            arr = Arrow(
                base=(bx, by), vector=vec, scale=scale, color=color, linewidth=2
            )
            arr.local_transform = translation(0.0, 0.0, bz)
            return arr

        return {
            "body": [
                _arrow(v_r_loc, 0.2, "blue", -b, 0.5 * tr, r_r),
                _arrow(v_r_loc, 0.2, "blue", -b, -0.5 * tr, r_r),
                _arrow(v_f_loc, 0.2, "blue", a, 0.5 * tr, r_f),
                _arrow(v_f_loc, 0.2, "blue", a, -0.5 * tr, r_f),
                _arrow(F_r_loc, 0.001, "red", -b, 0.5 * tr, r_r),
                _arrow(F_r_loc, 0.001, "red", -b, -0.5 * tr, r_r),
                _arrow(F_f_loc, 0.001, "red", a, 0.5 * tr, r_f),
                _arrow(F_f_loc, 0.001, "red", a, -0.5 * tr, r_f),
            ]
        }


if __name__ == "__main__":
    sys = DynamicBicycle()
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u = np.array([5.0, 0.1])
    sys.x0 = x
    sys.compute_forced(u=u, tf=10)
    sys.plot_trajectory()
    sys.animate()
