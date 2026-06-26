"""
Dynamic bicycle (planar rigid body + tire forces), ported minimally from pyro
``vehicle_dynamic.DynamicBicycle``.

State ``x = [x, y, theta, vx, vy, yaw_rate]``: world pose and body velocities
(surge, sway, yaw rate). Inputs are two named ports ``w_rear`` (rear wheel spin
rate [rad/s]) and ``delta`` (steer angle [rad]) so diagrams can wire each
command independently.

:class:`DynamicBicycleCar3D` subclasses this model with identical dynamics and richer 3D graphics.
:class:`JaxDynamicBicycle` is a JAX-traceable variant of the same dynamics, suitable for
gradient-based trajectory optimization.
"""

import numpy as np

from minilink.core.backends import array_module, require_jax_numpy
from minilink.core.kinematics import identity_matrix, pose2d_matrix, translation_matrix
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.legacy import (
    bicycle_car3d_world_arrows,
    bicycle_world_arrows,
)
from minilink.graphical.animation.primitives import Box, CustomLine, Plane, Rod
from minilink.graphical.animation.skins import car_skin_3d


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


class LinearTire:
    """Linear slip tire (same structure as pyro ``LinearTire``)."""

    def __init__(self, Ca=60000.0, Ck=100000.0, mu=1.0):
        self.v_min_epsilon = 0.1
        self.Ca = Ca
        self.Ck = Ck
        self.mu = mu

    def vel2slip(self, vx, vy, w, R):
        vx_adj = abs(vx) + self.v_min_epsilon
        alpha = -np.arctan(vy / vx_adj)
        kappa = (w * R - vx) / vx_adj
        return alpha, kappa

    def vel2forces(self, vx, vy, w, R, Fz):
        alpha, kappa = self.vel2slip(vx, vy, w, R)
        Fx = self.Ck * kappa
        Fy = self.Ca * alpha
        F_max = self.mu * Fz
        F_total = np.sqrt(Fx**2 + Fy**2)
        if F_total > F_max:
            ratio = F_max / F_total
            Fx *= ratio
            Fy *= ratio
        return Fx, Fy


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
        # [kg/m^3], and drag area CdA = Cd * A [m^2].
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
        }

        self.tire_model_f = LinearTire()
        self.tire_model_r = LinearTire()

        # Graphics-only attributes
        self.wheel_len = 0.6
        self.wheel_width = 0.2
        self.camera_follow_frame = "body"
        self.camera_scale = 4.0 * (self.params["a"] + self.params["b"])

    def _bicycle_actuators(self, x, u):
        xp = array_module(x)
        if self.n == 8:
            return xp.asarray(x[6:8])
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        return xp.array([w_rear[0], delta[0]])

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
        L = a + b

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs, params
        )

        # Static normal-load split between front and rear axles.
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, r_r, Fz_r)
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

    def get_kinematic_geometry(self):
        a = self.params["a"]
        b = self.params["b"]
        wl, ww = self.wheel_len, self.wheel_width
        chassis = CustomLine(
            np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]]),
            color="black",
            linewidth=2,
        )
        wpts = _wheel_rectangle_pts(wl, ww)
        return {
            "body": [chassis],
            "axle_rear": [CustomLine(wpts, color="black", linewidth=1)],
            "axle_front": [CustomLine(wpts, color="black", linewidth=1)],
        }

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        u_in = self._bicycle_actuators(x, u)
        delta = u_in[1]
        t_body = pose2d_matrix(x[0], x[1], x[2])
        return {
            "world": identity_matrix(x),
            "body": t_body,
            "axle_rear": t_body @ pose2d_matrix(-b, 0.0, 0.0),
            "axle_front": t_body @ pose2d_matrix(a, 0.0, delta),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {"world": bicycle_world_arrows(x, u, t, self)}


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

    def get_kinematic_geometry(self):
        params = self.params
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]
        r_r = params["r_r"]
        tr = self.track
        bl = (a + b) + self.body_length_overhang
        skin = car_skin_3d(
            bl,
            self.body_width_ratio * tr,
            tr,
            body_height=self.body_height,
            ground_size=self.ground_plane_size,
        )
        cx_body = 0.5 * (a - b)
        z_body = r_r + self.body_ground_clearance + 0.5 * self.body_height
        skin["body"][0].local_transform = translation_matrix(cx_body, 0.0, z_body)
        tire_rad = max(0.045, self._visual_tire_radius_ratio * min(r_f, r_r))
        w_len = self._visual_wheel_width
        wheel = Rod(
            length=w_len,
            radius=tire_rad,
            color="#0a0a0a",
            opacity=1.0,
        )
        skin["wheel_rl"] = [wheel]
        skin["wheel_rr"] = [wheel]
        skin["wheel_fl"] = [wheel]
        skin["wheel_fr"] = [wheel]
        return skin

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {"world": bicycle_car3d_world_arrows(x, u, t, self)}

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]
        r_r = params["r_r"]
        u_in = self._bicycle_actuators(x, u)
        delta = u_in[1]
        tr = self.track
        cx_body = 0.5 * (a - b)
        z_body = r_r + self.body_ground_clearance + 0.5 * self.body_height
        t_wb = pose2d_matrix(x[0], x[1], x[2])
        r_steer = pose2d_matrix(0.0, 0.0, delta)
        frames = {
            "world": identity_matrix(x),
            "body": t_wb @ translation_matrix(cx_body, 0.0, z_body),
            "wheel_rl": t_wb @ translation_matrix(-b, 0.5 * tr, r_r),
            "wheel_rr": t_wb @ translation_matrix(-b, -0.5 * tr, r_r),
            "wheel_fl": t_wb @ translation_matrix(a, 0.5 * tr, r_f) @ r_steer,
            "wheel_fr": t_wb @ translation_matrix(a, -0.5 * tr, r_f) @ r_steer,
            "axle_rear": t_wb @ translation_matrix(-b, 0.0, r_r),
            "axle_front": t_wb @ translation_matrix(a, 0.0, r_f) @ r_steer,
        }
        return frames


# === JAX-traceable variant ===================================================
#
# Same equations as :class:`DynamicBicycle`, but written so the dynamics ``f``
# trace through ``jax.numpy``. The branch in :meth:`LinearTire.vel2forces`
# becomes a smooth ``where`` so the model is differentiable end-to-end and
# usable by the JAX trajectory-optimization transcriptions in
# :mod:`minilink.planning.trajectory_optimization`.


class JaxLinearTire:
    """JAX-traceable counterpart of :class:`LinearTire`."""

    def __init__(self, Ca=60000.0, Ck=100000.0, mu=1.0):
        self.v_min_epsilon = 0.1
        self.Ca = Ca
        self.Ck = Ck
        self.mu = mu

    def vel2slip(self, vx, vy, w, R):
        jnp = require_jax_numpy()
        vx_adj = jnp.abs(vx) + self.v_min_epsilon
        alpha = -jnp.arctan(vy / vx_adj)
        kappa = (w * R - vx) / vx_adj
        return alpha, kappa

    def vel2forces(self, vx, vy, w, R, Fz):
        jnp = require_jax_numpy()
        alpha, kappa = self.vel2slip(vx, vy, w, R)
        Fx = self.Ck * kappa
        Fy = self.Ca * alpha
        F_max = self.mu * Fz
        F_total = jnp.sqrt(Fx**2 + Fy**2)
        # Saturate to the friction circle without a Python branch.
        ratio = jnp.where(F_total > F_max, F_max / (F_total + 1e-12), 1.0)
        return Fx * ratio, Fy * ratio


class JaxDynamicBicycle(DynamicBicycle):
    """JAX-traceable :class:`DynamicBicycle`.

    Inherits the geometry and visualization contract from
    :class:`DynamicBicycle` and only overrides the equations of motion so that
    ``f(x, u, t)`` traces through ``jax.numpy``. Useful with trajectory
    optimization run with ``compile_backend="jax"``.
    """

    def __init__(self):
        super().__init__()
        self.name = "JAX Dynamic Bicycle"
        self.tire_model_f = JaxLinearTire()
        self.tire_model_r = JaxLinearTire()

    # --- Equations of motion (JAX) ---

    def M(self, q, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        return jnp.diag(jnp.array([mass, mass, inertia], dtype=float))

    def C(self, q, v, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        mass = params["mass"]

        w = v[2]
        # fmt: off
        return jnp.array(
            [
                [0.0,      -mass * w, 0.0],
                [mass * w,  0.0,      0.0],
                [0.0,       0.0,      0.0],
            ]
        )
        # fmt: on

    def N(self, q, params=None):
        jnp = require_jax_numpy()
        theta = q[2]
        c, s = jnp.cos(theta), jnp.sin(theta)

        # World-frame velocity kinematics: dq = N(q) v
        # fmt: off
        return jnp.array(
            [
                [c, -s, 0.0],
                [s,  c, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        # fmt: on

    def compute_wheel_velocities(self, v_body, u_inputs, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]

        vx = v_body[0]
        vy = v_body[1]
        r = v_body[2]
        delta = u_inputs[1]

        vx_f_b = vx
        vy_f_b = vy + a * r
        vx_r_b = vx
        vy_r_b = vy - b * r

        c_d, s_d = jnp.cos(delta), jnp.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

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
        L = a + b

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs, params
        )
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)
        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, r_r, Fz_r)
        return Fx_f, Fy_f, Fx_r, Fy_r

    def generalized_d(self, q, v, u_in, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        rho = params["rho"]
        CdA = params["CdA"]

        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v, u_in, params)
        delta = u_in[1]
        c_d, s_d = jnp.cos(delta), jnp.sin(delta)
        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d
        Fx_r_b = Fx_r
        Fy_r_b = Fy_r
        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = a * Fy_f_b - b * Fy_r_b
        F_aero = 0.5 * rho * CdA * v[0] * jnp.abs(v[0])
        Sum_Fx = Sum_Fx - F_aero
        F_ext = jnp.array([Sum_Fx, Sum_Fy, Sum_Mz])
        return -F_ext

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        u_in = jnp.array([w_rear[0], delta[0]])

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Rigid-body EoM in body frame: M dv + C v + d = 0; dq = N v
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v
        return jnp.concatenate([dq, dv])

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(x)


class JaxDynamicBicycleRateInputs(JaxDynamicBicycle):
    """JAX-traceable :class:`DynamicBicycle` with rate inputs.

    Inherits the geometry and visualization contract from
    :class:`DynamicBicycle` and only overrides the equations of motion so that
    ``f(x, u, t)`` traces through ``jax.numpy``. Useful with trajectory
    optimization run with ``compile_backend="jax"``.
    """

    def __init__(self):

        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "JAX Dynamic Bicycle (rate inputs)"

        self.n = 8

        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        self.state.labels = [
            "x",
            "y",
            "theta",
            "vx",
            "vy",
            "yaw_rate",
            "w_rear",
            "delta",
        ]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad/s", "rad"]

        self.inputs = {}
        self.add_input_port(
            "w_rear_dot",
            nominal_value=0.0,
            labels=["w_rear_dot"],
            units=["rad/s^2"],
        )
        self.add_input_port(
            "delta_dot",
            nominal_value=0.0,
            labels=["delta_dot"],
            units=["rad/s^2"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        u_in = x[6:8]  # [w_rear, delta]

        # w_rear = x[6]
        # delta = x[7]
        # w_rear_dot = u[0]
        # delta_dot = u[1]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Wheel/steer commands integrate the rate inputs; body follows the EoM.
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        return jnp.concatenate([dq, dv, u])


if __name__ == "__main__":
    sys = JaxDynamicBicycleRateInputs()
    # sys = DynamicBicycleCar3D()
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u = np.array([5.0, 0.1])
    t = 0.0

    sys.x0 = x

    # sys.compute_trajectory(tf=10)
    sys.compute_forced(u=u, tf=10)
    sys.plot_trajectory()
    sys.animate()
