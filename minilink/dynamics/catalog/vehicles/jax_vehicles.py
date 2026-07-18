"""JAX vehicle fidelity ladder (planning / trajopt plants).

Module-scoped names — no ``Jax`` prefix. Default ports are stacked ``u`` with
``y = x``; named-port variants use the ``Ports`` suffix.

Ladder (increasing fidelity)
----------------------------
=======  ==================  ===  =========================================
Class    State dim ``n``     u    Role
=======  ==================  ===  =========================================
Holonomic                    2    ``[vx, vy]``          point, velocity
HolonomicAccel               4    ``[ax, ay]``          point, accel
BicycleKin                   3    ``[v, delta]``        nonholonomic
BicycleAcc                   5    ``[a_x, delta_dot]``  no-slip accel/steer
BicycleDyn                   6    ``[w_rear, delta]``   body + linear tires
BicycleDynRate               8    ``[w_rear_dot, delta_dot]``
BicycleDynTauRate            8    ``[tau_rear, delta_dot]``
BicycleDynServo              9    ``[tau_cmd, delta_cmd]``  lag on τ and δ
BicycleDynEngine             9    ``[P_cmd, delta_cmd]``    power lag + wheel
=======  ==================  ===  =========================================

EoM ``params`` carry only independent quantities that enter ``f``. Graphics axle
offsets ``a`` / ``b`` default to ``length / 2`` as object attributes (not EoM
params). Dynamic plants use a **centered CG** in the equations
(``a = b = length / 2``).
"""

from functools import partial

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import Arrow, Circle
from minilink.graphical.catalog.skins import car_skin_2d

# ---------------------------------------------------------------------------
# Tire helper (pure; no objects on the traced path)
# ---------------------------------------------------------------------------


def linear_tire_forces(vx, vy, w, R, Fz, Ca, Ck, mu, v_min_epsilon):
    """Linear slip tire forces with a friction-circle ``where`` saturate.

    Parameters
    ----------
    vx, vy : wheel-frame contact velocities [m/s]
    w : wheel spin rate [rad/s]
    R : wheel radius [m]
    Fz : normal load [N]
    Ca, Ck, mu, v_min_epsilon : tire / regularization coeffs from ``params``

    Returns
    -------
    Fx, Fy : longitudinal and lateral tire forces [N]
    """
    jnp = require_jax_numpy()
    vx_adj = jnp.abs(vx) + v_min_epsilon
    alpha = -jnp.arctan(vy / vx_adj)
    kappa = (w * R - vx) / vx_adj
    Fx = Ck * kappa
    Fy = Ca * alpha
    F_max = mu * Fz
    F_total = jnp.sqrt(Fx**2 + Fy**2)
    ratio = jnp.where(F_total > F_max, F_max / (F_total + 1e-12), 1.0)
    return Fx * ratio, Fy * ratio


# ---------------------------------------------------------------------------
# Holonomic point plants
# ---------------------------------------------------------------------------


class Holonomic(DynamicSystem):
    """Holonomic point mass with velocity commands.

    State ``x = [x, y]``; input ``u = [vx, vy]``; ``params = {}``.
    """

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Holonomic"
        self.params = {}
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(x)

    def get_kinematic_geometry(self):
        return {
            "body": [
                Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True)
            ]
        }

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "body": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(u[0], u[1]),
                    scale=0.4,
                    color="red",
                    linewidth=2,
                )
            ]
        }


class HolonomicAccel(DynamicSystem):
    """Holonomic point mass with acceleration commands.

    State ``x = [x, y, vx, vy]``; input ``u = [ax, ay]``; ``params = {}``.
    """

    def __init__(self):
        super().__init__(n=4, input_dim=2, output_dim=4, expose_state=True)
        self.name = "HolonomicAccel"
        self.params = {}
        self.state.labels = ["x", "y", "vx", "vy"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["ax", "ay"]
        self.inputs["u"].units = ["m/s^2", "m/s^2"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        # double integrator: position integrates velocity, velocity integrates accel
        return jnp.array([x[2], x[3], u[0], u[1]])

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(x)

    def get_kinematic_geometry(self):
        return {
            "body": [
                Circle(radius=0.25, center=[0.0, 0.0, 0.0], color="blue", fill=True)
            ]
        }

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "body": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(x[2], x[3]),
                    scale=0.4,
                    color="red",
                    linewidth=2,
                )
            ]
        }


# ---------------------------------------------------------------------------
# Kinematic / no-slip bicycle
# ---------------------------------------------------------------------------


def _bicycle_graphics_attrs(plant, length):
    """Shared 2-D centerline graphics defaults for bicycle plants.

    Axle offsets ``a`` / ``b`` are object attributes only (not EoM ``params``).
    """
    plant.a = 0.5 * length
    plant.b = 0.5 * length
    plant.wheel_len = 0.76
    plant.wheel_width = 0.27
    plant.camera_scale = 10.0
    plant.camera_follow_frame = "body"
    plant.skin = partial(car_skin_2d, color="#1a1a1a")


class BicycleKin(DynamicSystem):
    """Kinematic bicycle with speed and steer-angle inputs.

    State ``x = [x, y, theta]``; input ``u = [v, delta]``.
    EoM ``params``: ``{"length": 2.0}``. Graphics ``a``, ``b`` default to
    ``length / 2`` (object attrs, not equation params).
    """

    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3, expose_state=True)
        self.name = "BicycleKin"
        length = 2.0
        self.params = {"length": length}
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["v", "delta"]
        self.inputs["u"].units = ["m/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        _bicycle_graphics_attrs(self, length)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        length = params["length"]
        v = u[0]
        delta = u[1]
        theta = x[2]

        # kinematic bicycle: heading rate = v * tan(delta) / length
        return jnp.array(
            [
                v * jnp.cos(theta),
                v * jnp.sin(theta),
                v * jnp.tan(delta) / length,
            ]
        )

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.asarray(x)

    def tf(self, x, u, t=0, params=None):
        a = self.a
        delta = u[1]
        T_wb = SE2(x[0], x[1], x[2])
        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, delta),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


class BicycleAcc(BicycleKin):
    """No-slip bicycle with longitudinal accel and steer-rate inputs.

    State ``x = [x, y, theta, v, delta]``; input ``u = [a_x, delta_dot]``.

    Equations of motion
    -------------------
    ``ẋ = v cos θ``, ``ẏ = v sin θ``, ``θ̇ = (v / L) tan δ``

    ``v̇ = a_x``, ``δ̇ = δ̇_u``

    EoM ``params``: ``length`` only (``L``). Axle offsets ``a`` / ``b`` are
    graphics attributes (default ``length / 2``), not EoM params.
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleAcc"
        self.n = 5
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)
        self.state.labels = ["x", "y", "theta", "v", "delta"]
        self.state.units = ["m", "m", "rad", "m/s", "rad"]

        self.inputs = {}
        self.add_input_port(
            "u",
            dim=2,
            nominal_value=np.zeros(2),
            labels=["a_x", "delta_dot"],
            units=["m/s^2", "rad/s"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        length = params["length"]
        theta = x[2]
        v = x[3]
        delta = x[4]
        a_x = u[0]
        delta_dot = u[1]

        pose_dot = jnp.array(
            [
                v * jnp.cos(theta),
                v * jnp.sin(theta),
                v * jnp.tan(delta) / length,
            ]
        )
        return jnp.concatenate([pose_dot, jnp.array([a_x, delta_dot])])

    def tf(self, x, u, t=0, params=None):
        a = self.a
        delta = x[4]
        T_wb = SE2(x[0], x[1], x[2])
        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, delta),
        }


class BicycleAccPorts(BicycleAcc):
    """:class:`BicycleAcc` with named inputs ``a_x`` and ``delta_dot``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleAcc (named ports)"
        self.inputs = {}
        self.add_input_port("a_x", nominal_value=0.0, labels=["a_x"], units=["m/s^2"])
        self.add_input_port(
            "delta_dot",
            nominal_value=0.0,
            labels=["delta_dot"],
            units=["rad/s"],
        )

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        a_x, delta_dot = self.get_port_values_from_u(u, "a_x", "delta_dot")
        return super().f(x, jnp.array([a_x[0], delta_dot[0]]), t, params)


# ---------------------------------------------------------------------------
# Dynamic bicycle (planar body + linear tires)
# ---------------------------------------------------------------------------


class BicycleDyn(DynamicSystem):
    """Dynamic bicycle with rear-wheel rate and steer-angle inputs.

    State ``x = [x, y, theta, vx, vy, yaw_rate]`` (world pose + body velocities).
    Input ``u = [w_rear, delta]``.

    Centered CG in the EoM: ``a = b = length / 2``. All tire coefficients live
    in ``params``; :func:`linear_tire_forces` is pure (no tire objects in ``f``).
    """

    def __init__(self):
        super().__init__(n=6, input_dim=2, output_dim=6, expose_state=True)
        self.name = "BicycleDyn"
        length = 2.0
        self.params = {
            "length": length,
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
        self.state.labels = ["x", "y", "theta", "vx", "vy", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["w_rear", "delta"]
        self.inputs["u"].units = ["rad/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        _bicycle_graphics_attrs(self, length)
        self.track = 1.92
        self.body_height = 0.22
        self.body_width_ratio = 0.72
        self.body_length_overhang = 0.26
        self.body_ground_clearance = 0.003
        self.ground_plane_size = 120.0
        self._visual_wheel_width = 0.2
        self._visual_tire_radius_ratio = 0.58

    # --- Rigid-body matrices ---

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
        length = params["length"]
        a = 0.5 * length
        b = 0.5 * length
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
        length = params["length"]
        a = 0.5 * length
        b = 0.5 * length
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
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        length = params["length"]
        a = 0.5 * length
        b = 0.5 * length
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

    def rear_wheel_ground_torque(self, v_body, w_rear, delta, params=None):
        """Tire reaction plus viscous torque on the rear wheel [Nm].

        Satisfies ``Jw * w_rear_dot = tau_motor - tau_ground``.
        """
        jnp = require_jax_numpy()
        params = self.params if params is None else params
        r_r = params["r_r"]
        bw = params.get("bw_rear", 0.0)

        u_in = jnp.array([w_rear, delta])
        _, _, Fx_r, _ = self.compute_tire_physics(v_body, u_in, params)
        return r_r * Fx_r + bw * w_rear

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        u_in = jnp.asarray(u)

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

    def _u_in(self, x, u):
        jnp = require_jax_numpy()
        return jnp.asarray(u)

    def tf(self, x, u, t=0, params=None):
        a = self.a
        delta = self._u_in(x, u)[1]
        T_wb = SE2(x[0], x[1], x[2])
        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, delta),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        length = params["length"]
        a = 0.5 * length
        b = 0.5 * length
        vb = x[3:6]
        u_in = self._u_in(x, u)
        delta = u_in[1]

        uu, vv, wr = vb[0], vb[1], vb[2]
        v_f_loc = np.array([uu, vv + a * wr])
        v_r_loc = np.array([uu, vv - b * wr])

        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(vb, u_in, params)
        # NumPy arrays for Arrow primitives (graphics path, not JAX-traced).
        Fx_f, Fy_f, Fx_r, Fy_r = (
            float(Fx_f),
            float(Fy_f),
            float(Fx_r),
            float(Fy_r),
        )
        cd, sd = np.cos(float(delta)), np.sin(float(delta))
        F_f_loc = np.array([Fx_f * cd - Fy_f * sd, Fx_f * sd + Fy_f * cd])
        F_r_loc = np.array([Fx_r, Fy_r])
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


class BicycleDynPorts(BicycleDyn):
    """:class:`BicycleDyn` with named inputs ``w_rear`` and ``delta``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleDyn (named ports)"
        self.inputs = {}
        self.add_input_port(
            "w_rear", nominal_value=0.0, labels=["w_rear"], units=["rad/s"]
        )
        self.add_input_port("delta", nominal_value=0.0, labels=["delta"], units=["rad"])

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        return super().f(x, jnp.array([w_rear[0], delta[0]]), t, params)

    def _u_in(self, x, u):
        jnp = require_jax_numpy()
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        return jnp.array([w_rear[0], delta[0]])


# ---------------------------------------------------------------------------
# Actuation ladder: Rate → TauRate → Servo → Engine
# ---------------------------------------------------------------------------


class BicycleDynRate(BicycleDyn):
    """Dynamic bicycle with rear-wheel and steer *rate* inputs.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta]``.
    Input ``u = [w_rear_dot, delta_dot]``.

    Extra EoM params: ``Jw_rear``, ``bw_rear`` (for
    :meth:`inverse_propulsion_dynamics` / ground torque).
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleDynRate"
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

        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0

        self.inputs = {}
        self.add_input_port(
            "u",
            dim=2,
            nominal_value=np.zeros(2),
            labels=["w_rear_dot", "delta_dot"],
            units=["rad/s^2", "rad/s"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        u_in = x[6:8]  # [w_rear, delta]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Wheel/steer commands integrate the rate inputs; body follows the EoM.
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v
        return jnp.concatenate([dq, dv, jnp.asarray(u)])

    def _u_in(self, x, u):
        return x[6:8]

    def inverse_propulsion_dynamics(self, x, u, t=0.0, params=None):
        """Rear motor torque [Nm] that produces commanded ``w_rear_dot = u[0]``.

        Inverts ``Jw_rear * w_rear_dot = tau_rear - rear_wheel_ground_torque(...)``.
        """
        params = self.params if params is None else params
        v_body = x[3:6]
        w_rear = x[6]
        delta = x[7]
        w_rear_dot = u[0]
        Jw = params["Jw_rear"]

        tau_ground = self.rear_wheel_ground_torque(v_body, w_rear, delta, params)
        return Jw * w_rear_dot + tau_ground


class BicycleDynRatePorts(BicycleDynRate):
    """:class:`BicycleDynRate` with named inputs ``w_rear_dot`` and ``delta_dot``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleDynRate (named ports)"
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
            units=["rad/s"],
        )

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        w_dot, d_dot = self.get_port_values_from_u(u, "w_rear_dot", "delta_dot")
        return super().f(x, jnp.array([w_dot[0], d_dot[0]]), t, params)


class BicycleDynTauRate(BicycleDyn):
    """Dynamic bicycle with direct rear torque and steer-rate inputs.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta]``.
    Input ``u = [tau_rear, delta_dot]``.

    Wheel spin: ``Jw_rear * w_rear_dot = tau_rear - rear_wheel_ground_torque``.
    Steer integrates ``delta_dot`` directly (no lag).
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleDynTauRate"
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

        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0

        self.inputs = {}
        self.add_input_port(
            "u",
            dim=2,
            nominal_value=np.zeros(2),
            labels=["tau_rear", "delta_dot"],
            units=["Nm", "rad/s"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        u_in = x[6:8]

        tau_rear = u[0]
        delta_dot = u[1]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        w_rear_dot = (tau_rear - tau_ground) / params["Jw_rear"]

        return jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot])])

    def _u_in(self, x, u):
        return x[6:8]


class BicycleDynTauRatePorts(BicycleDynTauRate):
    """:class:`BicycleDynTauRate` with named inputs ``tau_rear`` and ``delta_dot``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleDynTauRate (named ports)"
        self.inputs = {}
        self.add_input_port(
            "tau_rear",
            nominal_value=0.0,
            labels=["tau_rear"],
            units=["Nm"],
        )
        self.add_input_port(
            "delta_dot",
            nominal_value=0.0,
            labels=["delta_dot"],
            units=["rad/s"],
        )

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        tau, d_dot = self.get_port_values_from_u(u, "tau_rear", "delta_dot")
        return super().f(x, jnp.array([tau[0], d_dot[0]]), t, params)


class BicycleDynServo(BicycleDyn):
    """Dynamic bicycle with first-order lag on torque and steer commands.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta, tau]``.
    Input ``u = [tau_cmd, delta_cmd]``.

    Actuator dynamics
    -----------------
    ``tau_dot = (tau_cmd - tau) / torque_tau``

    ``delta_dot = clip((delta_cmd - delta) / steering_tau, ±steer_rate_max)``

    No steer-angle hard-stop in ``f`` (angle caps live on ports / ``CarLimits``).
    Wheel: ``Jw_rear * w_rear_dot = tau - rear_wheel_ground_torque``.
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleDynServo"
        self.n = 9
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
            "tau",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad",
            "Nm",
        ]

        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0
        self.params["steering_tau"] = 0.15
        self.params["steer_rate_max"] = 10.0
        self.params["torque_tau"] = 0.05

        self.inputs = {}
        self.add_input_port(
            "u",
            dim=2,
            nominal_value=np.zeros(2),
            labels=["tau_cmd", "delta_cmd"],
            units=["Nm", "rad"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        tau = x[8]
        u_in = x[6:8]

        tau_cmd = u[0]
        delta_cmd = u[1]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        w_rear_dot = (tau - tau_ground) / params["Jw_rear"]

        torque_tau = params["torque_tau"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        tau_dot = (tau_cmd - tau) / torque_tau
        delta_dot = (delta_cmd - delta) / steering_tau
        delta_dot = jnp.clip(delta_dot, -rate_max, rate_max)

        return jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot, tau_dot])])

    def _u_in(self, x, u):
        return x[6:8]


class BicycleDynServoPorts(BicycleDynServo):
    """:class:`BicycleDynServo` with named inputs ``tau_cmd`` and ``delta_cmd``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleDynServo (named ports)"
        self.inputs = {}
        self.add_input_port(
            "tau_cmd",
            nominal_value=0.0,
            labels=["tau_cmd"],
            units=["Nm"],
        )
        self.add_input_port(
            "delta_cmd",
            nominal_value=0.0,
            labels=["delta_cmd"],
            units=["rad"],
        )

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        tau_cmd, delta_cmd = self.get_port_values_from_u(u, "tau_cmd", "delta_cmd")
        return super().f(x, jnp.array([tau_cmd[0], delta_cmd[0]]), t, params)


class BicycleDynEngine(BicycleDyn):
    """Dynamic bicycle with lagged wheel power and steer commands.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta, P]``.
    Input ``u = [P_cmd, delta_cmd]`` (wheel-frame watts; no gearbox).

    Actuator / propulsion
    ---------------------
    ``P_dot = (P_cmd - P) / engine_tau`` — first-order lag on commanded power.
    ``P_cmd`` is not clipped in ``f`` (peaks live on ports / ``CarLimits``).

    ``τ = clip(P / ω_r, ±τ_sat)`` — stall / low-speed torque limit. Under
    saturation, delivered ``τ ω_r`` can be less than lagged ``|P|`` (``P`` is
    filtered command power, not measured shaft power).

    Engine / drivetrain brake (dry + viscous), separate from tire viscous
    ``bw_rear`` inside :meth:`rear_wheel_ground_torque`::

        τ_brake = bw_engine · ω_r + tau_fric · sign(ω_r)
        Jw_rear · ω̇_r = τ − τ_ground − τ_brake

    At constant ``P``, brake + body aero yield a terminal speed.

    Steer (rate sat only; same idea as :class:`BicycleDynServo`)
    -----------------------------------------------------------
    ``delta_dot = clip((delta_cmd - delta) / steering_tau, ±steer_rate_max)``

    No steer-angle hard-stop and no clip of ``delta_cmd`` in ``f``.
    """

    def __init__(self):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleDynEngine"
        self.n = 9
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
            "P",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad",
            "W",
        ]

        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0
        self.params["steering_tau"] = 0.15
        self.params["steer_rate_max"] = 10.0
        self.params["engine_tau"] = 0.25
        self.params["tau_sat"] = 2500.0
        self.params["bw_engine"] = 2.0
        self.params["tau_fric"] = 20.0

        self.inputs = {}
        self.add_input_port(
            "u",
            dim=2,
            nominal_value=np.zeros(2),
            labels=["P_cmd", "delta_cmd"],
            units=["W", "rad"],
        )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        params = self.params if params is None else params

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        P = x[8]
        u_in = x[6:8]

        P_cmd = u[0]
        delta_cmd = u[1]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        tau_sat = params["tau_sat"]
        # ω≈0 → τ = τ_sat · sign(P); else clip(P/ω, ±τ_sat). Safe denom avoids 0/0.
        w_safe = jnp.where(jnp.abs(w_rear) < 1e-6, 1e-6, w_rear)
        tau = jnp.clip(P / w_safe, -tau_sat, tau_sat)

        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        tau_brake = params["bw_engine"] * w_rear + params["tau_fric"] * jnp.sign(w_rear)
        w_rear_dot = (tau - tau_ground - tau_brake) / params["Jw_rear"]

        engine_tau = params["engine_tau"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        P_dot = (P_cmd - P) / engine_tau
        delta_dot = (delta_cmd - delta) / steering_tau
        delta_dot = jnp.clip(delta_dot, -rate_max, rate_max)

        return jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot, P_dot])])

    def _u_in(self, x, u):
        return x[6:8]


class BicycleDynEnginePorts(BicycleDynEngine):
    """:class:`BicycleDynEngine` with named inputs ``P_cmd`` and ``delta_cmd``."""

    def __init__(self):
        super().__init__()
        self.name = "BicycleDynEngine (named ports)"
        self.inputs = {}
        self.add_input_port(
            "P_cmd",
            nominal_value=0.0,
            labels=["P_cmd"],
            units=["W"],
        )
        self.add_input_port(
            "delta_cmd",
            nominal_value=0.0,
            labels=["delta_cmd"],
            units=["rad"],
        )

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()
        P_cmd, delta_cmd = self.get_port_values_from_u(u, "P_cmd", "delta_cmd")
        return super().f(x, jnp.array([P_cmd[0], delta_cmd[0]]), t, params)


if __name__ == "__main__":
    sys = BicycleKin()
    print(sys.name, "n =", sys.n, "params =", sys.params)
