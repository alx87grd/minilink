"""Dynamic bicycle: a planar rigid body on its tires, and its wheel-rate / steer-rate rung."""

import copy
from functools import partial

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.core.params import held_params
from minilink.core.signals import VectorSignal
from minilink.core.system import DynamicSystem, fastest_time_constant
from minilink.dynamics.catalog.vehicles.terrain import Terrain
from minilink.dynamics.catalog.vehicles.tires import LinearSaturation
from minilink.graphical.animation.primitives import Arrow
from minilink.graphical.catalog.skins import car_skin_2d

###############################################################################
# Public API
###############################################################################


class DynamicBicycle(DynamicSystem):
    """
    Dynamic bicycle with rear wheel speed and steer inputs.

    State ``x = [x, y, psi, vx, vy, yaw_rate]``: the world pose and the body-frame
    velocities. The two inputs come as one stacked ``u`` port, the convention of every
    bicycle rung, or as two named ports with ``named_ports=True``. The research rungs live in
    ``examples/projects/car_trajopt/vehicles/ladder.py``.

    Tires
    -----
    The car's wheels are ``("f", "r")``. With no ``tire`` given, both run the linear
    tire saturated on its friction circle
    (:class:`~minilink.dynamics.catalog.vehicles.tires.LinearSaturation`) on the car's
    own ``Ca``, ``Ck``, ``mu`` and slip floor ``v_min_epsilon``. A tire given, one for
    both wheels or a dict by wheel name, is a part: each wheel holds its own copy, its
    numbers at ``params["tire"][wheel]``, and the car's tire keys leave its ``params``.
    The road is a ``terrain`` (flat by default), its numbers at
    ``params["terrain"]``: its friction factor under each contact, 1 on the reference
    surface, multiplies the tire's ``mu`` (``params["terrain"]["friction"]`` for a
    uniform road, a terrain subclass for a map). The planar car reads only the
    terrain's friction, never its height.

    Inputs
    ------
    w_rear : rear wheel angular rate [rad/s]
    delta  : front steer angle [rad]
    """

    params = held_params()
    wheels = ("f", "r")

    def __init__(self, named_ports=False, tire=None, terrain=None):
        super().__init__(state_dim=6)
        self.named_ports = bool(named_ports)

        # Parameters: CG-to-axle distances a/b [m], wheel radii r_f/r_r [m],
        # mass [kg], yaw inertia [kg m^2], gravity [m/s^2], air density rho
        # [kg/m^3], drag area CdA = Cd * A [m^2], and the tires: absolute
        # stiffnesses shared by both axles, cornering Ca [N/rad] and longitudinal
        # Ck [N], friction mu [-] and the slip regularization speed v_min_epsilon
        # [m/s] (the keys of the car_trajopt project's ladder). A law of the tire
        # module carries its own instead
        self.params.update(
            {
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
        )

        # The road: a part holding its own numbers, its friction factor among them
        self.terrain = Terrain() if terrain is None else copy.deepcopy(terrain)
        self.params["terrain"] = self.terrain.params

        # The tires, chosen once: the saturated linear tire on the car's own numbers,
        # or a part per wheel holding its own
        if tire is None:
            self.tire = {wheel: LinearSaturation() for wheel in self.wheels}
            self.tire_is_part = False
            self.axle_forces = self.lent_tire_forces
            self.axle_stiffnesses = self.lent_tire_stiffnesses
        else:
            self.tire = wheel_parts(tire, self.wheels)
            self.tire_is_part = True
            for key in ("Ca", "Ck", "mu", "v_min_epsilon"):
                del self.params[key]
            self.params["tire"] = {
                wheel: self.tire[wheel].params for wheel in self.wheels
            }
            self.axle_forces = self.part_tire_forces
            self.axle_stiffnesses = self.part_tire_stiffnesses

        # Ports: two named commands, or one stacked command port (the planning and
        # trajectory-optimization convention)
        if self.named_ports:
            self.add_input_port(
                "w_rear", nominal_value=0.0, labels=["w_rear"], units=["rad/s"]
            )
            self.add_input_port(
                "delta", nominal_value=0.0, labels=["delta"], units=["rad"]
            )
        else:
            self.add_input_port(
                "u",
                dim=2,
                nominal_value=np.zeros(2),
                labels=["w_rear", "delta"],
                units=["rad/s", "rad"],
            )
        self.add_output_port("y", dim=6, function=self.h, dependencies=())

        # Signal labels and units
        self.state.labels = ["x", "y", "psi", "vx", "vy", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]

        # Graphics: the 2-D centerline look
        self.wheel_len = 0.76
        self.wheel_width = 0.27

        # Graphics: the 3-D four-wheel look (read by ``car_skin_3d`` / ``tf``). It
        # lives on the base plant, so the 3-D look is just ``skin = car_skin_3d``,
        # and it does not affect the default 2-D centerline skin. A wide stance, a
        # low body and a wide tub, a small gap between the body's underside and the
        # tire crown, and larger tires (display only: the wheel centers use r_f / r_r)
        self.track = 1.92
        self.body_height = 0.22
        self.body_width_ratio = 0.72
        self.body_length_overhang = 0.26
        self.body_ground_clearance = 0.003
        self.ground_plane_size = 120.0
        self._visual_wheel_width = 0.2
        self._visual_tire_radius_ratio = 0.58

        # Graphics: the default 2-D skin (black centerline chassis) and a camera that
        # tracks the body frame. A numeric scale keeps auto-fit from taking over
        # (``camera_scale=None`` would frame the whole trail / overlays).
        self.skin = partial(car_skin_2d, color="#1a1a1a")
        self.camera_follow_frame = "body"
        self.camera_scale = 10.0

    @property
    def solver_info(self) -> dict:
        """The solver hints, the smallest time constant read from the current ``params`` (:meth:`fastest_mode`)."""
        info = self.own_solver_info
        info["smallest_time_constant"] = self.fastest_mode()

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

    def fastest_mode(self) -> float:
        """The solver hint: the fastest tire mode of the body at rest, in seconds.

        At rest the slip denominators sit on their floor ``v_eps``, where
        the tires are stiffest: the body's velocities ``[vx, vy, r]`` then relax on
        the linearization below (the rear tire pulling ``vx`` to the wheel's
        speed, the two cornering forces coupling ``vy`` and the yaw rate ``r``),
        and ``τ = 1 / max |λᵢ|`` of its modes.
        """
        params = self.params

        a = params["a"]
        b = params["b"]
        mass = params["mass"]
        inertia = params["inertia"]

        C_alpha_f, C_alpha_r, C_kappa, v_eps = self.axle_stiffnesses(params)

        # m v̇x = −C_κ vx / ε, the rear tire alone (the front wheel rolls free)
        A11 = -C_kappa / (mass * v_eps)

        # m v̇y = C_αf α_f + C_αr α_r and I ṙ = a C_αf α_f − b C_αr α_r, with
        # α_f = −(vy + a r) / ε and α_r = −(vy − b r) / ε
        A22 = -(C_alpha_f + C_alpha_r) / (mass * v_eps)
        A23 = -(a * C_alpha_f - b * C_alpha_r) / (mass * v_eps)
        A32 = -(a * C_alpha_f - b * C_alpha_r) / (inertia * v_eps)
        A33 = -(a**2 * C_alpha_f + b**2 * C_alpha_r) / (inertia * v_eps)

        # fmt: off
        A = np.array([
            [A11, 0.0, 0.0],
            [0.0, A22, A23],
            [0.0, A32, A33],
        ])
        # fmt: on

        tau = fastest_time_constant(np.linalg.eigvals(A))

        return tau

    def M(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        inertia = params["inertia"]

        # Mass matrix of the planar rigid body: diag(m, m, I_z)
        M = xp.diag(xp.array([mass, mass, inertia], dtype=float))

        return M

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        xp = array_module(q, v)

        mass = params["mass"]

        r = v[2]

        # Coriolis coupling of the body-frame velocities by the yaw rate r
        # fmt: off
        C = xp.array([
            [    0.0, -mass * r, 0.0],
            [mass * r,      0.0, 0.0],
            [    0.0,       0.0, 0.0],
        ])
        # fmt: on

        return C

    def N(self, q, params=None):
        xp = array_module(q)

        psi = q[2]
        c = xp.cos(psi)
        s = xp.sin(psi)

        # World-frame velocity kinematics: q̇ = N(q) v
        # fmt: off
        N = xp.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        # fmt: on

        return N

    def compute_wheel_velocities(self, v_body, u_inputs, params=None):
        params = self.params if params is None else params
        xp = array_module(v_body, u_inputs)

        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]

        vx = v_body[0]
        vy = v_body[1]
        r = v_body[2]
        delta = u_inputs[1]

        # Contact-point velocities in the body frame
        vx_f_b = vx
        vy_f_b = vy + a * r
        vx_r_b = vx
        vy_r_b = vy - b * r

        # The same velocities in each wheel frame: the front one turned by the steer δ
        c_d = xp.cos(delta)
        s_d = xp.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        # The rear wheel rate is the input; the front wheel rolls: ω_f = vx_f / r_f
        w_r = u_inputs[0]
        w_f = vx_f / r_f

        return vx_f, vy_f, w_f, vx_r, vy_r, w_r

    def compute_tire_physics(self, v_body, u_inputs, params=None, q=None):
        """Axle forces ``(Fx_f, Fy_f, Fx_r, Fy_r)`` [N], each in its own wheel frame.

        ``q`` is the pose ``[x, y, psi]``, where the road's friction is read; without
        it the car stands at the origin.
        """
        xp = array_module(v_body, u_inputs)
        q = xp.zeros(3) if q is None else q

        Fx_f, Fy_f, Fx_r, Fy_r = self.axle_forces(v_body, u_inputs, q, params)

        return Fx_f, Fy_f, Fx_r, Fy_r

    def contact_friction(self, q, params=None):
        """The road's friction factor ``(friction_f, friction_r)`` [-] under the two axles."""
        params = self.params if params is None else params
        xp = array_module(q)

        terrain = self.terrain
        terrain_params = params["terrain"]
        a = params["a"]
        b = params["b"]

        X = q[0]
        Y = q[1]
        psi = q[2]
        c_psi = xp.cos(psi)
        s_psi = xp.sin(psi)

        # The contacts on the centreline, a ahead of and b behind the centre of gravity
        friction_f = terrain.friction(X + a * c_psi, Y + a * s_psi, 0.0, terrain_params)
        friction_r = terrain.friction(X - b * c_psi, Y - b * s_psi, 0.0, terrain_params)

        return friction_f, friction_r

    def lent_tire_forces(self, v_body, u_inputs, q, params=None):
        """The axle forces of the saturated linear tire, on the car's ``Ca``, ``Ck``, ``mu``."""
        params = self.params if params is None else params

        tire_f = self.tire["f"]
        tire_r = self.tire["r"]
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
        friction_f, friction_r = self.contact_friction(q, params)

        L = a + b
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs, params
        )

        # The car lends its own numbers to both wheels' tires
        lent = {"mu": mu, "Ca": Ca, "Ck": Ck, "v_eps": v_min_epsilon}

        # Static normal-load split between the axles: F_zf = m g b / L, F_zr = m g a / L
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        # The slips at each contact, then the saturated linear forces
        kappa_f, alpha_f = tire_f.slip(vx_f, vy_f, w_f, r_f, lent)
        kappa_r, alpha_r = tire_r.slip(vx_r, vy_r, w_r, r_r, lent)
        Fx_f, Fy_f = tire_f.forces(kappa_f, alpha_f, Fz_f, friction_f, lent)
        Fx_r, Fy_r = tire_r.forces(kappa_r, alpha_r, Fz_r, friction_r, lent)

        return Fx_f, Fy_f, Fx_r, Fy_r

    def part_tire_forces(self, v_body, u_inputs, q, params=None):
        """The axle forces of each wheel's own tire, on its own numbers."""
        params = self.params if params is None else params

        tire_f = self.tire["f"]
        tire_r = self.tire["r"]
        p_f = params["tire"]["f"]
        p_r = params["tire"]["r"]
        a = params["a"]
        b = params["b"]
        r_f = params["r_f"]
        r_r = params["r_r"]
        mass = params["mass"]
        gravity = params["gravity"]
        friction_f, friction_r = self.contact_friction(q, params)

        L = a + b
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs, params
        )

        # Static normal-load split between the axles: F_zf = m g b / L, F_zr = m g a / L
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        # The slips at each contact, then each tire's forces
        kappa_f, alpha_f = tire_f.slip(vx_f, vy_f, w_f, r_f, p_f)
        kappa_r, alpha_r = tire_r.slip(vx_r, vy_r, w_r, r_r, p_r)
        Fx_f, Fy_f = tire_f.forces(kappa_f, alpha_f, Fz_f, friction_f, p_f)
        Fx_r, Fy_r = tire_r.forces(kappa_r, alpha_r, Fz_r, friction_r, p_r)

        return Fx_f, Fy_f, Fx_r, Fy_r

    def lent_tire_stiffnesses(self, params):
        """``(C_alpha_f, C_alpha_r, C_kappa, v_eps)``: the car's shared ``Ca``, ``Ck`` and floor."""
        Ca = params["Ca"]
        Ck = params["Ck"]
        v_eps = params["v_min_epsilon"]

        return Ca, Ca, Ck, v_eps

    def part_tire_stiffnesses(self, params):
        """``(C_alpha_f, C_alpha_r, C_kappa, v_eps)``: each tire's at the static loads, the smaller floor."""
        a = params["a"]
        b = params["b"]
        mass = params["mass"]
        gravity = params["gravity"]
        p_f = params["tire"]["f"]
        p_r = params["tire"]["r"]

        # Static axle loads, and the stiffnesses each tire has under them
        Fz_f = mass * gravity * b / (a + b)
        Fz_r = mass * gravity * a / (a + b)
        _, C_alpha_f = self.tire["f"].stiffnesses(Fz_f, p_f)
        C_kappa, C_alpha_r = self.tire["r"].stiffnesses(Fz_r, p_r)
        v_eps = min(p_f["v_eps"], p_r["v_eps"])

        return C_alpha_f, C_alpha_r, C_kappa, v_eps

    def generalized_d(self, q, v, u_in, params=None):
        params = self.params if params is None else params
        xp = array_module(v, u_in)

        a = params["a"]
        b = params["b"]
        rho = params["rho"]
        CdA = params["CdA"]

        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v, u_in, params, q)
        delta = u_in[1]
        c_d = xp.cos(delta)
        s_d = xp.sin(delta)
        vx = v[0]

        # Tire forces in the body frame: the front pair turned by the steer δ
        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d
        Fx_r_b = Fx_r
        Fy_r_b = Fy_r

        # Resultant wrench: ΣF_x less the drag ½ ρ CdA vx |vx|, ΣF_y and the yaw
        # moment ΣM_z = a F_yf − b F_yr
        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = a * Fy_f_b - b * Fy_r_b
        F_aero = 0.5 * rho * CdA * vx * xp.abs(vx)
        Sum_Fx = Sum_Fx - F_aero

        # The generalized damping is minus the resultant tire and drag wrench
        d = -xp.array([Sum_Fx, Sum_Fy, Sum_Mz])

        return d

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        q = x[0:3]
        v = x[3:6]
        u_in = self.wheel_rate_and_steer(x, u)

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Rigid-body EoM in the body frame: M v̇ + C v + d = 0, and q̇ = N v
        dv = xp.linalg.solve(M, -C @ v - d)
        dq = N @ v
        dx = xp.concatenate([dq, dv])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def rear_wheel_ground_torque(self, v_body, w_rear, delta, params=None, q=None):
        """Tire reaction plus viscous torque on the rear wheel [Nm].

        Satisfies ``Jw * w_rear_dot = tau_motor - tau_ground`` for the rate,
        torque, servo, and engine variants built on this model.
        """
        params = self.params if params is None else params
        xp = array_module(v_body, w_rear, delta)

        r_r = params["r_r"]
        bw = params.get("bw_rear", 0.0)

        u_in = xp.array([w_rear, delta])
        _, _, Fx_r, _ = self.compute_tire_physics(v_body, u_in, params, q)

        # τ_ground = r_r F_xr + b_w ω_rear: the road reaction through the rear tire,
        # plus the viscous bearing torque
        tau_ground = r_r * Fx_r + bw * w_rear

        return tau_ground

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        a = params["a"]
        r_f = params["r_f"]
        tr = self.track

        delta = self.wheel_rate_and_steer(x, u)[1]
        # The body frame: 2-D chassis, 3-D body box, rear wheels, contact arrows
        T_wb = SE2(x[0], x[1], x[2])
        R_steer = SE2(0.0, 0.0, delta)

        # The 2-D front axle, steered; the rear axle's wheels ride on ``body``
        # through ``local_transform`` in the skin
        T_front = T_wb @ SE2(a, 0.0, delta)

        # The 3-D steered front wheels (lift and steer need frames of their own)
        T_fl = T_wb @ translation(a, 0.5 * tr, r_f) @ R_steer
        T_fr = T_wb @ translation(a, -0.5 * tr, r_f) @ R_steer

        return {"body": T_wb, "axle_front": T_front, "wheel_fl": T_fl, "wheel_fr": T_fr}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """2-D centerline look: one velocity and one force arrow per axle."""
        params = self.params if params is None else params

        b = params["b"]
        a = params["a"]

        v_r_loc, v_f_loc, F_r_loc, F_f_loc = self.contact_fields(x, u, params)

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

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def param_parts(self) -> dict:
        """The parts whose own numbers the tree holds: the terrain, each wheel's tire when it is a part."""
        parts = {"terrain": self.terrain}
        if self.tire_is_part:
            parts["tire"] = dict(self.tire)

        return parts

    def wheel_rate_and_steer(self, x, u):
        """``[w_rear, delta]`` from the port vector (overridden by the rate variant).

        The two named ports stack in that order, so the port vector is the pair.
        """
        xp = array_module(x, u)

        u_in = xp.asarray(u)

        return u_in

    def contact_fields(self, x, u, params=None):
        """Body-frame per-axle velocities and tire forces for the arrow geometry.

        Returns ``(v_r_loc, v_f_loc, F_r_loc, F_f_loc)`` — the rear/front contact
        velocity and tire force in the body frame. The ``body`` ``tf`` rotates them
        to world, so no manual ``cos``/``sin`` here.
        """
        params = self.params if params is None else params

        a = params["a"]
        b = params["b"]

        q = x[0:3]
        v_body = x[3:6]
        u_in = self.wheel_rate_and_steer(x, u)
        delta = u_in[1]
        vx = v_body[0]
        vy = v_body[1]
        r = v_body[2]

        # Contact velocities at the front and rear axles
        v_f_loc = np.array([vx, vy + a * r])
        v_r_loc = np.array([vx, vy - b * r])

        # Tire forces: the front pair turned from the steered wheel to the body; the
        # rear pair is already in the body frame
        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v_body, u_in, params, q)
        cd = np.cos(delta)
        sd = np.sin(delta)
        Fx_f_b = Fx_f * cd - Fy_f * sd
        Fy_f_b = Fx_f * sd + Fy_f * cd
        F_f_loc = np.array([Fx_f_b, Fy_f_b])
        F_r_loc = np.array([Fx_r, Fy_r])

        return v_r_loc, v_f_loc, F_r_loc, F_f_loc


class BicycleDynRate(DynamicBicycle):
    """Dynamic bicycle with rear-wheel and steer *rate* inputs (the MPC plant).

    State ``x = [x, y, psi, vx, vy, yaw_rate, w_rear, delta]``: the wheel
    rate and the steer angle are states driven by the inputs
    ``[w_rear_dot, delta_dot]`` — one stacked ``u`` port, or ``named_ports=True``
    for a ``w_rear_dot`` and a ``delta_dot`` port. ``Jw_rear`` / ``bw_rear``
    feed :meth:`inverse_propulsion_dynamics`. ``tire`` as on
    :class:`DynamicBicycle`.
    """

    def __init__(self, named_ports=False, tire=None, terrain=None):
        super().__init__(named_ports=False, tire=tire, terrain=terrain)
        self.named_ports = bool(named_ports)

        # State: the body's six, then the rear wheel rate and the steer angle
        self.n = 8
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        # Parameters: the rear wheel's inertia and viscous drag
        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0

        # Ports: two named rate commands, or one stacked port
        self.inputs = {}
        if self.named_ports:
            self.add_input_port(
                "w_rear_dot",
                nominal_value=0.0,
                labels=["w_rear_dot"],
                units=["rad/s^2"],
            )
            self.add_input_port(
                "delta_dot", nominal_value=0.0, labels=["delta_dot"], units=["rad/s"]
            )
        else:
            self.add_input_port(
                "u",
                dim=2,
                nominal_value=np.zeros(2),
                labels=["w_rear_dot", "delta_dot"],
                units=["rad/s^2", "rad/s"],
            )
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.add_output_port("x", dim=self.n, function=self.compute_state)

        # Signal labels and units
        self.state.labels = [
            "x",
            "y",
            "psi",
            "vx",
            "vy",
            "yaw_rate",
            "w_rear",
            "delta",
        ]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        q = x[0:3]
        v = x[3:6]
        u_in = x[6:8]

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)
        rates = self.rates(x, u)

        # The body follows the rigid-body EoM M v̇ + C v + d = 0, q̇ = N v; the wheel
        # rate and the steer angle integrate the rate commands
        dv = xp.linalg.solve(M, -C @ v - d)
        dq = N @ v
        dx = xp.concatenate([dq, dv, rates])

        return dx

    def inverse_propulsion_dynamics(self, x, u, t=0.0, params=None):
        """Rear motor torque [Nm] that produces the commanded ``w_rear_dot``.

        Inverts ``Jw_rear * w_rear_dot = tau_rear - rear_wheel_ground_torque(...)``.
        """
        params = self.params if params is None else params

        Jw_rear = params["Jw_rear"]

        q = x[0:3]
        v_body = x[3:6]
        w_rear = x[6]
        delta = x[7]
        w_rear_dot = self.rates(x, u)[0]
        tau_ground = self.rear_wheel_ground_torque(v_body, w_rear, delta, params, q)

        # Jw ẇ_rear = τ_rear − τ_ground, solved for the motor torque
        tau_rear = Jw_rear * w_rear_dot + tau_ground

        return tau_rear

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def rates(self, x, u):
        """``[w_rear_dot, delta_dot]`` from the port vector: the two named ports in order."""
        xp = array_module(x, u)

        rates = xp.asarray(u)

        return rates

    def wheel_rate_and_steer(self, x, u):
        """``[w_rear, delta]``: both are states on this rung."""
        return x[6:8]


###############################################################################
# Internal machinery
###############################################################################


def wheel_parts(tire, wheels):
    """Each wheel's own tire: a copy of the one given, or of the one given for it by name.

    The car holds copies, so a tire passed in is a template: edit the car's through
    ``car.params["tire"][wheel]``. A dict must name every wheel and no other.
    """
    if isinstance(tire, dict):
        if set(tire) != set(wheels):
            raise ValueError(
                f"tire names the wheels {sorted(tire)}; this car's wheels are "
                f"{list(wheels)}: pass one tire for all, or one per wheel by name"
            )
        return {wheel: copy.deepcopy(tire[wheel]) for wheel in wheels}

    return {wheel: copy.deepcopy(tire) for wheel in wheels}


if __name__ == "__main__":
    sys = DynamicBicycle()
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u = np.array([5.0, 0.1])
    sys.x0 = x
    sys.compute_trajectory(u=u, tf=10)
    sys.plot_trajectory()
    sys.animate()
