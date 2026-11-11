"""The 3-D car: a rigid body on four sprung corners, its tires on a terrain."""

import copy

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE3, Rx, Ry, Rz
from minilink.core.params import own_params
from minilink.core.signals import VectorSignal
from minilink.dynamics.catalog.vehicles.drivetrain_bicycle import DrivetrainBicycle
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import wheel_parts
from minilink.dynamics.catalog.vehicles.terrain import Terrain
from minilink.dynamics.catalog.vehicles.tires import (
    BrushTire,
    smooth_floor,
    smooth_sign,
)
from minilink.graphical.animation.primitives import Arrow

###############################################################################
# Public API
###############################################################################


class SuspensionCar3D(DrivetrainBicycle):
    """A 3-D car: rung 4's drivetrain bicycle on four sprung corners, on a terrain.

    State ``x = [x, y, psi, vx, vy, yaw_rate, w_rear, delta, P, z, phi, theta, vz,
    p, q]``: the drivetrain bicycle's nine, with the same meaning, then the height of
    the centre of gravity, the roll and the pitch, and their rates. ``vx, vy, vz``
    are the velocity of the centre of gravity in the heading frame (the world turned
    by ``psi``, not rolled or pitched), and ``p, q, yaw_rate`` the rates of the roll,
    the pitch and the yaw angles; the attitude is ``R = Rz(psi) Ry(theta) Rx(phi)``.
    With ``wheel_angles=True`` the rear and front rolling angles follow. A controller
    written for the drivetrain bicycle reads the first nine states unchanged.

    Inputs, drive and steering are the drivetrain bicycle's: ``u = [P_cmd,
    delta_cmd]``, a power-limited rear axle (both rear wheels at ``w_rear``, a
    locked axle) and a steering servo on the front wheels.

    Parts
    -----
    The wheels are ``("fl", "fr", "rl", "rr")``. Each holds its own tire (``tire``,
    one for all or a dict by wheel name, a
    :class:`~minilink.dynamics.catalog.vehicles.tires.BrushTire` by default) and its
    own :class:`SpringDamper` (``suspension``), their numbers at
    ``params["tire"][wheel]`` and ``params["suspension"][wheel]``. ``terrain`` (flat
    by default) gives the ground's height and its friction factor under each wheel,
    its numbers at ``params["terrain"]``; ``road_friction`` scales every tire's grip.

    Corners
    -------
    Each corner hangs from an attachment point ``c_i`` on the body, at the height of
    the centre of gravity. Its spring acts along the terrain's normal ``n_i`` under
    the attachment: the gap ``d_i`` from the attachment to the ground sets the load
    ``Fz_i``, which is the static load at the ride height ``h_cg`` and lifts off
    smoothly at zero. The tire acts at the contact point ``C_i = P_i - d_i n_i``, in
    the plane normal to ``n_i``, along the wheel's heading (the body's ``x`` axis
    projected on that plane, turned by ``delta`` at the front). Because the forces act
    at the ground, below the centre of gravity, braking pitches the car and a turn
    rolls it: the load transfer follows with no term of its own.

    Simplifications: the spring acts along the terrain's normal, not the body's ``z``
    axis; the terrain is read under the attachment point; the gap rate neglects the
    terrain's curvature; the wheels are massless. All vanish on flat ground at small
    roll. The roll-pitch-yaw attitude is singular at a pitch of ``+-90`` deg, an
    end-over-end flip. Past 90 deg of roll the same springs hold the attachments
    off the ground: only the tires touch it.

    Outputs
    -------
    ``y`` (the state), ``speed``, ``slip`` (the four slip ratios, then the four slip
    angles), ``grip`` (the used fraction of each tire's friction ellipse), ``imu``
    (the specific force at the centre of gravity and the body rates, in the body
    frame), ``power``, ``loads`` (the four normal loads) and ``rollover`` (the
    smallest load and the roll angle). ``show_contact_vectors = True`` draws each
    tire's contact velocity and force.
    """

    wheels = ("fl", "fr", "rl", "rr")

    def __init__(
        self,
        named_ports=False,
        tire=None,
        suspension=None,
        terrain=None,
        wheel_angles=False,
    ):
        super().__init__(named_ports=named_ports, tire=tire, wheel_angles=False)
        self.wheel_angles = bool(wheel_angles)
        self.suspension = wheel_parts(
            SpringDamper() if suspension is None else suspension, self.wheels
        )
        self.terrain = Terrain() if terrain is None else copy.deepcopy(terrain)

        # State: the drivetrain bicycle's nine, the body's six, the wheel angles
        self.n = 17 if self.wheel_angles else 15
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        # Parameters: the drivetrain bicycle's full-size car, then the 3-D body (the
        # track, the ride height of the centre of gravity, the roll and pitch
        # inertias; "inertia" stays the yaw inertia), the corners' springs and the
        # terrain
        self.params.update(
            {
                "track": 1.6,  # [m]
                "h_cg": 0.55,  # [m] height of the centre of gravity at rest
                "Ixx": 550.0,  # [kg m^2] roll inertia
                "Iyy": 2400.0,  # [kg m^2] pitch inertia
            }
        )
        self.params["suspension"] = {
            wheel: self.suspension[wheel].params for wheel in self.wheels
        }
        self.params["terrain"] = self.terrain.params
        self.x0[9] = self.params["h_cg"]

        # Ports: the sensors of a 3-D body
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.add_output_port("speed", function=self.speed, labels=["vx"], units=["m/s"])
        self.add_output_port(
            "slip",
            dim=8,
            function=self.slips,
            labels=[f"kappa_{w}" for w in self.wheels]
            + [f"alpha_{w}" for w in self.wheels],
            units=["-"] * 4 + ["rad"] * 4,
        )
        self.add_output_port(
            "grip",
            dim=4,
            function=self.grip,
            labels=[f"grip_{w}" for w in self.wheels],
            units=["-"] * 4,
        )
        self.add_output_port(
            "imu",
            dim=6,
            function=self.imu,
            labels=["a_x", "a_y", "a_z", "omega_x", "omega_y", "omega_z"],
            units=["m/s^2"] * 3 + ["rad/s"] * 3,
        )
        self.add_output_port(
            "power",
            dim=2,
            function=self.power,
            labels=["P", "P_wheel"],
            units=["W", "W"],
        )
        self.add_output_port(
            "loads",
            dim=4,
            function=self.loads,
            labels=[f"Fz_{w}" for w in self.wheels],
            units=["N"] * 4,
        )
        self.add_output_port(
            "rollover",
            dim=2,
            function=self.rollover,
            labels=["Fz_min", "phi"],
            units=["N", "rad"],
        )

        # Signal labels and units
        labels = ["x", "y", "psi", "vx", "vy", "yaw_rate", "w_rear", "delta", "P"]
        labels = labels + ["z", "phi", "theta", "vz", "p", "q"]
        units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad/s", "rad", "W"]
        units = units + ["m", "rad", "rad", "m/s", "rad/s", "rad/s"]
        if self.wheel_angles:
            labels = labels + ["phi_rear", "phi_front"]
            units = units + ["rad", "rad"]
        self.state.labels = labels
        self.state.units = units
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.skins import car_skin_3d

        # Graphics: the generic car's plain look, a camera that follows it, the
        # contact vectors off, and no ground drawn until scene_bounds is set
        self.skin = car_skin_3d
        self.camera_follow_frame = "body"
        self.camera_scale = 8.0
        self.show_contact_vectors = False
        self.scene_bounds = None

    @property
    def track(self) -> float:
        """The track [m], a view of ``params["track"]``."""
        return self.params["track"]

    @track.setter
    def track(self, value: float):
        self.params["track"] = value

    def fastest_mode(self) -> float:
        """The solver hint, in seconds: ten times the largest step an explicit run takes stably.

        As on the drivetrain bicycle, the tire modes at rest come from the slip
        floor ``v_eps``: the hint keeps Euler stable on the stiffest of them
        (``|λ| Δt < 2``, with a margin). The body's own modes, the heave, the roll and
        the pitch on the springs, and the actuator lags are resolved: the step is a
        tenth of each.
        """
        params = self.params

        r_r = params["r_r"]
        Jw_rear = params["Jw_rear"]
        mass = params["mass"]
        inertia = params["inertia"]
        Ixx = params["Ixx"]
        Iyy = params["Iyy"]
        a = params["a"]
        b = params["b"]
        track = params["track"]
        gravity = params["gravity"]
        engine_tau = params["engine_tau"]
        steering_tau = params["steering_tau"]
        tires = params["tire"]
        springs = params["suspension"]

        Fz0 = static_loads(mass, gravity, a, b)
        C_kappa_r = sum(
            self.tire[w].stiffnesses(Fz0[w], tires[w])[0] for w in ("rl", "rr")
        )
        C_alpha_f = sum(
            self.tire[w].stiffnesses(Fz0[w], tires[w])[1] for w in ("fl", "fr")
        )
        C_alpha_r = sum(
            self.tire[w].stiffnesses(Fz0[w], tires[w])[1] for w in ("rl", "rr")
        )
        v_eps = min(tires[w]["v_eps"] for w in self.wheels)
        k_f = springs["fl"]["k"] + springs["fr"]["k"]
        k_r = springs["rl"]["k"] + springs["rr"]["k"]

        # The tire modes at rest, as on the drivetrain bicycle
        T_long = v_eps / (C_kappa_r * (r_r**2 / Jw_rear + 1.0 / mass))
        T_yaw = inertia * v_eps / (C_alpha_f * a**2 + C_alpha_r * b**2)
        T_lat = mass * v_eps / (C_alpha_f + C_alpha_r)

        # The body on its springs: heave, roll and pitch, τ = 1 / ω
        T_heave = np.sqrt(mass / (k_f + k_r))
        T_roll = np.sqrt(Ixx / ((k_f + k_r) * (0.5 * track) ** 2))
        T_pitch = np.sqrt(Iyy / (k_f * a**2 + k_r * b**2))

        # Euler stable on the slip-floor modes, Δt = 1.9 T so τ = 19 T; the body's
        # modes and the actuator lags resolved
        T_stable = 19.0 * min(T_long, T_yaw, T_lat)
        T_min = min(T_stable, T_heave, T_roll, T_pitch, engine_tau, steering_tau)

        return T_min

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        mass = params["mass"]
        gravity = params["gravity"]
        rho = params["rho"]
        CdA = params["CdA"]
        Izz = params["inertia"]
        Ixx = params["Ixx"]
        Iyy = params["Iyy"]
        r_r = params["r_r"]
        r_f = params["r_f"]
        P_max = params["P_max"]
        engine_tau = params["engine_tau"]
        tau_sat = params["tau_sat"]
        w_eps = params["w_min_epsilon"]
        Jw_rear = params["Jw_rear"]
        bw_drive = params["bw_drive"]
        bw_rear = params["bw_rear"]
        tau_fric = params["tau_fric"]
        w_c = params["w_c"]
        delta_max = params["delta_max"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        psi = x[2]
        vx = x[3]
        vy = x[4]
        r = x[5]
        w_rear = x[6]
        delta = x[7]
        P = x[8]
        phi = x[10]
        theta = x[11]
        vz = x[12]
        p = x[13]
        q = x[14]
        P_cmd, delta_cmd = self.commands(x, u)
        c_psi = xp.cos(psi)
        s_psi = xp.sin(psi)

        # The corners: each tire's force and where it acts
        corners = self.contact_forces(x, params)
        F_world = corners["F"].sum(axis=0)
        tau_world = cross(corners["arm"], corners["F"]).sum(axis=0)
        Fx_rear = corners["Fx"][2] + corners["Fx"][3]

        # The resultant force in the heading frame, with the drag ½ ρ CdA vx |vx|
        F_aero = 0.5 * rho * CdA * vx * xp.abs(vx)
        Fx_H = c_psi * F_world[0] + s_psi * F_world[1] - F_aero
        Fy_H = -s_psi * F_world[0] + c_psi * F_world[1]
        Fz = F_world[2]

        # Translation in the heading frame, which turns at r about the vertical:
        #   m (v̇x − r vy) = ΣFx,  m (v̇y + r vx) = ΣFy,  m v̇z = ΣFz − m g
        dvx = Fx_H / mass + r * vy
        dvy = Fy_H / mass - r * vx
        dvz = Fz / mass - gravity

        # Rotation in the body frame: ω = E [p, q, r], and I ω̇ + ω × I ω = τ
        R = attitude(phi, theta, psi)
        E = euler_rate_map(phi, theta)
        omega = E @ xp.array([p, q, r])
        I_omega = xp.array([Ixx * omega[0], Iyy * omega[1], Izz * omega[2]])
        tau_body = R.T @ tau_world
        domega_num = tau_body - cross(omega, I_omega)
        domega = xp.array(
            [domega_num[0] / Ixx, domega_num[1] / Iyy, domega_num[2] / Izz]
        )

        # [ṗ, q̇, ṙ] = E⁻¹ (ω̇ − Ė [p, q, r])
        dE_rates = euler_rate_map_derivative(phi, theta, p, q, r)
        d_rates = xp.linalg.solve(E, domega - dE_rates)

        # Kinematics: [ẋ, ẏ] = Rz(ψ) [vx, vy], ż = vz, [φ̇, θ̇, ψ̇] = [p, q, r]
        dX = c_psi * vx - s_psi * vy
        dY = s_psi * vx + c_psi * vy
        dz = vz

        # Drive, as on the drivetrain bicycle, the ground torque of the locked rear
        # axle τ_ground = (Fx_rl + Fx_rr) r_r + b_w ω_rear
        P_dot = (xp.clip(P_cmd, -P_max, P_max) - P) / engine_tau
        tau = xp.clip(P / xp.sqrt(w_rear**2 + w_eps**2), -tau_sat, tau_sat)
        tau_ground = r_r * Fx_rear + bw_rear * w_rear
        tau_brake = bw_drive * w_rear + tau_fric * xp.tanh(w_rear / w_c)
        w_rear_dot = (tau - tau_ground - tau_brake) / Jw_rear

        # Servo, as on the drivetrain bicycle
        delta_ref = xp.clip(delta_cmd, -delta_max, delta_max)
        delta_dot = rate_max * xp.tanh((delta_ref - delta) / (steering_tau * rate_max))
        delta_dot = xp.where(delta > delta_max, xp.minimum(delta_dot, 0.0), delta_dot)
        delta_dot = xp.where(delta < -delta_max, xp.maximum(delta_dot, 0.0), delta_dot)

        # The rolling angles, when they are states: the rear at w_rear, the front
        # with the mean of its contacts, φ̇_front = (vx_fl + vx_fr) / (2 r_f)
        dphi_rear = w_rear
        dphi_front = 0.5 * (corners["vx"][0] + corners["vx"][1]) / r_f
        if self.wheel_angles:
            dphi = xp.array([dphi_rear, dphi_front])
        else:
            dphi = xp.zeros(0)

        # ẋ = [ẋ, ẏ, ψ̇, v̇x, v̇y, ṙ, ẇ_rear, δ̇, Ṗ, ż, φ̇, θ̇, v̇z, ṗ, q̇, φ̇_wheels]
        dx = xp.concatenate(
            [
                xp.array([dX, dY, r, dvx, dvy, d_rates[2], w_rear_dot, delta_dot]),
                xp.array([P_dot, dz, p, q, dvz, d_rates[0], d_rates[1]]),
                dphi,
            ]
        )

        return dx

    def corner_geometry(self, x, params=None):
        """Each corner's attachment, terrain normal, gap and contact point, in the world.

        Returns a dict of arrays over the wheels ``(fl, fr, rl, rr)``: ``P`` (the
        attachments, shape ``(4, 3)``), ``n`` (the terrain normals), ``d`` (the gaps
        along the normals), ``C`` (the contact points), ``R`` (the attitude) and
        ``e_x``, ``e_y`` (the wheel frames' heading and axle, on the ground plane).
        """
        params = self.params if params is None else params
        xp = array_module(x)

        terrain = self.terrain
        terrain_params = params["terrain"]
        a = params["a"]
        b = params["b"]
        track = params["track"]

        X = x[0]
        Y = x[1]
        psi = x[2]
        z = x[9]
        phi = x[10]
        theta = x[11]
        delta = x[7]
        R = attitude(phi, theta, psi)
        p_cg = xp.array([X, Y, z])

        # The attachments, on the body at the height of the centre of gravity:
        # P_i = p_cg + R c_i
        # fmt: off
        c_body = xp.array([
            [ a,  0.5 * track, 0.0],
            [ a, -0.5 * track, 0.0],
            [-b,  0.5 * track, 0.0],
            [-b, -0.5 * track, 0.0],
        ])
        # fmt: on
        P = p_cg + c_body @ R.T

        # The ground under each attachment, and its normal n = (−∂z/∂x, −∂z/∂y, 1) / ‖·‖
        z_ground = terrain.height(P[:, 0], P[:, 1], 0.0, terrain_params)
        dz_dx, dz_dy = terrain.gradient(P[:, 0], P[:, 1], 0.0, terrain_params)
        norm = xp.sqrt(dz_dx**2 + dz_dy**2 + 1.0)
        n = xp.stack([-dz_dx / norm, -dz_dy / norm, 1.0 / norm], axis=1)

        # The gap along the normal, d = n_z (P_z − z_ground), and the contact C = P − d n
        d = n[:, 2] * (P[:, 2] - z_ground)
        C = P - d[:, None] * n

        # The wheel frames: the body's x axis on the ground plane, turned by δ at the
        # front; e_y = n × e_x
        x_body = R[:, 0]
        e_flat = x_body - xp.sum(n * x_body, axis=1)[:, None] * n
        e_flat = e_flat / xp.sqrt(xp.sum(e_flat**2, axis=1))[:, None]
        steer = xp.array([1.0, 1.0, 0.0, 0.0]) * delta
        n_cross_e = cross(n, e_flat)
        e_x = xp.cos(steer)[:, None] * e_flat + xp.sin(steer)[:, None] * n_cross_e
        e_y = cross(n, e_x)

        geometry = {"P": P, "n": n, "d": d, "C": C, "R": R, "e_x": e_x, "e_y": e_y}

        return geometry

    def contact_forces(self, x, params=None):
        """Each tire's force at its contact, in the world, and what it rests on.

        Returns a dict of arrays over the wheels: ``F`` (the forces, shape
        ``(4, 3)``), ``arm`` (the contacts from the centre of gravity), ``Fz`` (the
        normal loads), ``Fx``, ``Fy`` (the tire forces in the wheel frames),
        ``kappa``, ``alpha`` (the slips), ``vx``, ``vy`` (the contact velocities in
        the wheel frames), ``grip`` (the used friction), ``e_x``, ``e_y`` (the wheel
        frames' axes) and the corner geometry.
        """
        params = self.params if params is None else params
        xp = array_module(x)

        terrain = self.terrain
        terrain_params = params["terrain"]
        mass = params["mass"]
        gravity = params["gravity"]
        a = params["a"]
        b = params["b"]
        h_cg = params["h_cg"]
        r_f = params["r_f"]
        r_r = params["r_r"]
        road_friction = params["road_friction"]
        C_rr = params["C_rr"]
        v_c = params["v_c"]

        X = x[0]
        Y = x[1]
        z = x[9]
        psi = x[2]
        vx = x[3]
        vy = x[4]
        r = x[5]
        w_rear = x[6]
        phi = x[10]
        theta = x[11]
        vz = x[12]
        p = x[13]
        q = x[14]
        corners = self.corner_geometry(x, params)
        R = corners["R"]
        n = corners["n"]
        d = corners["d"]
        C = corners["C"]
        P = corners["P"]
        p_cg = xp.array([X, Y, z])
        Fz0 = static_loads(mass, gravity, a, b)

        # The body's velocity in the world: v = Rz(ψ) [vx, vy, 0] + [0, 0, vz], and
        # its angular velocity ω_world = R E [p, q, r]
        c_psi = xp.cos(psi)
        s_psi = xp.sin(psi)
        v_cg = xp.array([c_psi * vx - s_psi * vy, s_psi * vx + c_psi * vy, vz])
        omega_world = R @ (euler_rate_map(phi, theta) @ xp.array([p, q, r]))

        # Each attachment's velocity Ṗ = v + ω × (P − p_cg), and the gap rate ḋ = n · Ṗ
        P_dot = v_cg + cross(omega_world, P - p_cg)
        d_dot = xp.sum(n * P_dot, axis=1)

        e_x = corners["e_x"]
        e_y = corners["e_y"]

        # Each contact's velocity v_C = v + ω × (C − p_cg), in its wheel frame
        arm = C - p_cg
        v_C = v_cg + cross(omega_world, arm)
        vx_w = xp.sum(v_C * e_x, axis=1)
        vy_w = xp.sum(v_C * e_y, axis=1)

        # The road's grip under each contact
        friction = road_friction * terrain.friction(
            C[:, 0], C[:, 1], 0.0, terrain_params
        )

        # Each wheel: its spring's load, its slips, its tire's forces
        Fz = []
        Fx = []
        Fy = []
        kappa = []
        alpha = []
        grip = []
        for i, wheel in enumerate(self.wheels):
            spring = self.suspension[wheel]
            tire = self.tire[wheel]
            p_spring = params["suspension"][wheel]
            p_tire = params["tire"][wheel]
            front = i < 2
            R_wheel = r_f if front else r_r

            # The front wheels roll free (κ = 0), the rear ones turn at w_rear
            w_wheel = vx_w[i] / R_wheel if front else w_rear
            Fz_i = spring.load(d[i], d_dot[i], Fz0[wheel], h_cg, p_spring)
            kappa_i, alpha_i = tire.slip(vx_w[i], vy_w[i], w_wheel, R_wheel, p_tire)
            Fx_i, Fy_i = tire.forces(kappa_i, alpha_i, Fz_i, friction[i], p_tire)

            # Rolling resistance of the free-rolling front wheels: −C_rr Fz sign(vx)
            if front:
                Fx_i = Fx_i - C_rr * Fz_i * smooth_sign(vx_w[i], v_c)
            grip_i = tire.friction_use(Fx_i, Fy_i, Fz_i, friction[i], p_tire)

            Fz.append(Fz_i)
            Fx.append(Fx_i)
            Fy.append(Fy_i)
            kappa.append(kappa_i)
            alpha.append(alpha_i)
            grip.append(grip_i)
        Fz = xp.stack(Fz)
        Fx = xp.stack(Fx)
        Fy = xp.stack(Fy)

        # The force at each contact: F = Fx e_x + Fy e_y + Fz n
        F = Fx[:, None] * e_x + Fy[:, None] * e_y + Fz[:, None] * n

        forces = {
            "F": F,
            "arm": arm,
            "Fz": Fz,
            "Fx": Fx,
            "Fy": Fy,
            "kappa": xp.stack(kappa),
            "alpha": xp.stack(alpha),
            "vx": vx_w,
            "vy": vy_w,
            "v_C": v_C,
            "grip": xp.stack(grip),
            **corners,
        }

        return forces

    # Sensor outputs

    def slips(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        corners = self.contact_forces(x, params)

        slip = xp.concatenate([corners["kappa"], corners["alpha"]])

        return slip

    def grip(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        corners = self.contact_forces(x, params)

        return corners["grip"]

    def loads(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        corners = self.contact_forces(x, params)

        return corners["Fz"]

    def rollover(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        corners = self.contact_forces(x, params)

        # The margin to rollover: the smallest wheel load, and the roll angle
        Fz_min = xp.min(corners["Fz"])
        phi = x[10]
        rollover = xp.array([Fz_min, phi])

        return rollover

    def imu(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        mass = params["mass"]
        rho = params["rho"]
        CdA = params["CdA"]

        psi = x[2]
        vx = x[3]
        r = x[5]
        phi = x[10]
        theta = x[11]
        p = x[13]
        q = x[14]
        corners = self.contact_forces(x, params)
        R = corners["R"]

        # The specific force: the contact and aero forces over the mass, in the body
        # frame (what an accelerometer reads; gravity is not a contact force)
        F_aero = 0.5 * rho * CdA * vx * xp.abs(vx)
        aero_world = -F_aero * xp.array([xp.cos(psi), xp.sin(psi), 0.0])
        specific = R.T @ ((corners["F"].sum(axis=0) + aero_world) / mass)
        omega = euler_rate_map(phi, theta) @ xp.array([p, q, r])
        imu = xp.concatenate([specific, omega])

        return imu

    def tf(self, x, u, t=0, params=None):
        """The body's full pose, and each wheel's frame at its hub, steered and lifted.

        A wheel frame sits at its hub, ``R`` above the contact along the terrain's
        normal: ``x`` along the wheel's heading, ``y`` along its axle, ``z`` along the
        normal; it turns with its rolling angle when the angles are states.
        """
        params = self.params if params is None else params

        wheels = self.wheels
        wheel_angles = self.wheel_angles
        r_f = params["r_f"]
        r_r = params["r_r"]

        corners = self.corner_geometry(np.asarray(x, dtype=float), params)
        R = np.asarray(corners["R"])
        n = np.asarray(corners["n"])
        C = np.asarray(corners["C"])
        e_x = np.asarray(corners["e_x"])
        e_y = np.asarray(corners["e_y"])
        spin = [x[16], x[16], x[15], x[15]] if wheel_angles else [0.0] * 4

        frames = {"body": SE3(R, np.array([x[0], x[1], x[9]]))}
        for i, wheel in enumerate(wheels):
            radius = r_f if i < 2 else r_r
            R_wheel = np.column_stack([e_x[i], e_y[i], n[i]]) @ Ry(spin[i])
            frames[f"wheel_{wheel}"] = SE3(R_wheel, C[i] + radius * n[i])

        # The planar frame of every car rung: the front axle, steered, on the ground
        # between its two contacts
        R_axle = np.column_stack([e_x[0], e_y[0], n[0]])
        frames["axle_front"] = SE3(R_axle, 0.5 * (C[0] + C[1]))

        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """Each tire's contact velocity (blue) and force (red), when shown."""
        if not self.show_contact_vectors:
            return {}

        params = self.params if params is None else params
        mass = params["mass"]
        gravity = params["gravity"]

        corners = self.contact_forces(np.asarray(x, dtype=float), params)
        C = np.asarray(corners["C"])
        F = np.asarray(corners["F"])
        v_C = np.asarray(corners["v_C"])

        # Forces at a metre per weight of the car, velocities at a tenth of a second
        arrows = []
        for i in range(4):
            arrows.append(
                Arrow(base=C[i], vector=v_C[i], scale=0.1, color="blue", linewidth=2)
            )
            arrows.append(
                Arrow(
                    base=C[i],
                    vector=F[i],
                    scale=1.0 / (mass * gravity),
                    color="red",
                    linewidth=2,
                )
            )

        return {"world": arrows}

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def param_parts(self) -> dict:
        """The parts whose own numbers the tree holds: tires, springs and the terrain."""
        return {
            "tire": dict(self.tire),
            "suspension": dict(self.suspension),
            "terrain": self.terrain,
        }


class FormulaCar3D(SuspensionCar3D):
    """A generic open-wheel formula car: light, low, stiff, powerful, on grippy slicks.

    The 3-D car with a racer's numbers and its look
    (:func:`~minilink.graphical.catalog.car_skins_3d.formula_car_skin_3d`): 800 kg on
    a 3.5 m wheelbase, the centre of gravity 0.30 m high, 600 kW, slicks of grip 1.6
    and a stiff suspension (a heave mode near 3.5 Hz). No aerodynamic downforce. A
    plausible cartoon, not a model of any real car; a ``tire`` or ``suspension``
    given replaces the preset's, its own numbers included.
    """

    def __init__(
        self,
        named_ports=False,
        tire=None,
        suspension=None,
        terrain=None,
        wheel_angles=False,
    ):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.car_skins_3d import formula_car_skin_3d

        # The racer's parts: slicks and stiff springs
        if tire is None:
            tire = BrushTire(mu=1.6, c_alpha=18.0, c_kappa=22.0, v_eps=0.5)
        if suspension is None:
            suspension = SpringDamper(k=96700.0, c=3500.0, F_eps=50.0)
        super().__init__(
            named_ports=named_ports,
            tire=tire,
            suspension=suspension,
            terrain=terrain,
            wheel_angles=wheel_angles,
        )

        # Parameters: the racer's numbers
        self.params.update(
            {
                "mass": 800.0,  # [kg] with its driver
                "inertia": 1100.0,  # [kg m^2] yaw
                "Ixx": 250.0,  # [kg m^2] roll
                "Iyy": 1000.0,  # [kg m^2] pitch
                "a": 1.9,  # [m]
                "b": 1.6,  # [m]
                "track": 1.6,  # [m]
                "h_cg": 0.30,  # [m]
                "r_f": 0.33,  # [m]
                "r_r": 0.34,  # [m]
                "CdA": 1.2,  # [m^2]
                "C_rr": 0.012,  # [-]
                "P_max": 600000.0,  # [W]
                "tau_sat": 5000.0,  # [N m]
                "engine_tau": 0.05,  # [s]
                "Jw_rear": 3.0,  # [kg m^2]
                "bw_drive": 0.2,  # [N m s]
                "tau_fric": 20.0,  # [N m]
                "delta_max": 0.35,  # [rad]
                "steering_tau": 0.05,  # [s]
                "steer_rate_max": 3.0,  # [rad/s]
            }
        )
        self.x0[9] = self.params["h_cg"]

        # Ports: the racer's command limits
        self.bound_commands()

        # Graphics: the racer's look
        self.skin = formula_car_skin_3d
        self.camera_scale = 9.0


class OffRoadCar3D(SuspensionCar3D):
    """A generic off-road side-by-side: tall, short, softly sprung, on big knobby tires.

    The 3-D car with a utility buggy's numbers and its look
    (:func:`~minilink.graphical.catalog.car_skins_3d.offroad_car_skin_3d`): 800 kg on
    a 2.4 m wheelbase, the centre of gravity 0.65 m high (a car that can roll over),
    130 kW, tires of grip 0.9 and a soft, long-travel suspension (a heave mode near
    1.3 Hz). A plausible cartoon, not a model of any real vehicle; a ``tire`` or
    ``suspension`` given replaces the preset's, its own numbers included.
    """

    def __init__(
        self,
        named_ports=False,
        tire=None,
        suspension=None,
        terrain=None,
        wheel_angles=False,
    ):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.car_skins_3d import offroad_car_skin_3d

        # The buggy's parts: knobby tires and soft springs
        if tire is None:
            tire = BrushTire(mu=0.9, c_alpha=8.0, c_kappa=10.0, v_eps=0.5)
        if suspension is None:
            suspension = SpringDamper(k=13350.0, c=1150.0, F_eps=50.0)
        super().__init__(
            named_ports=named_ports,
            tire=tire,
            suspension=suspension,
            terrain=terrain,
            wheel_angles=wheel_angles,
        )

        # Parameters: the buggy's numbers
        self.params.update(
            {
                "mass": 800.0,  # [kg] with its driver
                "inertia": 900.0,  # [kg m^2] yaw
                "Ixx": 300.0,  # [kg m^2] roll
                "Iyy": 800.0,  # [kg m^2] pitch
                "a": 1.15,  # [m]
                "b": 1.25,  # [m]
                "track": 1.6,  # [m]
                "h_cg": 0.65,  # [m]
                "r_f": 0.37,  # [m]
                "r_r": 0.37,  # [m]
                "CdA": 1.6,  # [m^2]
                "C_rr": 0.03,  # [-] rolling on dirt
                "P_max": 130000.0,  # [W]
                "tau_sat": 3000.0,  # [N m]
                "engine_tau": 0.15,  # [s]
                "Jw_rear": 2.5,  # [kg m^2]
                "bw_drive": 0.3,  # [N m s]
                "tau_fric": 15.0,  # [N m]
                "delta_max": 0.6,  # [rad]
                "steering_tau": 0.08,  # [s]
                "steer_rate_max": 2.0,  # [rad/s]
            }
        )
        self.x0[9] = self.params["h_cg"]

        # Ports: the buggy's command limits
        self.bound_commands()

        # Graphics: the buggy's look
        self.skin = offroad_car_skin_3d
        self.camera_scale = 7.0


class SpringDamper:
    """A corner's spring and damper, preloaded to carry its static load at ride height.

    The load along the terrain's normal is ``Fz = Fz0 + k (h - d) - c d_dot``,
    floored smoothly at zero (the wheel lifts off): ``d`` the gap from the attachment
    to the ground, ``h`` the ride height at which the spring carries the static load
    ``Fz0``.
    """

    params = own_params()

    def __init__(self, k=33000.0, c=2100.0, F_eps=50.0):
        # Parameters: the stiffness [N/m], the damping [N s/m] and the width of the
        # lift-off's smooth corner [N]
        self.params = {"k": k, "c": c, "F_eps": F_eps}

    def load(self, d, d_dot, Fz0, h, params=None):
        """Normal load ``Fz`` [N] of the corner at gap ``d`` [m] closing at ``-d_dot`` [m/s]."""
        params = self.params if params is None else params

        k = params["k"]
        c = params["c"]
        F_eps = params["F_eps"]

        # Fz = Fz0 + k (h − d) − c ḋ, floored smoothly at zero
        F_spring = Fz0 + k * (h - d) - c * d_dot
        Fz = smooth_floor(F_spring, F_eps)

        return Fz


def attitude(phi, theta, psi):
    """Body-to-world rotation ``R = Rz(psi) Ry(theta) Rx(phi)``."""
    R = Rz(psi) @ Ry(theta) @ Rx(phi)

    return R


def euler_rate_map(phi, theta):
    """``E``: the body rates ``omega = E [p, q, r]`` from the roll, pitch and yaw rates."""
    xp = array_module(phi, theta)

    c_phi = xp.cos(phi)
    s_phi = xp.sin(phi)
    c_theta = xp.cos(theta)
    s_theta = xp.sin(theta)
    zero = xp.zeros_like(c_phi)
    one = xp.ones_like(c_phi)

    # ω₁ = p − r sin θ, ω₂ = q cos φ + r sin φ cos θ, ω₃ = −q sin φ + r cos φ cos θ
    # fmt: off
    E = xp.array([
        [ one,   zero,          -s_theta],
        [zero,  c_phi,  s_phi * c_theta],
        [zero, -s_phi,  c_phi * c_theta],
    ])
    # fmt: on

    return E


def euler_rate_map_derivative(phi, theta, p, q, r):
    """``Ė [p, q, r]``: the part of ``omega_dot`` from the turning of the map ``E``."""
    xp = array_module(phi, theta, p, q, r)

    c_phi = xp.cos(phi)
    s_phi = xp.sin(phi)
    c_theta = xp.cos(theta)
    s_theta = xp.sin(theta)

    # d/dt of each row of E, times [p, q, r], with φ̇ = p and θ̇ = q
    e1 = -c_theta * q * r
    e2 = -s_phi * p * q + (c_phi * c_theta * p - s_phi * s_theta * q) * r
    e3 = -c_phi * p * q - (s_phi * c_theta * p + c_phi * s_theta * q) * r
    dE_rates = xp.array([e1, e2, e3])

    return dE_rates


def static_loads(mass, gravity, a, b):
    """Each corner's share of the weight at rest [N]: ``m g b / 2L`` front, ``m g a / 2L`` rear."""
    L = a + b
    Fz0_front = 0.5 * mass * gravity * b / L
    Fz0_rear = 0.5 * mass * gravity * a / L
    Fz0 = {"fl": Fz0_front, "fr": Fz0_front, "rl": Fz0_rear, "rr": Fz0_rear}

    return Fz0


def cross(a, b):
    """``a × b`` along the last axis, for 3-vectors or stacks of them."""
    xp = array_module(a, b)

    # (a × b) = (a₂ b₃ − a₃ b₂, a₃ b₁ − a₁ b₃, a₁ b₂ − a₂ b₁)
    c1 = a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1]
    c2 = a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2]
    c3 = a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    c = xp.stack([c1, c2, c3], axis=-1)

    return c
