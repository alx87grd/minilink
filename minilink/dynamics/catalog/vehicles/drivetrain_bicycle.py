"""Drivetrain bicycle: power-limited rear drive, a tire law, servo steering."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.signals import VectorSignal
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.dynamics.catalog.vehicles.tires import (
    BrushTire,
    axle_slips,
    friction_use,
    smooth_sign,
)

###############################################################################
# Public API
###############################################################################


class DrivetrainBicycle(DynamicBicycle):
    """Dynamic bicycle driven by a power-limited rear axle and a steering servo.

    State ``x = [x, y, psi, vx, vy, yaw_rate, w_rear, delta, P]``: world pose, body
    velocities, rear wheel rate [rad/s], steer angle [rad] and the power on its way to
    the ground [W]. With ``wheel_angles=True`` the rear and front rolling angles
    ``phi_rear, phi_front`` [rad] follow, for a look whose wheels turn. The equation
    paths use the ``xp`` idiom and trace under JAX.

    Inputs
    ------
    P_cmd : requested drive power [W], clipped to ``+- P_max`` (negative brakes).
    delta_cmd : requested steer angle [rad], clipped to ``+- delta_max``.

    ``named_ports=False`` stacks the two commands into one ``u`` port instead — the
    planning and trajectory-optimization convention of the minilink vehicle ladder.

    Tires
    -----
    ``tire`` is the law of both axles, :class:`~minilink.dynamics.catalog.vehicles.tires.BrushTire`
    by default; its own numbers join ``params`` at construction, while ``mu`` and the
    stiffnesses per unit load (``c_kappa``, ``c_alpha_f``, ``c_alpha_r``) stay the car's.

    Propulsion
    ----------
    The drive is described by what it can deliver::

        P_dot = (clip(P_cmd, +-P_max) - P) / engine_tau
        tau   = clip(P / sqrt(w_rear^2 + w_min_epsilon^2), +-tau_sat)
        Jw_rear w_rear_dot = tau - tau_ground(v, w_rear, delta) - tau_brake(w_rear)

    with ``tau_brake = bw_drive w_rear + tau_fric tanh(w_rear / w_c)``. The smooth
    denominator keeps the torque's sign tied to the power's and stays finite at rest,
    where ``tau_sat`` is what limits the launch. Asking for power the rear tire cannot
    put down spins the rear wheel instead of accelerating the car.

    Steering
    --------
    The servo is a first-order lag with a smooth rate limit and end stops::

        delta_dot = steer_rate_max tanh((delta_ref - delta) / (steering_tau steer_rate_max))

    Outputs
    -------
    ``y`` (the full state), ``speed`` [m/s], ``slip`` (``kappa_r``, ``alpha_f``,
    ``alpha_r``), ``grip`` (used fraction of each friction ellipse), ``imu``
    (specific force at the centre of gravity and yaw rate) and ``power``
    (the lagged power and the power actually reaching the wheel).

    Notes
    -----
    The default numbers are the full-size car of :class:`DynamicBicycle` (its mass,
    inertia, geometry and drag, its ``Ca`` and ``Ck`` over the static loads), with a
    100 kW drive and a road-car steering servo.
    """

    def __init__(self, named_ports=True, tire=None, wheel_angles=False):
        super().__init__(named_ports=True)
        self.named_ports = bool(named_ports)
        self.wheel_angles = bool(wheel_angles)
        self.tire = BrushTire() if tire is None else tire

        # State: the body's six, the rear wheel rate, the steer angle and the power,
        # then the two rolling angles when they are states
        self.n = 11 if self.wheel_angles else 9
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        # Parameters: the full-size car of the dynamic bicycle, then the tire law's own
        car = {
            # Chassis
            "mass": 1500.0,  # [kg]
            "inertia": 2500.0,  # [kg m^2]
            "a": 1.0,  # [m]    CG to front axle
            "b": 1.0,  # [m]    CG to rear axle
            "r_f": 0.3,  # [m]    front tire radius
            "r_r": 0.3,  # [m]    rear tire radius
            "gravity": 9.81,  # [m/s^2]
            # Aero and rolling resistance
            "rho": 1.225,  # [kg/m^3]
            "CdA": 0.3 * 2.2,  # [m^2]
            "C_rr": 0.015,  # [-]   rolling resistance of a road tire
            # Tires (stiffnesses per unit load: Ca and Ck over the static loads)
            "mu": 1.0,  # [-]
            "c_alpha_f": 60000.0 / (1500.0 * 9.81 * 0.5),  # [1/rad]
            "c_alpha_r": 60000.0 / (1500.0 * 9.81 * 0.5),  # [1/rad]
            "c_kappa": 100000.0 / (1500.0 * 9.81 * 0.5),  # [-]
            # Propulsion
            "P_max": 100000.0,  # [W]
            "tau_sat": 2500.0,  # [N m] torque the drivetrain holds at the rear axle
            "engine_tau": 0.2,  # [s]   lag from the command to the power
            "Jw_rear": 1.6,  # [kg m^2] rear wheels plus the drivetrain
            "bw_drive": 0.1,  # [N m s] viscous drag of the drivetrain
            "bw_rear": 0.0,  # [N m s] the base class's own wheel drag: f() adds bw_drive
            "tau_fric": 10.0,  # [N m] dry friction of the drivetrain
            # Steering
            "delta_max": 0.6,  # [rad] road-wheel lock, about 34 deg
            "steering_tau": 0.1,  # [s]
            "steer_rate_max": 1.0,  # [rad/s]
            # Smoothing scales: they keep the equations finite and differentiable at rest
            "v_min_epsilon": 0.1,  # [m/s]   floor of the slip denominators
            "w_min_epsilon": 1.0,  # [rad/s] floor of the power-to-torque denominator
            "v_c": 0.1,  # [m/s]   creep speed of the rolling resistance
            "w_c": 1.0,  # [rad/s] creep rate of the drivetrain dry friction
        }
        self.params = {**car, **self.tire.params}

        # The wheel angles, chosen once: two more states, or none
        if self.wheel_angles:
            self.wheel_angle_rates = self.rolling_angle_rates
        else:
            self.wheel_angle_rates = self.no_wheel_angle_rates

        # Ports: the two commands within their limits, as two named ports or one
        # stacked port (the planning / trajectory-optimization convention); the state
        # and the sensors as outputs
        self.inputs = {}
        if self.named_ports:
            self.add_input_port(
                "P_cmd", nominal_value=0.0, labels=["P_cmd"], units=["W"]
            )
            self.add_input_port(
                "delta_cmd", nominal_value=0.0, labels=["delta_cmd"], units=["rad"]
            )
        else:
            self.add_input_port(
                "u",
                dim=2,
                nominal_value=np.zeros(2),
                labels=["P_cmd", "delta_cmd"],
                units=["W", "rad"],
            )
        self.bound_commands()
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.add_output_port("speed", function=self.speed, labels=["vx"], units=["m/s"])
        self.add_output_port(
            "slip",
            dim=3,
            function=self.slips,
            labels=["kappa_r", "alpha_f", "alpha_r"],
            units=["-", "rad", "rad"],
        )
        self.add_output_port(
            "grip",
            dim=2,
            function=self.grip,
            labels=["phi_f", "phi_r"],
            units=["-", "-"],
        )
        self.add_output_port(
            "imu",
            dim=3,
            function=self.imu,
            labels=["a_x", "a_y", "yaw_rate"],
            units=["m/s^2", "m/s^2", "rad/s"],
        )
        self.add_output_port(
            "power",
            dim=2,
            function=self.power,
            labels=["P", "P_wheel"],
            units=["W", "W"],
        )

        # Signal labels and units
        labels = ["x", "y", "psi", "vx", "vy", "yaw_rate", "w_rear", "delta", "P"]
        units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad/s", "rad", "W"]
        if self.wheel_angles:
            labels = labels + ["phi_rear", "phi_front"]
            units = units + ["rad", "rad"]
        self.state.labels = labels
        self.state.units = units
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

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
        """The solver hint: an estimate of the plant's fastest mode, in seconds.

        The tire modes are stiffest at rest, where the slip denominators sit on their
        floor ``v_min_epsilon``. Three of them are estimated — the wheel against the
        longitudinal stiffness, the yaw and the sideslip against the cornering
        stiffnesses — and the smallest is halved, because each is written for one
        degree of freedom while the real modes couple them (the longitudinal pair alone
        is some 16 % faster than an uncoupled wheel would be).
        """
        params = self.params

        r_r = params["r_r"]
        Jw_rear = params["Jw_rear"]
        c_kappa = params["c_kappa"]
        v_eps = params["v_min_epsilon"]
        c_alpha_f = params["c_alpha_f"]
        c_alpha_r = params["c_alpha_r"]
        mass = params["mass"]
        gravity = params["gravity"]
        inertia = params["inertia"]
        a = params["a"]
        b = params["b"]
        engine_tau = params["engine_tau"]
        steering_tau = params["steering_tau"]

        # Static axle loads, and the slip stiffnesses per unit load they scale
        Fz_f = mass * gravity * b / (a + b)
        Fz_r = mass * gravity * a / (a + b)
        C_kappa = c_kappa * Fz_r
        C_alpha_f = c_alpha_f * Fz_f
        C_alpha_r = c_alpha_r * Fz_r

        # The wheel and the car exchanging momentum through the rear contact; the yaw
        # and the sideslip against the two cornering stiffnesses
        T_long = v_eps / (C_kappa * (r_r**2 / Jw_rear + 1.0 / mass))
        T_yaw = inertia * v_eps / (C_alpha_f * a**2 + C_alpha_r * b**2)
        T_lat = mass * v_eps / (C_alpha_f + C_alpha_r)

        # Half the smallest of the three estimates and the two actuator lags
        T_min = 0.5 * min(T_long, T_yaw, T_lat, engine_tau, steering_tau)

        return T_min

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        P_max = params["P_max"]
        engine_tau = params["engine_tau"]
        tau_sat = params["tau_sat"]
        w_eps = params["w_min_epsilon"]
        Jw_rear = params["Jw_rear"]
        bw_drive = params["bw_drive"]
        tau_fric = params["tau_fric"]
        w_c = params["w_c"]
        delta_max = params["delta_max"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        q = x[0:3]
        v = x[3:6]
        u_in = x[6:8]
        w_rear = x[6]
        delta = x[7]
        P = x[8]
        P_cmd, delta_cmd = self.commands(x, u)

        # The chassis terms: the inherited rigid-body equations, fed by the tires
        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Chassis: M v̇ + C v + d = 0, and q̇ = N v
        dv = xp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        # Drive: lagged power, a torque limit at low speed, and the drivetrain brake
        #   Ṗ = (clip(P_cmd, ±P_max) − P) / engine_tau
        #   τ = clip(P / √(w_rear² + w_eps²), ±τ_sat)
        #   Jw ẇ_rear = τ − τ_ground(v, w_rear, δ) − τ_brake(w_rear)
        P_dot = (xp.clip(P_cmd, -P_max, P_max) - P) / engine_tau
        tau = xp.clip(P / xp.sqrt(w_rear**2 + w_eps**2), -tau_sat, tau_sat)
        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        tau_brake = bw_drive * w_rear + tau_fric * xp.tanh(w_rear / w_c)
        w_rear_dot = (tau - tau_ground - tau_brake) / Jw_rear

        # Servo: a first-order lag with a smooth rate limit, held by the end stops
        #   δ̇ = rate_max tanh((δ_ref − δ) / (steering_tau rate_max))
        delta_ref = xp.clip(delta_cmd, -delta_max, delta_max)
        delta_dot = rate_max * xp.tanh((delta_ref - delta) / (steering_tau * rate_max))
        delta_dot = xp.where(delta > delta_max, xp.minimum(delta_dot, 0.0), delta_dot)
        delta_dot = xp.where(delta < -delta_max, xp.maximum(delta_dot, 0.0), delta_dot)

        # The rolling angles, when they are states
        dphi = self.wheel_angle_rates(x, params)

        # ẋ = [q̇; v̇; ẇ_rear; δ̇; Ṗ; φ̇]
        dx = xp.concatenate([dq, dv, xp.array([w_rear_dot, delta_dot, P_dot]), dphi])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    # Sensor outputs

    def speed(self, x, u, t=0.0, params=None):
        return x[3:4]

    def slips(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        vx = x[3]
        vy = x[4]
        yaw_rate = x[5]
        w_rear = x[6]
        delta = x[7]

        # The rear slip ratio and the two slip angles
        kappa_r, alpha_f, alpha_r = axle_slips(vx, vy, yaw_rate, w_rear, delta, params)
        slip = xp.array([kappa_r, alpha_f, alpha_r])

        return slip

    def grip(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        v_body = x[3:6]
        u_in = x[6:8]
        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v_body, u_in, params)

        # The used fraction of each axle's friction ellipse
        phi_f, phi_r = friction_use(Fx_f, Fy_f, Fx_r, Fy_r, params)
        grip = xp.array([phi_f, phi_r])

        return grip

    def imu(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        mass = params["mass"]
        rho = params["rho"]
        CdA = params["CdA"]

        vx = x[3]
        yaw_rate = x[5]
        delta = x[7]
        v_body = x[3:6]
        u_in = x[6:8]
        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(v_body, u_in, params)
        c_d = xp.cos(delta)
        s_d = xp.sin(delta)

        # Specific force at the centre of gravity: the contact and aero forces over the
        # mass, with the drag ½ ρ CdA vx |vx|
        F_aero = 0.5 * rho * CdA * vx * xp.abs(vx)
        a_x = (Fx_r + Fx_f * c_d - Fy_f * s_d - F_aero) / mass
        a_y = (Fy_r + Fx_f * s_d + Fy_f * c_d) / mass
        imu = xp.array([a_x, a_y, yaw_rate])

        return imu

    def power(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        tau_sat = params["tau_sat"]
        w_eps = params["w_min_epsilon"]

        w_rear = x[6]
        P = x[8]

        # Under the torque limit the wheel receives less than the lagged command power:
        # τ = clip(P / √(w_rear² + w_eps²), ±τ_sat), and P_wheel = τ w_rear
        tau = xp.clip(P / xp.sqrt(w_rear**2 + w_eps**2), -tau_sat, tau_sat)
        P_wheel = tau * w_rear
        power = xp.array([P, P_wheel])

        return power

    # Tires (the only physics overridden from DynamicBicycle)

    def compute_tire_physics(self, v_body, u_inputs, params=None):
        """Axle forces ``(Fx_f, Fy_f, Fx_r, Fy_r)`` [N], each in its own wheel frame.

        Static axle loads carry the weight. The free-rolling front axle makes a
        cornering force and pays rolling resistance; the driven rear axle shares one
        friction ellipse between its traction and its cornering force.

        ``C_rr`` is charged to the front axle only: what the rear wheels lose to rolling
        is carried by the drivetrain terms (``bw_drive``, ``tau_fric``), which resist
        the wheel rate itself rather than the contact patch.
        """
        params = self.params if params is None else params

        tire = self.tire
        mass = params["mass"]
        gravity = params["gravity"]
        a = params["a"]
        b = params["b"]
        mu = params["mu"]
        c_kappa = params["c_kappa"]
        c_alpha_f = params["c_alpha_f"]
        c_alpha_r = params["c_alpha_r"]
        C_rr = params["C_rr"]
        v_c = params["v_c"]

        vx = v_body[0]
        vy = v_body[1]
        yaw_rate = v_body[2]
        w_rear = u_inputs[0]
        delta = u_inputs[1]

        # Static axle loads
        Fz_f = mass * gravity * b / (a + b)
        Fz_r = mass * gravity * a / (a + b)

        # The front axle rolls free (κ_f = 0); the rear one is driven
        kappa_r, alpha_f, alpha_r = axle_slips(vx, vy, yaw_rate, w_rear, delta, params)
        Fx_f, Fy_f = tire.forces(0.0, alpha_f, Fz_f, mu, c_kappa, c_alpha_f, params)
        Fx_r, Fy_r = tire.forces(kappa_r, alpha_r, Fz_r, mu, c_kappa, c_alpha_r, params)

        # Rolling resistance of the free-rolling front axle: −C_rr Fz_f sign(vx)
        Fx_f = Fx_f - C_rr * Fz_f * smooth_sign(vx, v_c)

        return Fx_f, Fy_f, Fx_r, Fy_r

    def rolling_angle_rates(self, x, params):
        """``[phi_rear_dot, phi_front_dot]``: each wheel turning with its contact."""
        xp = array_module(x)

        a = params["a"]
        r_f = params["r_f"]

        vx = x[3]
        vy = x[4]
        yaw_rate = x[5]
        w_rear = x[6]
        delta = x[7]
        c_d = xp.cos(delta)
        s_d = xp.sin(delta)

        # The rear turns at its own rate, the front rolls with its contact:
        # φ̇_rear = w_rear and φ̇_front = vx_f / r_f
        vx_f = vx * c_d + (vy + a * yaw_rate) * s_d
        dphi_rear = w_rear
        dphi_front = vx_f / r_f
        dphi = xp.array([dphi_rear, dphi_front])

        return dphi

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def commands(self, x, u):
        """``[P_cmd, delta_cmd]`` from the port vector: the two named ports in order."""
        xp = array_module(x, u)

        commands = xp.asarray(u)

        return commands

    def wheel_rate_and_steer(self, x, u):
        """The base class reads ``[w_rear, delta]`` here; both are states now."""
        return x[6:8]

    def no_wheel_angle_rates(self, x, params):
        """No rolling angle in the state: an empty block."""
        xp = array_module(x)

        dphi = xp.zeros(0)

        return dphi

    def bound_commands(self):
        """The command ports' limits, ``+- P_max`` and ``+- delta_max``, from ``params``."""
        P_max = self.params["P_max"]
        delta_max = self.params["delta_max"]

        if self.named_ports:
            self.inputs["P_cmd"].lower_bound = np.array([-P_max])
            self.inputs["P_cmd"].upper_bound = np.array([P_max])
            self.inputs["delta_cmd"].lower_bound = np.array([-delta_max])
            self.inputs["delta_cmd"].upper_bound = np.array([delta_max])
        else:
            self.inputs["u"].lower_bound = np.array([-P_max, -delta_max])
            self.inputs["u"].upper_bound = np.array([P_max, delta_max])
