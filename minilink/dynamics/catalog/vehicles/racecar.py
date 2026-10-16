"""1/10-scale racecar: power-limited rear drive, brush tires, servo steering."""

from functools import partial

import numpy as np

from minilink.core.backends import array_module
from minilink.core.signals import VectorSignal
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.dynamics.catalog.vehicles.steering import KinematicBicycle
from minilink.dynamics.catalog.vehicles.tires import (
    axle_slips,
    brush_tire_forces,
    friction_use,
)
from minilink.graphical.catalog.skins import car_skin_2d

# Geometry and actuators of the 1/10 car.
PUBLIC_RACECAR_PARAMS = {
    # chassis
    "mass": 5.0,  # [kg]
    "inertia": 0.06,  # [kg m^2]
    "a": 0.17,  # [m]    CG to front axle (wheelbase 0.34 m, mass centred)
    "b": 0.17,  # [m]    CG to rear axle
    "r_f": 0.05,  # [m]    front tire radius
    "r_r": 0.05,  # [m]    rear tire radius
    "gravity": 9.81,  # [m/s^2]
    # aero and rolling resistance
    "rho": 1.2,  # [kg/m^3] air at room temperature
    "CdA": 0.03,  # [m^2]  frontal area at Cd ~ 1 for an open 1/10 body
    "C_rr": 0.02,  # [-]   rolling resistance of a rubber tire on a hard floor
    # tires (stiffnesses per unit load)
    "mu": 1.0,  # [-]      soft rubber on a smooth hard floor
    "c_alpha_f": 4.0,  # [1/rad] front cornering stiffness per unit load
    "c_alpha_r": 5.0,  # [1/rad] rear: stiffer, so the car understeers at the limit
    "c_kappa": 10.0,  # [-]  longitudinal stiffness per unit load
    # propulsion (what the drive can deliver, not how it is built)
    "P_max": 80.0,  # [W]
    "tau_sat": 1.5,  # [N m] torque the drivetrain holds at the rear axle at rest
    "engine_tau": 0.1,  # [s] lag from the command to the power on the ground
    "Jw_rear": 0.002,  # [kg m^2] rear wheels plus the drivetrain
    "bw_drive": 0.001,  # [N m s] viscous drag of the drivetrain
    "bw_rear": 0.0,  # [N m s] the base class's own wheel drag: f() adds bw_drive instead
    "tau_fric": 0.02,  # [N m] dry friction of the drivetrain
    # steering
    "delta_max": 0.52,  # [rad] 30 deg
    "steering_tau": 0.08,  # [s] hobby servo closing its own position loop
    "steer_rate_max": 5.0,  # [rad/s] ~0.2 s per 60 deg
    # smoothing scales: they keep the equations finite and differentiable near rest
    "v_min_epsilon": 0.2,  # [m/s]   floor of the slip denominators
    "w_min_epsilon": 1.0,  # [rad/s] floor of the power-to-torque denominator
    "v_c": 0.05,  # [m/s]   creep speed of the rolling resistance
    "w_c": 1.0,  # [rad/s]  creep rate of the drivetrain dry friction
    "z_floor": 1.0e-6,  # [-] keeps d(slip demand)/d(slip) finite at zero slip
}


###############################################################################
# Public API
###############################################################################


class UdeSRacecar(KinematicBicycle):
    """Kinematic bicycle at the 1/10 racecar scale.

    Same equations as :class:`KinematicBicycle`, with ``a = b`` from
    :data:`PUBLIC_RACECAR_PARAMS`. ``tf`` publishes the link frames, so the 3-D
    look attaches with ``car.skin = racecar_skin_3d``. There is no rolling angle
    in the state: the wheels steer, they do not spin.
    """

    def __init__(self):
        super().__init__()
        self.name = "UdeS Racecar"

        # Parameters: the 1/10 car's wheelbase and rear wheel radius
        a = PUBLIC_RACECAR_PARAMS["a"]
        b = PUBLIC_RACECAR_PARAMS["b"]
        r = PUBLIC_RACECAR_PARAMS["r_r"]
        self.a = a
        self.b = b
        self.params["a"] = a
        self.params["b"] = b
        self.params["length"] = a + b
        self.params["r_r"] = r

        # Graphics: 1/10-scale wheels, and a camera that follows the body
        self.wheel_len = 2.0 * r
        self.wheel_width = 0.045
        self.wheel_radius = r
        self.camera_follow_frame = "body"
        self.camera_scale = 2.0 * (a + b)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        return link_frames(x[0], x[1], x[2], u[1], params)


class UdeSRacecarDyn(DynamicBicycle):
    """1/10-scale racecar: power-limited rear drive, brush tires, servo steering.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta, P]``: world pose, body
    velocities, rear wheel rate [rad/s], steer angle [rad] and the power on its way to
    the ground [W]. The equation paths use the ``xp`` idiom and trace under JAX.

    Inputs
    ------
    P_cmd : requested drive power [W], clipped to ``+- P_max`` (negative brakes).
    delta_cmd : requested steer angle [rad], clipped to ``+- delta_max``.

    ``named_ports=False`` stacks the two commands into one ``u`` port instead — the
    planning and trajectory-optimization convention of the minilink vehicle ladder.

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
    """

    def __init__(self, named_ports=True):
        super().__init__(named_ports=True)
        self.name = "UdeS Racecar"
        self.named_ports = bool(named_ports)

        # State: the body's six, the rear wheel rate, the steer angle and the power
        self.n = 9
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        # Parameters
        self.params = dict(PUBLIC_RACECAR_PARAMS)

        # Ports: the two commands within their limits, as two named ports or one
        # stacked port (the planning / trajectory-optimization convention); the state
        # and the sensors as outputs
        P_max = self.params["P_max"]
        delta_max = self.params["delta_max"]
        self.inputs = {}
        if self.named_ports:
            self.add_input_port(
                "P_cmd",
                nominal_value=0.0,
                labels=["P_cmd"],
                units=["W"],
                lower_bound=-P_max,
                upper_bound=P_max,
            )
            self.add_input_port(
                "delta_cmd",
                nominal_value=0.0,
                labels=["delta_cmd"],
                units=["rad"],
                lower_bound=-delta_max,
                upper_bound=delta_max,
            )
        else:
            self.add_input_port(
                "u",
                dim=2,
                nominal_value=np.zeros(2),
                labels=["P_cmd", "delta_cmd"],
                units=["W", "rad"],
                lower_bound=np.array([-P_max, -delta_max]),
                upper_bound=np.array([P_max, delta_max]),
            )
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
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad/s", "rad", "W"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a 1/10-scale car; ``tf`` publishes the link frames, so
        # ``car.skin = racecar_skin_3d`` attaches the 3-D look (the wheels spin only on
        # :class:`UdeSRacecarDyn3D`, which carries the rolling angles)
        self.wheel_len = 2.0 * self.params["r_r"]
        self.wheel_width = 0.045
        self.wheel_radius = self.params["r_r"]
        self.track = 0.20
        self.skin = partial(car_skin_2d, color="#1a1a1a")
        self.camera_follow_frame = "body"
        self.camera_scale = 1.5

        # Solver hint: the plant's fastest mode
        self.refresh()

    def refresh(self):
        """Recompute the solver hint: an estimate of the plant's fastest mode.

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
        self.solver_info["smallest_time_constant"] = T_min

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

        # The chassis terms: the inherited rigid-body equations, fed by the brush tires
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

        # ẋ = [q̇; v̇; ẇ_rear; δ̇; Ṗ]
        dx = xp.concatenate([dq, dv, xp.array([w_rear_dot, delta_dot, P_dot])])

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
        """Brush-tire axle forces, replacing the linear-slip pair of the base class."""
        params = self.params if params is None else params

        vx = v_body[0]
        vy = v_body[1]
        yaw_rate = v_body[2]
        w_rear = u_inputs[0]
        delta = u_inputs[1]

        # The axle slips, then the brush-tire forces they produce
        kappa_r, alpha_f, alpha_r = axle_slips(vx, vy, yaw_rate, w_rear, delta, params)
        Fx_f, Fy_f, Fx_r, Fy_r = brush_tire_forces(
            kappa_r, alpha_f, alpha_r, vx, params
        )

        return Fx_f, Fy_f, Fx_r, Fy_r

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        return link_frames(x[0], x[1], x[2], x[7], params)

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


class UdeSRacecarDyn3D(UdeSRacecarDyn):
    """Same car with the two wheel rolling angles as states, for the 3-D skin.

    State ``x = [..., P, phi_rear, phi_front]`` (``n = 11``). An animation frame is a
    pure function of ``(x, u, t)``, so a wheel that turns needs its angle in the state;
    the two extra states are plain integrators and change nothing else.
    """

    def __init__(self):
        super().__init__()

        # Lazy import: the skin lives beside this module, and importing it here keeps
        # ``f`` mesh-free
        from minilink.graphical.catalog.racecar_skin import racecar_skin_3d

        self.name = "UdeS Racecar (3D)"

        # State: the car's nine, then the rear and front rolling angles
        self.n = 11
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        # Ports: the state output grows with the state
        self.outputs["y"].dim = self.n

        # Signal labels and units
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
            "phi_rear",
            "phi_front",
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
            "rad",
            "rad",
        ]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the 3-D skin, its wheels turned by the rolling angles
        self.skin = racecar_skin_3d
        self.wheel_radius = self.params["r_r"]

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        a = params["a"]
        r_f = params["r_f"]

        vx = x[3]
        vy = x[4]
        yaw_rate = x[5]
        w_rear = x[6]
        delta = x[7]
        dx_car = super().f(x[0:9], u, t, params)
        c_d = xp.cos(delta)
        s_d = xp.sin(delta)

        # Wheel angles: the rear turns at its own rate, the front rolls with its
        # contact, φ̇_rear = w_rear and φ̇_front = vx_f / r_f
        vx_f = vx * c_d + (vy + a * yaw_rate) * s_d
        dphi_rear = w_rear
        dphi_front = vx_f / r_f

        dx = xp.concatenate([dx_car, xp.array([dphi_rear, dphi_front])])

        return dx

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        return link_frames(x[0], x[1], x[2], x[7], params, x[9], x[10])

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


def stiff_tires(params):
    """The same car with rigid tires: the no-slip limit, for tests and comparisons.

    Multiplies every slip stiffness and the friction coefficient so that the axles hold
    their contact conditions, which turns the dynamic bicycle into a kinematic one.
    """
    rigid = dict(params)
    rigid["c_alpha_f"] = 1.0e4
    rigid["c_alpha_r"] = 1.0e4
    rigid["c_kappa"] = 1.0e4
    rigid["mu"] = 1.0e3
    rigid["C_rr"] = 0.0

    return rigid


###############################################################################
# Internal machinery
###############################################################################


def link_frames(X, Y, psi, delta, params, phi_rear=0.0, phi_front=None):
    """Link poses for either racecar plant."""
    # Lazy import: the racecar meshes load on first use, so ``f`` stays mesh-free
    from minilink.graphical.catalog.racecar_skin import racecar_frames

    b = params["b"]
    a = params["a"]
    r_r = params["r_r"]

    wheelbase = a + b

    return racecar_frames(
        X,
        Y,
        psi,
        delta,
        phi_rear=phi_rear,
        phi_front=phi_front,
        ref_to_rear_axle=b,
        wheelbase=wheelbase,
        wheel_radius=r_r,
    )


if __name__ == "__main__":
    from minilink.graphical.catalog.racecar_skin import racecar_skin_3d

    car = UdeSRacecar()  # or UdeSRacecarDyn()
    car.skin = racecar_skin_3d

    car.inputs["u"].nominal_value = np.array([1.0, 0.2])

    car.plot_trajectory()
    car.animate(renderer="meshcat")

    car = UdeSRacecarDyn3D()

    car.inputs["P_cmd"].nominal_value = 150.0
    car.inputs["delta_cmd"].nominal_value = 0.2

    car.compute_trajectory(tf=10.0)

    car.plot_trajectory()
    car.animate(renderer="meshcat")
