"""1/10-scale racecar: power-limited rear drive, brush tires, servo steering.

The drive is described by the power it can put on the ground and by the torque the
drivetrain can hold at low speed, never by a motor circuit.

State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta, P]``, inputs ``P_cmd`` [W] and
``delta_cmd`` [rad]. Both equation paths use the ``xp`` idiom, so the class runs on NumPy
and traces under JAX.
"""

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


def _frames(X, Y, psi, delta, params, phi_rear=0.0, phi_front=None):
    """Link poses for either racecar plant. Import is lazy so ``f`` stays mesh-free."""
    from minilink.graphical.catalog.racecar_skin import racecar_frames

    b = params["b"]
    return racecar_frames(
        X,
        Y,
        psi,
        delta,
        phi_rear=phi_rear,
        phi_front=phi_front,
        ref_to_rear_axle=b,
        wheelbase=params["a"] + b,
        wheel_radius=params["r_r"],
    )


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
        a = PUBLIC_RACECAR_PARAMS["a"]
        b = PUBLIC_RACECAR_PARAMS["b"]
        r = PUBLIC_RACECAR_PARAMS["r_r"]
        self.a = a
        self.b = b
        self.params["a"] = a
        self.params["b"] = b
        self.params["length"] = a + b
        self.params["r_r"] = r
        self.wheel_len = 2.0 * r
        self.wheel_width = 0.045
        self.wheel_radius = r
        self.camera_follow_frame = "body"
        self.camera_scale = 2.0 * (a + b)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return _frames(x[0], x[1], x[2], u[1], params)


class UdeSRacecarDyn(DynamicBicycle):
    """1/10-scale racecar: power-limited rear drive, brush tires, servo steering.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta, P]``: world pose, body
    velocities, rear wheel rate [rad/s], steer angle [rad] and the power on its way to
    the ground [W].

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
        self.n = 9
        self.state = VectorSignal("x", dim=self.n)
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
        self.x0 = np.zeros(self.n)
        self.params = dict(PUBLIC_RACECAR_PARAMS)

        P_max, delta_max = self.params["P_max"], self.params["delta_max"]
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
        else:  # one stacked command port (planning / trajectory-optimization convention)
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
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
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

        # graphics: 1/10-scale car; ``tf`` publishes the link frames so
        # ``car.skin = racecar_skin_3d`` attaches the 3-D look (wheels spin
        # only on :class:`UdeSRacecarDyn3D`, which carries the rolling angles)
        self.wheel_len = 2.0 * self.params["r_r"]
        self.wheel_width = 0.045
        self.wheel_radius = self.params["r_r"]
        self.track = 0.20
        self.skin = partial(car_skin_2d, color="#1a1a1a")
        self.camera_follow_frame = "body"
        self.camera_scale = 1.5

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
        r_r, Jw_rear = params["r_r"], params["Jw_rear"]
        c_kappa, v_eps = params["c_kappa"], params["v_min_epsilon"]
        c_alpha_f, c_alpha_r = params["c_alpha_f"], params["c_alpha_r"]
        mass, gravity = params["mass"], params["gravity"]
        inertia, a, b = params["inertia"], params["a"], params["b"]

        Fz_f = mass * gravity * b / (a + b)
        Fz_r = mass * gravity * a / (a + b)
        C_kappa, C_alpha_f, C_alpha_r = (
            c_kappa * Fz_r,
            c_alpha_f * Fz_f,
            c_alpha_r * Fz_r,
        )

        # wheel and car exchanging momentum through the rear contact; yaw and sideslip
        # against the two cornering stiffnesses
        tau_long = v_eps / (C_kappa * (r_r**2 / Jw_rear + 1.0 / mass))
        tau_yaw = inertia * v_eps / (C_alpha_f * a**2 + C_alpha_r * b**2)
        tau_lat = mass * v_eps / (C_alpha_f + C_alpha_r)

        self.solver_info["smallest_time_constant"] = 0.5 * min(
            tau_long, tau_yaw, tau_lat, params["engine_tau"], params["steering_tau"]
        )

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        P_max, engine_tau = params["P_max"], params["engine_tau"]
        tau_sat, w_eps = params["tau_sat"], params["w_min_epsilon"]
        Jw_rear, bw_drive = params["Jw_rear"], params["bw_drive"]
        tau_fric, w_c = params["tau_fric"], params["w_c"]
        delta_max, steering_tau = params["delta_max"], params["steering_tau"]
        rate_max = params["steer_rate_max"]
        xp = array_module(x, u)
        q, v, u_in = x[0:3], x[3:6], x[6:8]
        w_rear, delta, P = x[6], x[7], x[8]
        P_cmd, delta_cmd = self.commands(x, u)

        # chassis: the inherited rigid-body equations, fed by the brush tires
        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)
        dv = xp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        # drive: lagged power, a torque limit at low speed, and the drivetrain brake
        P_dot = (xp.clip(P_cmd, -P_max, P_max) - P) / engine_tau
        tau = xp.clip(P / xp.sqrt(w_rear**2 + w_eps**2), -tau_sat, tau_sat)
        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        tau_brake = bw_drive * w_rear + tau_fric * xp.tanh(w_rear / w_c)
        w_rear_dot = (tau - tau_ground - tau_brake) / Jw_rear

        # servo: first-order lag with a smooth rate limit, held by the end stops
        delta_ref = xp.clip(delta_cmd, -delta_max, delta_max)
        delta_dot = rate_max * xp.tanh((delta_ref - delta) / (steering_tau * rate_max))
        delta_dot = xp.where(delta > delta_max, xp.minimum(delta_dot, 0.0), delta_dot)
        delta_dot = xp.where(delta < -delta_max, xp.maximum(delta_dot, 0.0), delta_dot)

        dx = xp.concatenate([dq, dv, xp.array([w_rear_dot, delta_dot, P_dot])])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def commands(self, x, u):
        """``[P_cmd, delta_cmd]`` from the port vector."""
        xp = array_module(x, u)
        if not self.named_ports:
            return xp.asarray(u)
        P_cmd, delta_cmd = self.get_port_values_from_u(u, "P_cmd", "delta_cmd")

        return xp.array([P_cmd[0], delta_cmd[0]])

    # Sensor outputs

    def speed(self, x, u, t=0.0, params=None):
        return x[3:4]

    def slips(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        vx, vy, yaw_rate = x[3], x[4], x[5]
        w_rear, delta = x[6], x[7]

        kappa_r, alpha_f, alpha_r = axle_slips(vx, vy, yaw_rate, w_rear, delta, params)

        return xp.array([kappa_r, alpha_f, alpha_r])

    def grip(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(x[3:6], x[6:8], params)

        phi_f, phi_r = friction_use(Fx_f, Fy_f, Fx_r, Fy_r, params)

        return xp.array([phi_f, phi_r])

    def imu(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        mass, rho, CdA = params["mass"], params["rho"], params["CdA"]
        xp = array_module(x)
        vx, yaw_rate, delta = x[3], x[5], x[7]
        Fx_f, Fy_f, Fx_r, Fy_r = self.compute_tire_physics(x[3:6], x[6:8], params)

        # specific force at the centre of gravity: contact and aero forces over the mass
        c_d, s_d = xp.cos(delta), xp.sin(delta)
        F_aero = 0.5 * rho * CdA * vx * xp.abs(vx)
        a_x = (Fx_r + Fx_f * c_d - Fy_f * s_d - F_aero) / mass
        a_y = (Fy_r + Fx_f * s_d + Fy_f * c_d) / mass

        return xp.array([a_x, a_y, yaw_rate])

    def power(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        tau_sat, w_eps = params["tau_sat"], params["w_min_epsilon"]
        xp = array_module(x)
        w_rear, P = x[6], x[8]

        # under the torque limit the wheel receives less than the lagged command power
        tau = xp.clip(P / xp.sqrt(w_rear**2 + w_eps**2), -tau_sat, tau_sat)
        P_wheel = tau * w_rear

        return xp.array([P, P_wheel])

    # Tires (the only physics overridden from DynamicBicycle)

    def compute_tire_physics(self, v_body, u_inputs, params=None):
        """Brush-tire axle forces, replacing the linear-slip pair of the base class."""
        params = self.params if params is None else params
        vx, vy, yaw_rate = v_body[0], v_body[1], v_body[2]
        w_rear, delta = u_inputs[0], u_inputs[1]

        kappa_r, alpha_f, alpha_r = axle_slips(vx, vy, yaw_rate, w_rear, delta, params)
        Fx_f, Fy_f, Fx_r, Fy_r = brush_tire_forces(
            kappa_r, alpha_f, alpha_r, vx, params
        )

        return Fx_f, Fy_f, Fx_r, Fy_r

    def _u_in(self, x, u):
        """The base class reads ``[w_rear, delta]`` here; both are states now."""
        return x[6:8]

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return _frames(x[0], x[1], x[2], x[7], params)


class UdeSRacecarDyn3D(UdeSRacecarDyn):
    """Same car with the two wheel rolling angles as states, for the 3-D skin.

    State ``x = [..., P, phi_rear, phi_front]`` (``n = 11``). An animation frame is a
    pure function of ``(x, u, t)``, so a wheel that turns needs its angle in the state;
    the two extra states are plain integrators and change nothing else.
    """

    def __init__(self):
        super().__init__()

        # the skin lives beside this module; importing it here keeps ``f`` mesh-free
        from minilink.graphical.catalog.racecar_skin import racecar_skin_3d

        self.name = "UdeS Racecar (3D)"
        self.n = 11
        self.state = VectorSignal("x", dim=self.n)
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
        self.x0 = np.zeros(self.n)

        self.outputs["y"].dim = self.n
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        self.skin = racecar_skin_3d
        self.wheel_radius = self.params["r_r"]

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        a, r_f = params["a"], params["r_f"]
        xp = array_module(x, u)
        vx, vy, yaw_rate = x[3], x[4], x[5]
        w_rear, delta = x[6], x[7]

        dx_car = super().f(x[0:9], u, t, params)

        # wheel angles: the rear turns at its own rate, the front rolls with its contact
        c_d, s_d = xp.cos(delta), xp.sin(delta)
        vx_f = vx * c_d + (vy + a * yaw_rate) * s_d
        dphi_rear = w_rear
        dphi_front = vx_f / r_f

        dx = xp.concatenate([dx_car, xp.array([dphi_rear, dphi_front])])

        return dx

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return _frames(x[0], x[1], x[2], x[7], params, x[9], x[10])

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {}


# Public functions


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
