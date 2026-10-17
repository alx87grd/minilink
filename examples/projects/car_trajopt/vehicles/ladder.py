"""Vehicle fidelity ladder: JAX points and bicycles, from velocity to engine power."""

from functools import partial

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.kinematics import SE2, translation
from minilink.core.signals import VectorSignal
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import BicycleDynRate
from minilink.graphical.animation.primitives import Arrow, Circle
from minilink.graphical.catalog.skins import car_skin_2d

###############################################################################
# Public API
###############################################################################

# The teaching catalog keeps four rungs: HolonomicMobileRobot → KinematicBicycle /
# KinematicCar → DynamicBicycle (linear tires) → BicycleDynRate (wheel-rate /
# steer-rate inputs, the MPC plant). This module adds the planning-oriented
# variants around them, JAX-first:
#
#   Class              n   u                       Role
#   Holonomic          2   [vx, vy]                point, velocity
#   HolonomicAccel     4   [ax, ay]                point, accel
#   BicycleKin         3   [v, delta]              nonholonomic
#   BicycleAcc         5   [a_x, delta_dot]        no-slip accel / steer
#   BicycleDynTauRate  8   [tau_rear, delta_dot]   torque + steer rate
#   BicycleDynServo    9   [tau_cmd, delta_cmd]    lag on tau and delta
#   BicycleDynEngine   9   [P_cmd, delta_cmd]      power lag + wheel


# Holonomic points


class Holonomic(DynamicSystem):
    """Holonomic point mass with velocity commands.

    State ``x = [x, y]``; input ``u = [vx, vy]``; ``params = {}``.
    """

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Holonomic"

        # Parameters: none
        self.params = {}

        # Signal labels and units
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a camera that follows the body
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()

        # ẋ = u: the commands are the velocities
        dx = jnp.asarray(u)

        return dx

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()

        y = jnp.asarray(x)

        return y

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

        # Parameters: none
        self.params = {}

        # Signal labels and units
        self.state.labels = ["x", "y", "vx", "vy"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["ax", "ay"]
        self.inputs["u"].units = ["m/s^2", "m/s^2"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a camera that follows the body
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()

        vx = x[2]
        vy = x[3]
        ax = u[0]
        ay = u[1]

        # Double integrator: ṗ = v, v̇ = a
        dx = jnp.array([vx, vy, ax, ay])

        return dx

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()

        y = jnp.asarray(x)

        return y

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


# Kinematic and no-slip bicycles


class BicycleKin(DynamicSystem):
    """Kinematic bicycle with speed and steer-angle inputs.

    State ``x = [x, y, theta]``; input ``u = [v, delta]``.
    EoM ``params``: ``{"length": 2.0}``. Graphics ``a``, ``b`` default to
    ``length / 2`` (object attrs, not equation params).
    """

    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3, expose_state=True)
        self.name = "BicycleKin"

        # Parameters: the wheelbase L [m]
        length = 2.0
        self.params = {"length": length}

        # Signal labels and units
        self.state.labels = ["x", "y", "theta"]
        self.state.units = ["m", "m", "rad"]
        self.inputs["u"].labels = ["v", "delta"]
        self.inputs["u"].units = ["m/s", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the 2-D centerline look of the bicycle rungs. The axle offsets
        # a and b split the wheelbase evenly; they are object attributes only, not
        # EoM params
        self.a = 0.5 * length
        self.b = 0.5 * length
        self.wheel_len = 0.76
        self.wheel_width = 0.27
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"
        self.skin = partial(car_skin_2d, color="#1a1a1a")

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        length = params["length"]

        v = u[0]
        delta = u[1]
        theta = x[2]

        # Kinematic bicycle: ẋ = v cos θ, ẏ = v sin θ, θ̇ = v tan δ / L
        dx = jnp.array(
            [
                v * jnp.cos(theta),
                v * jnp.sin(theta),
                v * jnp.tan(delta) / length,
            ]
        )

        return dx

    def h(self, x, u, t=0.0, params=None):
        jnp = require_jax_numpy()

        y = jnp.asarray(x)

        return y

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


# The command ports of the command rungs: BicycleAcc and the three dynamic rungs


class CommandPorts:
    """``named_ports=True`` splits the stacked command ``u`` into one port per command.

    A rung lists its commands in ``port_specs``, ``(name, unit)`` pairs in the order
    of ``u``, and sets ``named_ports`` in its constructor before it adds the ports.
    """

    port_specs: tuple[tuple[str, str], ...] = ()

    def add_command_ports(self):
        """One input port per command when ``named_ports``, else the stacked ``u``."""
        names = [name for name, _ in self.port_specs]
        units = [unit for _, unit in self.port_specs]
        if self.named_ports:
            for name, unit in self.port_specs:
                self.add_input_port(
                    name, nominal_value=0.0, labels=[name], units=[unit]
                )
        else:
            self.add_input_port(
                "u",
                dim=len(names),
                nominal_value=np.zeros(len(names)),
                labels=names,
                units=units,
            )

    def command_vector(self, u):
        """The commands as one vector, from the named ports or the stacked ``u``."""
        jnp = require_jax_numpy()

        if self.named_ports:
            parts = self.get_port_values_from_u(
                u, *[name for name, _ in self.port_specs]
            )
            commands = jnp.array([part[0] for part in parts])
        else:
            commands = jnp.asarray(u)

        return commands


class BicycleAcc(BicycleKin, CommandPorts):
    """No-slip bicycle with longitudinal accel and steer-rate inputs.

    State ``x = [x, y, theta, v, delta]``; input ``u = [a_x, delta_dot]``.

    Equations of motion
    -------------------
    ``ẋ = v cos θ``, ``ẏ = v sin θ``, ``θ̇ = (v / L) tan δ``

    ``v̇ = a_x``, ``δ̇ = δ̇_u``

    EoM ``params``: ``length`` only (``L``). Axle offsets ``a`` / ``b`` are
    graphics attributes (default ``length / 2``), not EoM params.
    """

    port_specs = (("a_x", "m/s^2"), ("delta_dot", "rad/s"))

    def __init__(self, named_ports=False):
        super().__init__()
        self.name = "BicycleAcc"

        # State: the pose, then the speed and the steer angle
        self.n = 5
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)
        self.state.labels = ["x", "y", "theta", "v", "delta"]
        self.state.units = ["m", "m", "rad", "m/s", "rad"]

        # Ports: one port per command, or the stacked u; the state out
        self.named_ports = bool(named_ports)
        self.inputs = {}
        self.add_command_ports()
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        length = params["length"]

        theta = x[2]
        v = x[3]
        delta = x[4]
        a_x, delta_dot = self.command_vector(u)

        # No-slip pose rates: ẋ = v cos θ, ẏ = v sin θ, θ̇ = v tan δ / L
        pose_dot = jnp.array(
            [
                v * jnp.cos(theta),
                v * jnp.sin(theta),
                v * jnp.tan(delta) / length,
            ]
        )

        # The speed and the steer integrate the commands: v̇ = a_x, δ̇ = δ̇_u
        dx = jnp.concatenate([pose_dot, jnp.array([a_x, delta_dot])])

        return dx

    def tf(self, x, u, t=0, params=None):
        a = self.a

        delta = x[4]
        T_wb = SE2(x[0], x[1], x[2])

        return {
            "body": T_wb,
            "axle_front": T_wb @ SE2(a, 0.0, delta),
        }


# The dynamic rungs: subclasses of the catalog's BicycleDynRate, so its a / b axle
# params, its tire physics and its [w_rear, delta] states carry over


class BicycleDynTauRate(BicycleDynRate, CommandPorts):
    """Dynamic bicycle with direct rear torque and steer-rate inputs.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta]``.
    Input ``u = [tau_rear, delta_dot]``.

    Wheel spin: ``Jw_rear * w_rear_dot = tau_rear - rear_wheel_ground_torque``.
    Steer integrates ``delta_dot`` directly (no lag).
    """

    port_specs = (("tau_rear", "Nm"), ("delta_dot", "rad/s"))

    def __init__(self, named_ports=False):
        super().__init__(named_ports=False)
        self.name = "BicycleDynTauRate"

        # State: the body's six, then the rear wheel rate and the steer angle
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

        # Parameters: the rear wheel's inertia and viscous drag
        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0

        # Ports: one port per command, or the stacked u; the state out
        self.named_ports = bool(named_ports)
        self.inputs = {}
        self.add_command_ports()
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        u_in = x[6:8]
        tau_rear, delta_dot = self.command_vector(u)

        # The chassis terms: the catalog rung's rigid-body equations
        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Chassis: M v̇ + C v + d = 0, and q̇ = N v
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        # Wheel parameters, read once the chassis terms have read theirs
        Jw_rear = params["Jw_rear"]

        # Wheel spin: Jw ẇ_rear = τ_rear − τ_ground(v, w_rear, δ)
        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        w_rear_dot = (tau_rear - tau_ground) / Jw_rear

        # ẋ = [q̇; v̇; ẇ_rear; δ̇], the steer integrating its rate command directly
        dx = jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot])])

        return dx


class BicycleDynServo(BicycleDynRate, CommandPorts):
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

    port_specs = (("tau_cmd", "Nm"), ("delta_cmd", "rad"))

    def __init__(self, named_ports=False):
        super().__init__(named_ports=False)
        self.name = "BicycleDynServo"

        # State: the body's six, the rear wheel rate, the steer angle and the
        # lagged torque
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

        # Parameters: the rear wheel's inertia and viscous drag, the steer lag and
        # rate limit, and the torque lag
        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0
        self.params["steering_tau"] = 0.15
        self.params["steer_rate_max"] = 10.0
        self.params["torque_tau"] = 0.05

        # Ports: one port per command, or the stacked u; the state out
        self.named_ports = bool(named_ports)
        self.inputs = {}
        self.add_command_ports()
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        tau = x[8]
        u_in = x[6:8]
        tau_cmd, delta_cmd = self.command_vector(u)

        # The chassis terms: the catalog rung's rigid-body equations
        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Chassis: M v̇ + C v + d = 0, and q̇ = N v
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        # Wheel and actuator parameters, read once the chassis terms have read theirs
        Jw_rear = params["Jw_rear"]
        torque_tau = params["torque_tau"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        # Wheel spin under the lagged torque: Jw ẇ_rear = τ − τ_ground(v, w_rear, δ)
        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        w_rear_dot = (tau - tau_ground) / Jw_rear

        # Torque lag: τ̇ = (τ_cmd − τ) / torque_tau
        tau_dot = (tau_cmd - tau) / torque_tau

        # Steer lag with a rate limit: δ̇ = clip((δ_cmd − δ) / steering_tau, ±rate_max)
        delta_dot = (delta_cmd - delta) / steering_tau
        delta_dot = jnp.clip(delta_dot, -rate_max, rate_max)

        # ẋ = [q̇; v̇; ẇ_rear; δ̇; τ̇]
        dx = jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot, tau_dot])])

        return dx


class BicycleDynEngine(BicycleDynRate, CommandPorts):
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

    port_specs = (("P_cmd", "W"), ("delta_cmd", "rad"))

    def __init__(self, named_ports=False):
        super().__init__(named_ports=False)
        self.name = "BicycleDynEngine"

        # State: the body's six, the rear wheel rate, the steer angle and the
        # lagged power
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

        # Parameters: the rear wheel's inertia and viscous drag, the steer lag and
        # rate limit, the power lag, the stall torque, and the drivetrain brake
        self.params["Jw_rear"] = 1.6
        self.params["bw_rear"] = 0.0
        self.params["steering_tau"] = 0.15
        self.params["steer_rate_max"] = 10.0
        self.params["engine_tau"] = 0.25
        self.params["tau_sat"] = 2500.0
        self.params["bw_engine"] = 2.0
        self.params["tau_fric"] = 20.0

        # Ports: one port per command, or the stacked u; the state out
        self.named_ports = bool(named_ports)
        self.inputs = {}
        self.add_command_ports()
        self.outputs = {}
        self.add_output_port("y", dim=self.n, function=self.h, dependencies=())
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        q = x[0:3]
        v = x[3:6]
        w_rear = x[6]
        delta = x[7]
        P = x[8]
        u_in = x[6:8]
        P_cmd, delta_cmd = self.command_vector(u)

        # The chassis terms: the catalog rung's rigid-body equations
        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        # Chassis: M v̇ + C v + d = 0, and q̇ = N v
        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        # Drive and steer parameters, read once the chassis terms have read theirs
        tau_sat = params["tau_sat"]
        bw_engine = params["bw_engine"]
        tau_fric = params["tau_fric"]
        Jw_rear = params["Jw_rear"]
        engine_tau = params["engine_tau"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        # Torque from power: τ = clip(P / ω_r, ±τ_sat). A stalled wheel (|ω_r| < 10⁻⁶)
        # divides by 10⁻⁶ instead, so τ = τ_sat sign(P) rather than 0/0
        w_safe = jnp.where(jnp.abs(w_rear) < 1e-6, 1e-6, w_rear)
        tau = jnp.clip(P / w_safe, -tau_sat, tau_sat)

        # Wheel spin: Jw ẇ_rear = τ − τ_ground − τ_brake, with the drivetrain brake
        # τ_brake = bw_engine ω_r + τ_fric sign(ω_r)
        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        tau_brake = bw_engine * w_rear + tau_fric * jnp.sign(w_rear)
        w_rear_dot = (tau - tau_ground - tau_brake) / Jw_rear

        # Power lag: Ṗ = (P_cmd − P) / engine_tau
        P_dot = (P_cmd - P) / engine_tau

        # Steer lag with a rate limit: δ̇ = clip((δ_cmd − δ) / steering_tau, ±rate_max)
        delta_dot = (delta_cmd - delta) / steering_tau
        delta_dot = jnp.clip(delta_dot, -rate_max, rate_max)

        # ẋ = [q̇; v̇; ẇ_rear; δ̇; Ṗ]
        dx = jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot, P_dot])])

        return dx
