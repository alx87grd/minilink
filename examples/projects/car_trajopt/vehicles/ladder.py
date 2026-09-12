"""JAX vehicle fidelity ladder — the research rungs (car_trajopt project).

The teaching catalog keeps four rungs: ``HolonomicMobileRobot`` →
``KinematicBicycle`` / ``KinematicCar`` → ``DynamicBicycle`` (linear tires) →
``BicycleDynRate`` (wheel-rate / steer-rate inputs, the MPC plant). This module
adds the planning-oriented variants around them, JAX-first:

=================  ===  ===========================  =========================
Class              n    u                            Role
=================  ===  ===========================  =========================
Holonomic          2    ``[vx, vy]``                 point, velocity
HolonomicAccel     4    ``[ax, ay]``                 point, accel
BicycleKin         3    ``[v, delta]``               nonholonomic
BicycleAcc         5    ``[a_x, delta_dot]``         no-slip accel / steer
BicycleDynTauRate  8    ``[tau_rear, delta_dot]``    torque + steer rate
BicycleDynServo    9    ``[tau_cmd, delta_cmd]``     lag on tau and delta
BicycleDynEngine   9    ``[P_cmd, delta_cmd]``       power lag + wheel
=================  ===  ===========================  =========================

Every command plant takes ``named_ports=True`` for one port per command
instead of the stacked ``u``. The torque / servo / engine rungs subclass the
catalog ``BicycleDynRate`` (a / b axle params, shared tire physics).
"""

from functools import partial

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import BicycleDynRate
from minilink.graphical.animation.primitives import Arrow, Circle
from minilink.graphical.catalog.skins import car_skin_2d


class _CommandPorts:
    """``named_ports=True`` splits the stacked command ``u`` into one port per command."""

    _port_specs: tuple[tuple[str, str], ...] = ()

    def _install_command_ports(self, named_ports):
        self.named_ports = bool(named_ports)
        self.inputs = {}
        names = [name for name, _ in self._port_specs]
        units = [unit for _, unit in self._port_specs]
        if self.named_ports:
            for name, unit in self._port_specs:
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

    def _commands(self, u):
        jnp = require_jax_numpy()
        if self.named_ports:
            parts = self.get_port_values_from_u(
                u, *[name for name, _ in self._port_specs]
            )
            return jnp.array([part[0] for part in parts])
        return jnp.asarray(u)


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


class BicycleAcc(BicycleKin, _CommandPorts):
    """No-slip bicycle with longitudinal accel and steer-rate inputs.

    State ``x = [x, y, theta, v, delta]``; input ``u = [a_x, delta_dot]``.

    Equations of motion
    -------------------
    ``ẋ = v cos θ``, ``ẏ = v sin θ``, ``θ̇ = (v / L) tan δ``

    ``v̇ = a_x``, ``δ̇ = δ̇_u``

    EoM ``params``: ``length`` only (``L``). Axle offsets ``a`` / ``b`` are
    graphics attributes (default ``length / 2``), not EoM params.
    """

    _port_specs = (("a_x", "m/s^2"), ("delta_dot", "rad/s"))

    def __init__(self, named_ports=False):
        from minilink.core.signals import VectorSignal

        super().__init__()
        self.name = "BicycleAcc"
        self.n = 5
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)
        self.state.labels = ["x", "y", "theta", "v", "delta"]
        self.state.units = ["m", "m", "rad", "m/s", "rad"]

        self._install_command_ports(named_ports)
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
        a_x, delta_dot = self._commands(u)

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


class BicycleDynTauRate(BicycleDynRate, _CommandPorts):
    """Dynamic bicycle with direct rear torque and steer-rate inputs.

    State ``x = [x, y, theta, vx, vy, yaw_rate, w_rear, delta]``.
    Input ``u = [tau_rear, delta_dot]``.

    Wheel spin: ``Jw_rear * w_rear_dot = tau_rear - rear_wheel_ground_torque``.
    Steer integrates ``delta_dot`` directly (no lag).
    """

    _port_specs = (("tau_rear", "Nm"), ("delta_dot", "rad/s"))

    def __init__(self, named_ports=False):
        from minilink.core.signals import VectorSignal

        super().__init__(named_ports=False)
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

        self._install_command_ports(named_ports)
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

        tau_rear, delta_dot = self._commands(u)

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


class BicycleDynServo(BicycleDynRate, _CommandPorts):
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

    _port_specs = (("tau_cmd", "Nm"), ("delta_cmd", "rad"))

    def __init__(self, named_ports=False):
        from minilink.core.signals import VectorSignal

        super().__init__(named_ports=False)
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

        self._install_command_ports(named_ports)
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

        tau_cmd, delta_cmd = self._commands(u)

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


class BicycleDynEngine(BicycleDynRate, _CommandPorts):
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

    _port_specs = (("P_cmd", "W"), ("delta_cmd", "rad"))

    def __init__(self, named_ports=False):
        from minilink.core.signals import VectorSignal

        super().__init__(named_ports=False)
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

        self._install_command_ports(named_ports)
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

        P_cmd, delta_cmd = self._commands(u)

        M = self.M(q, params)
        C = self.C(q, v, params)
        N = self.N(q, params)
        d = self.generalized_d(q, v, u_in, params)

        dv = jnp.linalg.solve(M, -C @ v - d)
        dq = N @ v

        tau_sat = params["tau_sat"]
        bw_engine = params["bw_engine"]
        tau_fric = params["tau_fric"]
        Jw_rear = params["Jw_rear"]
        engine_tau = params["engine_tau"]
        steering_tau = params["steering_tau"]
        rate_max = params["steer_rate_max"]

        # ω≈0 → τ = τ_sat · sign(P); else clip(P/ω, ±τ_sat). Safe denom avoids 0/0.
        w_safe = jnp.where(jnp.abs(w_rear) < 1e-6, 1e-6, w_rear)
        tau = jnp.clip(P / w_safe, -tau_sat, tau_sat)

        tau_ground = self.rear_wheel_ground_torque(v, w_rear, delta, params)
        tau_brake = bw_engine * w_rear + tau_fric * jnp.sign(w_rear)
        w_rear_dot = (tau - tau_ground - tau_brake) / Jw_rear

        P_dot = (P_cmd - P) / engine_tau
        delta_dot = (delta_cmd - delta) / steering_tau
        delta_dot = jnp.clip(delta_dot, -rate_max, rate_max)

        return jnp.concatenate([dq, dv, jnp.array([w_rear_dot, delta_dot, P_dot])])

    def _u_in(self, x, u):
        return x[6:8]
