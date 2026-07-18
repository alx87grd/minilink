"""Named vehicle profiles for bicycle-model plants.

Each :class:`CarProfile` bundles rigid-body parameters, linear tire corners,
actuator time constants, and planning limits derived from the same physical
envelope. Use :func:`apply_car_profile` to copy a profile onto a catalog plant
without editing individual demos.

Propulsion limits (nominal speed, power only)
---------------------------------------------
At nominal road speed ``v_nom`` (below top speed ``vx_max``) the actuator cap
is power-limited only:

``tau_max = P_max / omega_nom``, ``omega_nom = v_nom / r_r``.

Grip is **not** applied here — tire slip and force saturation are handled by
the plant tire model during simulation. With no-slip kinematics for the
wheel/vehicle couple,

``w_rear_dot_max = tau_max / (J_w + m * r_r^2)``,
``a_long_max = r_r * w_rear_dot_max``.

Both directions use the same symmetric cap (regen rated like forward drive).

Every profile uses the same pipeline (:func:`nominal_actuator_torque` →
:func:`effective_wheel_inertia` → ``w_rear_dot``, ``a_long``) via
:func:`_make_limits` and the :class:`CarProfile` propulsion helpers.
``v_nom`` is profile-specific; a passenger sedan near traction at 15 m/s is
normal, while race / RC profiles pick lower ``v_nom`` for more headroom.
Compare actuator vs grip with :meth:`CarProfile.actuator_traction_headroom`.

Steering rate ``delta_dot_max`` equals ``steer_rate_max`` (road-wheel slew).

Profiles
--------
- :func:`passenger_car_profile` — full-size sedan-scale catalog defaults
- :func:`racecar_profile` — bicycle LOS ``EngineBicycle`` (rounded from demo)
- :func:`udes_1_5_profile` — 1:5 UdeS racecar scale (:class:`~minilink.dynamics.catalog.vehicles.steering.UdeSRacecar` geometry)

The compare TrajOpt demo solves seven JAX catalog plants (through servo torque
inputs). The next rung — ``EngineBicycle`` with throttle input and first-order
engine lag — lives in ``examples/projects/bicycle_los*/vehicle.py``; see
``DESIGN.md`` vehicle ladder and
``examples/notebooks/demo_bicycle_trajopt_obstacle_scene_compare.ipynb``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

GRAVITY = 9.81


@dataclass(frozen=True)
class CarLimits:
    """State and input bounds shared across abstraction layers.

    ``tau_rear_max`` and ``w_rear_dot_max`` are the symmetric nominal caps from
    power at ``v_nom`` (no slip); ``tau_rear_min = -tau_rear_max``.
    """

    vx_max: float
    vy_max: float
    yaw_rate_max: float
    v_max: float
    v_dot_max: float
    delta_max: float
    delta_dot_max: float
    w_rear_max: float
    w_rear_dot_max: float
    tau_rear_max: float
    tau_rear_min: float
    cascade_accel_max: float
    a_long_max: float


@dataclass(frozen=True)
class CarProfile:
    """Vehicle parameters and limits for bicycle-family plants."""

    name: str
    mass: float
    inertia: float
    a: float
    b: float
    r_f: float
    r_r: float
    gravity: float
    rho: float
    CdA: float
    Ca: float
    Ck: float
    mu: float
    Jw_rear: float
    bw_rear: float
    steering_tau: float
    steer_rate_max: float
    engine_power_peak: float
    engine_tau: float
    transmission_ratio: float
    v_nom: float
    limits: CarLimits

    @property
    def length(self) -> float:
        return self.a + self.b

    @property
    def delta_min(self) -> float:
        return -self.limits.delta_max

    @property
    def steer_Kp(self) -> float:
        return 1.0 / self.steering_tau

    def rear_normal_load(self) -> float:
        return self.mass * self.gravity * self.b / self.length

    def effective_wheel_inertia(self) -> float:
        """``J_w + m r_r^2`` — no-slip wheel/vehicle inertia at the contact patch."""
        return effective_wheel_inertia(self.mass, self.r_r, self.Jw_rear)

    def traction_torque_max(self) -> float:
        return self.mu * self.rear_normal_load() * self.r_r

    def power_torque_at_speed(self, vx: float) -> float:
        return power_torque_at_speed(
            vx,
            engine_power_peak=self.engine_power_peak,
            transmission_ratio=self.transmission_ratio,
            r_r=self.r_r,
        )

    def propulsion_torque_nominal(self) -> float:
        """Peak motor torque at ``v_nom`` [Nm] from ``P_max`` only."""
        return nominal_actuator_torque(
            engine_power_peak=self.engine_power_peak,
            transmission_ratio=self.transmission_ratio,
            r_r=self.r_r,
            v_nom=self.v_nom,
        )

    def propulsion_wheel_accel_nominal(self) -> float:
        """``w_rear_dot`` at ``v_nom`` from motor torque (no-slip couple) [rad/s²]."""
        return nominal_actuator_wheel_accel(
            mass=self.mass,
            r_r=self.r_r,
            Jw_rear=self.Jw_rear,
            engine_power_peak=self.engine_power_peak,
            transmission_ratio=self.transmission_ratio,
            v_nom=self.v_nom,
        )

    def propulsion_longitudinal_accel_nominal(self) -> float:
        """``a_x`` at ``v_nom`` from motor torque (no-slip couple) [m/s²]."""
        return self.r_r * self.propulsion_wheel_accel_nominal()

    def traction_wheel_accel_reference(self) -> float:
        """Grip-coupled ``w_rear_dot`` at full rear traction (reference, not a cap) [rad/s²]."""
        return self.traction_torque_max() / self.effective_wheel_inertia()

    def actuator_traction_headroom(self) -> float:
        """Actuator torque limit divided by traction reference (values > 1 allow wheel spin)."""
        return self.limits.tau_rear_max / self.traction_torque_max()

    def motor_torque_cap(self, vx: float) -> float:
        """Motor torque cap at road speed ``vx`` [Nm] from ``P_max`` only."""
        return self.power_torque_at_speed(vx)

    def to_plant_params(self) -> dict[str, float]:
        return {
            "mass": self.mass,
            "inertia": self.inertia,
            "a": self.a,
            "b": self.b,
            "r_f": self.r_f,
            "r_r": self.r_r,
            "gravity": self.gravity,
            "rho": self.rho,
            "CdA": self.CdA,
            "Jw_rear": self.Jw_rear,
            "bw_rear": self.bw_rear,
            "steer_Kp": self.steer_Kp,
            "steer_rate_max": self.steer_rate_max,
            "delta_min": self.delta_min,
            "delta_max": self.limits.delta_max,
        }


def effective_wheel_inertia(mass: float, r_r: float, Jw_rear: float) -> float:
    """``J_w + m r_r^2`` — no-slip wheel/vehicle inertia [kg·m²]."""
    return Jw_rear + mass * r_r**2


def power_torque_at_speed(
    vx: float,
    *,
    engine_power_peak: float,
    transmission_ratio: float,
    r_r: float,
) -> float:
    """Motor torque [Nm] from ``P_max`` at road speed ``vx``."""
    w_rear = max(abs(vx) / r_r, 1.0)
    w_engine = w_rear * transmission_ratio
    return engine_power_peak / w_engine


def nominal_actuator_torque(
    *,
    engine_power_peak: float,
    transmission_ratio: float,
    r_r: float,
    v_nom: float,
) -> float:
    """Peak motor torque [Nm] from ``P_max`` at nominal speed (planner rating)."""
    return power_torque_at_speed(
        v_nom,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        r_r=r_r,
    )


def nominal_actuator_wheel_accel(
    *,
    mass: float,
    r_r: float,
    Jw_rear: float,
    engine_power_peak: float,
    transmission_ratio: float,
    v_nom: float,
) -> float:
    """``w_rear_dot`` [rad/s²] from nominal motor torque and wheel/vehicle inertia."""
    tau = nominal_actuator_torque(
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        r_r=r_r,
        v_nom=v_nom,
    )
    return tau / effective_wheel_inertia(mass, r_r, Jw_rear)


def _round_torque(tau: float) -> float:
    if abs(tau) >= 100.0:
        return float(round(tau, -1))
    return float(round(tau, 1))


def _make_limits(
    *,
    vx_max: float,
    vy_max: float,
    yaw_rate_max: float,
    delta_max: float,
    steer_rate_max: float,
    r_r: float,
    mass: float,
    Jw_rear: float,
    engine_power_peak: float,
    transmission_ratio: float,
    v_nom: float,
) -> CarLimits:
    """Build symmetric actuator limits from ``P_max`` at ``v_nom``."""
    tau_max = nominal_actuator_torque(
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        r_r=r_r,
        v_nom=v_nom,
    )
    w_rear_dot = nominal_actuator_wheel_accel(
        mass=mass,
        r_r=r_r,
        Jw_rear=Jw_rear,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        v_nom=v_nom,
    )
    a_long = r_r * w_rear_dot

    w_rear_dot_r = round(w_rear_dot)
    w_rear_max = round(vx_max / r_r)
    tau_r = _round_torque(tau_max)
    a_long_r = round(a_long, 1)

    return CarLimits(
        vx_max=vx_max,
        vy_max=vy_max,
        yaw_rate_max=yaw_rate_max,
        v_max=vx_max,
        v_dot_max=a_long_r,
        delta_max=delta_max,
        delta_dot_max=steer_rate_max,
        w_rear_max=w_rear_max,
        w_rear_dot_max=w_rear_dot_r,
        tau_rear_max=tau_r,
        tau_rear_min=-tau_r,
        cascade_accel_max=a_long_r,
        a_long_max=a_long_r,
    )


def passenger_car_profile() -> CarProfile:
    """Full-size passenger sedan (~1500 kg, 2.7 m wheelbase).

    ``P = 120 kW`` at ``v_nom = 15 m/s`` → ``tau ≈ 2640 Nm``, ``w_rear_dot ≈ 16 rad/s²``
    (~1.1× rear traction — typical sedan near grip at moderate speed).
    """
    mass = 1500.0
    a = 1.2
    b = 1.5
    r_r = 0.33
    mu = 0.9
    vx_max = 27.0
    v_nom = 15.0
    delta_max = 0.60
    steer_rate_max = 1.5
    return CarProfile(
        name="passenger_car",
        mass=mass,
        inertia=2200.0,
        a=a,
        b=b,
        r_f=r_r,
        r_r=r_r,
        gravity=GRAVITY,
        rho=1.225,
        CdA=0.65,
        Ca=55000.0,
        Ck=90000.0,
        mu=mu,
        Jw_rear=2.0,
        bw_rear=0.0,
        steering_tau=0.15,
        steer_rate_max=steer_rate_max,
        engine_power_peak=120000.0,
        engine_tau=0.25,
        transmission_ratio=1.0,
        v_nom=v_nom,
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=3.0,
            yaw_rate_max=1.5,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            Jw_rear=2.0,
            engine_power_peak=120000.0,
            transmission_ratio=1.0,
            v_nom=v_nom,
        ),
    )


def racecar_profile() -> CarProfile:
    """Bicycle LOS race vehicle (rounded from ``create_vehicle``).

    ``P = 100 kW`` at ``v_nom = 10 m/s`` → ``tau ≈ 3400 Nm``, ``w_rear_dot ≈ 41 rad/s²``
    (~3× rear traction reference).
    """
    mass = 700.0
    a = 1.2
    b = 1.0
    r_r = 0.34
    mu = 1.0
    vx_max = 35.0
    v_nom = 10.0
    delta_max = round(np.pi / 4.0, 2)
    steer_rate_max = 3.0
    engine_power_peak = 100000.0
    return CarProfile(
        name="racecar",
        mass=mass,
        inertia=700.0,
        a=a,
        b=b,
        r_f=r_r,
        r_r=r_r,
        gravity=GRAVITY,
        rho=1.225,
        CdA=0.0,
        Ca=60000.0,
        Ck=100000.0,
        mu=mu,
        Jw_rear=1.6,
        bw_rear=0.0,
        steering_tau=0.15,
        steer_rate_max=steer_rate_max,
        engine_power_peak=engine_power_peak,
        engine_tau=0.25,
        transmission_ratio=1.0,
        v_nom=v_nom,
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=4.0,
            yaw_rate_max=2.0,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            Jw_rear=1.6,
            engine_power_peak=engine_power_peak,
            transmission_ratio=1.0,
            v_nom=v_nom,
        ),
    )


def udes_1_5_profile() -> CarProfile:
    """1:5 UdeS racecar scale (:class:`~minilink.dynamics.catalog.vehicles.steering.UdeSRacecar`).

    ``P = 200 W`` at ``v_nom = 5 m/s`` → ``tau ≈ 2.8 Nm``, ``w_rear_dot ≈ 41 rad/s²``.
    """
    mass = 10.0
    a = 0.17
    b = 0.17
    r_r = 0.07
    mu = 1.0
    vx_max = 15.0
    v_nom = 5.0
    delta_max = 0.55
    steer_rate_max = 3.0
    engine_power_peak = 200.0
    return CarProfile(
        name="udes_1_5",
        mass=mass,
        inertia=0.10,
        a=a,
        b=b,
        r_f=r_r,
        r_r=r_r,
        gravity=GRAVITY,
        rho=1.225,
        CdA=0.0,
        Ca=12000.0,
        Ck=20000.0,
        mu=mu,
        Jw_rear=0.02,
        bw_rear=0.0,
        steering_tau=0.08,
        steer_rate_max=steer_rate_max,
        engine_power_peak=engine_power_peak,
        engine_tau=0.10,
        transmission_ratio=1.0,
        v_nom=v_nom,
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=2.0,
            yaw_rate_max=4.0,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            Jw_rear=0.02,
            engine_power_peak=engine_power_peak,
            transmission_ratio=1.0,
            v_nom=v_nom,
        ),
    )


CAR_PROFILES: dict[str, CarProfile] = {
    "passenger_car": passenger_car_profile(),
    "racecar": racecar_profile(),
    "udes_1_5": udes_1_5_profile(),
}


def get_car_profile(name: str) -> CarProfile:
    """Return a registered profile by name."""
    try:
        return CAR_PROFILES[name]
    except KeyError as exc:
        known = ", ".join(sorted(CAR_PROFILES))
        raise KeyError(f"Unknown car profile {name!r}; known: {known}") from exc


def list_car_profiles() -> tuple[str, ...]:
    """Registered profile names."""
    return tuple(sorted(CAR_PROFILES))


def _set_tire_models(sys: Any, profile: CarProfile) -> None:
    from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
        JaxLinearTire,
        LinearTire,
    )

    tire_cls = JaxLinearTire if "Jax" in type(sys).__name__ else LinearTire
    tire = tire_cls(Ca=profile.Ca, Ck=profile.Ck, mu=profile.mu)
    sys.tire_model_f = tire
    sys.tire_model_r = tire


def _apply_plant_params(sys: Any, profile: CarProfile) -> None:
    for key, value in profile.to_plant_params().items():
        sys.params[key] = value
    if hasattr(sys, "tire_model_f"):
        _set_tire_models(sys, profile)


def _set_port_bounds(
    port: Any, lower: float | np.ndarray, upper: float | np.ndarray
) -> None:
    port.lower_bound = np.asarray(lower, dtype=float)
    port.upper_bound = np.asarray(upper, dtype=float)


def _apply_kinematic(sys: Any, profile: CarProfile) -> None:
    lim = profile.limits
    delta = lim.delta_max
    if "u" in sys.inputs:
        _set_port_bounds(sys.inputs["u"], [0.0, -delta], [lim.v_max, delta])
    if sys.n >= 5:
        sys.state.lower_bound[3] = 0.0
        sys.state.upper_bound[3] = lim.v_max
        sys.state.lower_bound[4] = -delta
        sys.state.upper_bound[4] = delta
        if "speed_dot" in sys.inputs:
            sd = lim.v_dot_max
            dd = lim.delta_dot_max
            _set_port_bounds(sys.inputs["speed_dot"], [-sd], [sd])
            _set_port_bounds(sys.inputs["steering_dot"], [-dd], [dd])


def _apply_dynamic_six(sys: Any, profile: CarProfile) -> None:
    lim = profile.limits
    delta = lim.delta_max
    w_max = lim.w_rear_max
    if "w_rear" in sys.inputs:
        _set_port_bounds(sys.inputs["w_rear"], [0.0], [w_max])
        _set_port_bounds(sys.inputs["delta"], [-delta], [delta])
    elif "u" in sys.inputs and sys.inputs["u"].dim == 2:
        labels = getattr(sys.inputs["u"], "labels", [])
        if labels and labels[0] == "w_rear":
            _set_port_bounds(sys.inputs["u"], [0.0, -delta], [w_max, delta])


def _apply_dynamic_eight(sys: Any, profile: CarProfile) -> None:
    lim = profile.limits
    delta = lim.delta_max
    w_max = lim.w_rear_max
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_max
    sys.state.lower_bound[7] = -delta
    sys.state.upper_bound[7] = delta

    wdot = lim.w_rear_dot_max
    dd = lim.delta_dot_max
    tau_max = lim.tau_rear_max
    tau_min = lim.tau_rear_min

    if "w_rear_dot" in sys.inputs:
        _set_port_bounds(sys.inputs["w_rear_dot"], [-wdot], [wdot])
        _set_port_bounds(sys.inputs["delta_dot"], [-dd], [dd])
    elif "tau_rear" in sys.inputs:
        _set_port_bounds(sys.inputs["tau_rear"], [tau_min], [tau_max])
        _set_port_bounds(sys.inputs["delta_sp"], [-delta], [delta])
    elif "u" in sys.inputs:
        labels = getattr(sys.inputs["u"], "labels", [])
        if labels and labels[0] in ("w_rear_dot", "tau_rear"):
            if labels[0] == "w_rear_dot":
                _set_port_bounds(sys.inputs["u"], [-wdot, -dd], [wdot, dd])
            else:
                _set_port_bounds(sys.inputs["u"], [tau_min, -delta], [tau_max, delta])


_APPLY_BY_CLASS: dict[str, Any] = {
    "JaxKinematicBicycle": _apply_kinematic,
    "KinematicBicycle": _apply_kinematic,
    "JaxKinematicBicycleRateInputs": _apply_kinematic,
    "JaxDynamicBicycle": _apply_dynamic_six,
    "DynamicBicycle": _apply_dynamic_six,
    "JaxDynamicBicycleRateInputs": _apply_dynamic_eight,
    "JaxDynamicBicycleRateInputsUY": _apply_dynamic_eight,
    "JaxDynamicBicycleServoInputs": _apply_dynamic_eight,
    "JaxDynamicBicycleServoInputsUY": _apply_dynamic_eight,
}


def apply_car_profile(sys: Any, profile: CarProfile | str) -> Any:
    """Apply ``profile`` parameters and bounds to a bicycle-family plant.

    Parameters
    ----------
    sys
        A kinematic or dynamic bicycle plant from ``minilink.dynamics.catalog.vehicles``.
    profile
        :class:`CarProfile` instance or registered profile name.

    Returns
    -------
    sys
        The same object, for chaining.
    """
    if isinstance(profile, str):
        profile = get_car_profile(profile)

    _apply_plant_params(sys, profile)
    applicator = _APPLY_BY_CLASS.get(type(sys).__name__)
    if applicator is not None:
        applicator(sys, profile)
    return sys
