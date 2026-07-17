"""Named vehicle profiles for bicycle-model plants.

Each :class:`CarProfile` bundles rigid-body parameters, linear tire corners,
actuator time constants, and planning limits derived from the same physical
envelope. Use :func:`apply_car_profile` to copy a profile onto a catalog plant
without editing individual demos.

Rate-input bounds (``JaxDynamicBicycleRateInputs``)
---------------------------------------------------
Two inputs are tied to actuator physics:

**Wheel spin acceleration** ``w_rear_dot`` [rad/s²]
    Derived from longitudinal grip at the contact patch:

    ``w_rear_dot_max = a_long_max / r_r``

    where ``a_long_max`` [m/s²] is the larger of peak accel and peak brake
    (typical passenger car: ~0.4 g accel, ~0.8 g brake). No-slip kinematics:
    ``a_x ≈ r_r * w_rear_dot``.

**Steering rate** ``delta_dot`` [rad/s]
    Maximum road-wheel slew rate (angle at the tire, not the steering wheel).
    Set equal to ``steer_rate_max`` in the profile — the cap on the embedded
    steering servo in :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycleServoInputs`
    and a first-order steer actuator with rate saturation. Typical passenger
    values are ~1 rad/s; race / demo plants may allow more.

Profiles
--------
- :func:`passenger_car_profile` — full-size sedan-scale catalog defaults
- :func:`racecar_profile` — bicycle LOS ``EngineBicycle`` (rounded from demo)
- :func:`udes_1_5_profile` — 1:5 UdeS racecar scale (:class:`~minilink.dynamics.catalog.vehicles.steering.UdeSRacecar` geometry)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

GRAVITY = 9.81


@dataclass(frozen=True)
class CarLimits:
    """State and input bounds shared across abstraction layers.

    ``delta_dot_max`` and ``w_rear_dot_max`` are the rate-input planning limits
    on :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycleRateInputs`.
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

    def traction_torque_max(self) -> float:
        return self.mu * self.rear_normal_load() * self.r_r

    def power_torque_at_speed(self, vx: float) -> float:
        w_rear = max(abs(vx) / self.r_r, 1.0)
        w_engine = w_rear * self.transmission_ratio
        return self.engine_power_peak / w_engine

    def motor_torque_cap(self, vx: float) -> float:
        return min(self.traction_torque_max(), self.power_torque_at_speed(vx))

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


def _make_limits(
    *,
    vx_max: float,
    vy_max: float,
    yaw_rate_max: float,
    a_accel: float,
    a_brake: float,
    delta_max: float,
    steer_rate_max: float,
    r_r: float,
    mass: float,
    mu: float,
    a: float,
    b: float,
    engine_power_peak: float,
    transmission_ratio: float,
) -> CarLimits:
    """Build limits with round numbers from physical caps.

    ``w_rear_dot_max`` comes from ``a_long_max / r_r`` (brake-limited wheel
    spin acceleration). ``delta_dot_max`` follows ``steer_rate_max`` (road-wheel
    steering slew).
    """
    length = a + b
    Fz_r = mass * GRAVITY * b / length
    tau_traction = mu * Fz_r * r_r
    w_cruise = vx_max / r_r
    tau_power = engine_power_peak / max(w_cruise * transmission_ratio, 1.0)
    tau_rear_max = min(tau_traction, tau_power)

    a_long_max = max(a_accel, a_brake)
    w_rear_max = round(vx_max / r_r)
    w_rear_dot_max = round(a_long_max / r_r)

    return CarLimits(
        vx_max=vx_max,
        vy_max=vy_max,
        yaw_rate_max=yaw_rate_max,
        v_max=vx_max,
        v_dot_max=a_accel,
        delta_max=delta_max,
        delta_dot_max=steer_rate_max,
        w_rear_max=w_rear_max,
        w_rear_dot_max=w_rear_dot_max,
        tau_rear_max=round(tau_rear_max, -1),
        tau_rear_min=-round(tau_traction, -1),
        cascade_accel_max=a_accel,
        a_long_max=a_long_max,
    )


def passenger_car_profile() -> CarProfile:
    """Full-size passenger sedan (~1500 kg, 2.7 m wheelbase).

    ``a_long_max = 8 m/s²`` (~0.8 g brake) → ``w_rear_dot_max = 8 / 0.33 ≈ 24 rad/s²``.
    ``steer_rate_max = 1 rad/s`` at the road wheel.
    """
    mass = 1500.0
    a = 1.2
    b = 1.5
    r_r = 0.33
    mu = 0.9
    vx_max = 27.0
    a_accel = 4.0
    a_brake = 8.0
    delta_max = 0.60
    steer_rate_max = 1.0
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
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=3.0,
            yaw_rate_max=1.5,
            a_accel=a_accel,
            a_brake=a_brake,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            mu=mu,
            a=a,
            b=b,
            engine_power_peak=120000.0,
            transmission_ratio=1.0,
        ),
    )


def racecar_profile() -> CarProfile:
    """Bicycle LOS race vehicle (rounded from ``create_vehicle``).

    ``a_long_max = 10 m/s²`` → ``w_rear_dot_max = 10 / 0.34 ≈ 29 rad/s²``.
    ``steer_rate_max = 10 rad/s`` matches the demo plant steering saturation.
    """
    mass = 700.0
    a = 1.2
    b = 1.0
    r_r = 0.34
    mu = 1.0
    vx_max = 35.0
    a_accel = 10.0
    a_brake = 10.0
    delta_max = round(np.pi / 4.0, 2)
    steer_rate_max = 10.0
    engine_power_peak = 48000.0
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
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=4.0,
            yaw_rate_max=2.0,
            a_accel=a_accel,
            a_brake=a_brake,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            mu=mu,
            a=a,
            b=b,
            engine_power_peak=engine_power_peak,
            transmission_ratio=1.0,
        ),
    )


def udes_1_5_profile() -> CarProfile:
    """1:5 UdeS racecar scale (:class:`~minilink.dynamics.catalog.vehicles.steering.UdeSRacecar`).

    ``a_long_max = 12 m/s²`` (high-grip RC) → ``w_rear_dot_max = 12 / 0.07 ≈ 170 rad/s²``.
    ``steer_rate_max = 3 rad/s`` at the road wheel.
    """
    mass = 10.0
    a = 0.17
    b = 0.17
    r_r = 0.07
    mu = 1.0
    vx_max = 15.0
    a_accel = 12.0
    a_brake = 12.0
    delta_max = 0.55
    steer_rate_max = 3.0
    engine_power_peak = 800.0
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
        limits=_make_limits(
            vx_max=vx_max,
            vy_max=2.0,
            yaw_rate_max=4.0,
            a_accel=a_accel,
            a_brake=a_brake,
            delta_max=delta_max,
            steer_rate_max=steer_rate_max,
            r_r=r_r,
            mass=mass,
            mu=mu,
            a=a,
            b=b,
            engine_power_peak=engine_power_peak,
            transmission_ratio=1.0,
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
