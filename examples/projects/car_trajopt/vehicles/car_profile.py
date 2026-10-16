"""Car profiles: the parameters and planning limits of the bicycle-family plants, from one physical envelope."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

GRAVITY = 9.81


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class CarProfile:
    """Vehicle parameters and limits for bicycle-family plants.

    Each profile bundles rigid-body parameters, linear tire corners, actuator
    time constants, and planning limits derived from the same physical
    envelope. Use :func:`apply_car_profile` to copy a profile onto a catalog
    plant without editing individual demos.

    Engine plants (``BicycleDynEngine`` (``ladder.py``)) also take
    ``tau_sat``, ``bw_engine``, and ``tau_fric`` from the profile;
    ``engine_power_peak`` is a port / state bound only (not an EoM saturation).

    Profiles
    --------
    - :func:`passenger_car_profile` — full-size sedan-scale catalog defaults
    - :func:`racecar_profile` — lightweight race vehicle envelope
    - :func:`udes_1_5_profile` — 1:5 RC racecar scale (same ``a``, ``b`` as catalog ``UdeSRacecar``)
    """

    name: str

    # Chassis
    mass: float
    inertia: float
    a: float
    b: float
    r_f: float
    r_r: float
    gravity: float

    # Aero
    rho: float
    CdA: float

    # Tires (linear, shared front and rear)
    Ca: float
    Ck: float
    mu: float

    # Rear wheel
    Jw_rear: float
    bw_rear: float

    # Steering
    steering_tau: float
    steer_rate_max: float

    # Propulsion
    engine_power_peak: float
    engine_tau: float
    transmission_ratio: float
    tau_sat: float
    bw_engine: float
    tau_fric: float

    # Planning
    v_nom: float
    limits: CarLimits

    @property
    def length(self) -> float:
        """The wheelbase ``L = a + b`` [m]."""
        return self.a + self.b

    @property
    def delta_min(self) -> float:
        """The lower steer stop, ``-delta_max`` [rad]."""
        return -self.limits.delta_max

    @property
    def steer_Kp(self) -> float:
        """The steering servo's gain, ``1 / steering_tau`` [1/s]."""
        return 1.0 / self.steering_tau

    def to_plant_params(self) -> dict[str, float]:
        """The profile under the parameter keys of the ``a`` / ``b`` plants."""
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

    # Grip

    def rear_normal_load(self) -> float:
        """The static rear axle load ``F_zr = m g b / L`` [N]."""
        mass = self.mass
        gravity = self.gravity
        b = self.b
        length = self.length

        # F_zr = m g b / L, the rear axle's share of the weight
        F_zr = mass * gravity * b / length

        return F_zr

    def traction_torque_max(self) -> float:
        """The rear torque at full grip, ``μ F_zr r_r`` [Nm]."""
        mu = self.mu
        r_r = self.r_r

        F_zr = self.rear_normal_load()

        # τ_traction = μ F_zr r_r
        tau_traction = mu * F_zr * r_r

        return tau_traction

    def traction_wheel_accel_reference(self) -> float:
        """Grip-coupled ``w_rear_dot`` at full rear traction (reference, not a cap) [rad/s²]."""
        tau_traction = self.traction_torque_max()
        J_eff = self.effective_wheel_inertia()

        # ẇ_r = τ_traction / (J_w + m r_r²)
        w_rear_dot = tau_traction / J_eff

        return w_rear_dot

    # Propulsion

    def effective_wheel_inertia(self) -> float:
        """``J_w + m r_r^2`` — no-slip wheel/vehicle inertia at the contact patch."""
        return effective_wheel_inertia(self.mass, self.r_r, self.Jw_rear)

    def power_torque_at_speed(self, vx: float) -> float:
        """Motor torque [Nm] from ``P_max`` at road speed ``vx``."""
        return power_torque_at_speed(
            vx,
            engine_power_peak=self.engine_power_peak,
            transmission_ratio=self.transmission_ratio,
            r_r=self.r_r,
        )

    def motor_torque_cap(self, vx: float) -> float:
        """Motor torque cap at road speed ``vx`` [Nm] from ``P_max`` only."""
        return self.power_torque_at_speed(vx)

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
        r_r = self.r_r

        w_rear_dot = self.propulsion_wheel_accel_nominal()

        # a_x = r_r ẇ_r, with no slip at the contact patch
        a_x = r_r * w_rear_dot

        return a_x

    # Actuator against grip

    def actuator_traction_headroom(self) -> float:
        """Actuator torque limit divided by traction reference (values > 1 allow wheel spin)."""
        tau_max = self.limits.tau_rear_max

        tau_traction = self.traction_torque_max()

        # τ_max / τ_traction
        headroom = tau_max / tau_traction

        return headroom


@dataclass(frozen=True)
class CarLimits:
    """State and input bounds shared across abstraction layers.

    ``tau_rear_max`` and ``w_rear_dot_max`` are the symmetric nominal caps from
    power at ``v_nom`` (no slip); ``tau_rear_min = -tau_rear_max``.

    Propulsion limits (nominal speed, power only)
    ---------------------------------------------
    At nominal road speed ``v_nom`` (below top speed ``vx_max``) the actuator
    cap is power-limited only:

    ``tau_max = P_max / omega_nom``, ``omega_nom = v_nom / r_r``.

    Grip is **not** applied here — tire slip and force saturation are handled
    by the plant tire model during simulation. With no-slip kinematics for the
    wheel/vehicle couple,

    ``w_rear_dot_max = tau_max / (J_w + m * r_r^2)``,
    ``a_long_max = r_r * w_rear_dot_max``.

    Both directions use the same symmetric cap (regen rated like forward drive).

    Every profile uses the same pipeline (:func:`nominal_actuator_torque` →
    :func:`effective_wheel_inertia` → ``w_rear_dot``, ``a_long``) via
    :func:`make_limits` and the :class:`CarProfile` propulsion helpers.
    ``v_nom`` is profile-specific; a passenger sedan near traction at 15 m/s
    is normal, while race / RC profiles pick lower ``v_nom`` for more headroom.
    Compare actuator vs grip with :meth:`CarProfile.actuator_traction_headroom`.

    Steering rate ``delta_dot_max`` equals ``steer_rate_max`` (road-wheel slew).
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


def apply_car_profile(sys: Any, profile: CarProfile | str) -> Any:
    """Apply ``profile`` parameters and bounds to a bicycle-family plant.

    Parameters
    ----------
    sys
        A kinematic or dynamic bicycle plant (catalog rung or ladder rung).
    profile
        :class:`CarProfile` instance or registered profile name.

    Returns
    -------
    sys
        The same object, for chaining.
    """
    # A registered name stands for its profile
    if isinstance(profile, str):
        profile = get_car_profile(profile)

    # The parameters the plant declares, then the bounds its class carries
    apply_plant_params(sys, profile)
    applicator = _APPLY_BY_CLASS.get(type(sys).__name__)
    if applicator is not None:
        applicator(sys, profile)

    return sys


def to_jax_plant_params(profile: CarProfile) -> dict[str, float]:
    """Minimal EoM ``params`` for the JAX ladder rungs (``length``-based plants)."""
    # L = a + b
    length = profile.a + profile.b

    return {
        "length": length,
        "mass": profile.mass,
        "inertia": profile.inertia,
        "r_f": profile.r_f,
        "r_r": profile.r_r,
        "gravity": profile.gravity,
        "rho": profile.rho,
        "CdA": profile.CdA,
        "Ca": profile.Ca,
        "Ck": profile.Ck,
        "mu": profile.mu,
        "v_min_epsilon": 0.1,
        "Jw_rear": profile.Jw_rear,
        "bw_rear": profile.bw_rear,
        "steering_tau": profile.steering_tau,
        "steer_rate_max": profile.steer_rate_max,
        "torque_tau": 0.05,
        "engine_tau": profile.engine_tau,
        "tau_sat": profile.tau_sat,
        "bw_engine": profile.bw_engine,
        "tau_fric": profile.tau_fric,
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


# The profiles


def passenger_car_profile() -> CarProfile:
    """Full-size passenger sedan (~1500 kg, 2.7 m wheelbase).

    ``P = 120 kW`` at ``v_nom = 15 m/s`` → ``tau ≈ 2640 Nm``, ``w_rear_dot ≈ 16 rad/s²``
    (~1.1× rear traction — typical sedan near grip at moderate speed).
    """
    # The envelope the limits derive from
    mass = 1500.0
    a = 1.2
    b = 1.5
    r_r = 0.33
    mu = 0.9
    vx_max = 27.0
    v_nom = 15.0
    delta_max = 0.60
    steer_rate_max = 1.5
    Jw_rear = 2.0
    engine_power_peak = 120000.0
    transmission_ratio = 1.0

    # The planning limits: P_max at v_nom, with no slip
    limits = make_limits(
        vx_max=vx_max,
        vy_max=3.0,
        yaw_rate_max=1.5,
        delta_max=delta_max,
        steer_rate_max=steer_rate_max,
        r_r=r_r,
        mass=mass,
        Jw_rear=Jw_rear,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        v_nom=v_nom,
    )

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
        Jw_rear=Jw_rear,
        bw_rear=0.0,
        steering_tau=0.15,
        steer_rate_max=steer_rate_max,
        engine_power_peak=engine_power_peak,
        engine_tau=0.25,
        transmission_ratio=transmission_ratio,
        tau_sat=3000.0,
        bw_engine=2.0,
        tau_fric=20.0,
        v_nom=v_nom,
        limits=limits,
    )


def racecar_profile() -> CarProfile:
    """Lightweight race vehicle envelope.

    ``P = 100 kW`` at ``v_nom = 10 m/s`` → ``tau ≈ 3400 Nm``, ``w_rear_dot ≈ 41 rad/s²``
    (~3× rear traction reference).
    """
    # The envelope the limits derive from
    mass = 700.0
    a = 1.2
    b = 1.0
    r_r = 0.34
    mu = 1.0
    vx_max = 35.0
    v_nom = 10.0
    delta_max = round(np.pi / 4.0, 2)
    steer_rate_max = 3.0
    Jw_rear = 1.6
    engine_power_peak = 100000.0
    transmission_ratio = 1.0

    # The planning limits: P_max at v_nom, with no slip
    limits = make_limits(
        vx_max=vx_max,
        vy_max=4.0,
        yaw_rate_max=2.0,
        delta_max=delta_max,
        steer_rate_max=steer_rate_max,
        r_r=r_r,
        mass=mass,
        Jw_rear=Jw_rear,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        v_nom=v_nom,
    )

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
        Jw_rear=Jw_rear,
        bw_rear=0.0,
        steering_tau=0.15,
        steer_rate_max=steer_rate_max,
        engine_power_peak=engine_power_peak,
        engine_tau=0.25,
        transmission_ratio=transmission_ratio,
        tau_sat=4000.0,
        bw_engine=2.0,
        tau_fric=20.0,
        v_nom=v_nom,
        limits=limits,
    )


def udes_1_5_profile() -> CarProfile:
    """1:5 UdeS racecar scale (same ``a``, ``b`` as catalog ``UdeSRacecar``).

    ``P = 200 W`` at ``v_nom = 5 m/s`` → ``tau ≈ 2.8 Nm``, ``w_rear_dot ≈ 41 rad/s²``.
    """
    # The envelope the limits derive from
    mass = 10.0
    a = 0.17
    b = 0.17
    r_r = 0.07
    mu = 1.0
    vx_max = 15.0
    v_nom = 5.0
    delta_max = 0.55
    steer_rate_max = 3.0
    Jw_rear = 0.02
    engine_power_peak = 200.0
    transmission_ratio = 1.0

    # The planning limits: P_max at v_nom, with no slip
    limits = make_limits(
        vx_max=vx_max,
        vy_max=2.0,
        yaw_rate_max=4.0,
        delta_max=delta_max,
        steer_rate_max=steer_rate_max,
        r_r=r_r,
        mass=mass,
        Jw_rear=Jw_rear,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        v_nom=v_nom,
    )

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
        Jw_rear=Jw_rear,
        bw_rear=0.0,
        steering_tau=0.08,
        steer_rate_max=steer_rate_max,
        engine_power_peak=engine_power_peak,
        engine_tau=0.10,
        transmission_ratio=transmission_ratio,
        tau_sat=5.0,
        bw_engine=0.05,
        tau_fric=0.2,
        v_nom=v_nom,
        limits=limits,
    )


# Propulsion: the power-limited torque, and the wheel acceleration it buys


def power_torque_at_speed(
    vx: float,
    *,
    engine_power_peak: float,
    transmission_ratio: float,
    r_r: float,
) -> float:
    """Motor torque [Nm] from ``P_max`` at road speed ``vx``."""
    # ω_r = max(|v_x| / r_r, 1), the rear wheel rate floored at 1 rad/s
    w_rear = max(abs(vx) / r_r, 1.0)

    # ω_e = N ω_r, through the transmission
    w_engine = w_rear * transmission_ratio

    # τ = P_max / ω_e
    tau = engine_power_peak / w_engine

    return tau


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


def effective_wheel_inertia(mass: float, r_r: float, Jw_rear: float) -> float:
    """``J_w + m r_r^2`` — no-slip wheel/vehicle inertia [kg·m²]."""
    # J_eff = J_w + m r_r², the vehicle's mass seen at the contact patch
    J_eff = Jw_rear + mass * r_r**2

    return J_eff


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
    J_eff = effective_wheel_inertia(mass, r_r, Jw_rear)

    # ẇ_r = τ / (J_w + m r_r²)
    w_rear_dot = tau / J_eff

    return w_rear_dot


###############################################################################
# Internal machinery
###############################################################################


# Parameters and bounds on a plant


def apply_plant_params(sys: Any, profile: CarProfile) -> None:
    """Copy the profile onto the keys the plant declares (``length`` or ``a`` / ``b``)."""
    # Every key the plant declares, from either key set
    values = {**profile.to_plant_params(), **to_jax_plant_params(profile)}
    for key, value in values.items():
        if key in sys.params:
            sys.params[key] = value

    # Graphics axle offsets kept as object attributes by the kinematic rungs
    for key in ("a", "b"):
        if key not in sys.params and hasattr(sys, key):
            setattr(sys, key, getattr(profile, key))

    # Linear-slip tire coefficients live in plant params (shared front/rear)
    if "Ca" in sys.params:
        sys.params["Ca"] = profile.Ca
        sys.params["Ck"] = profile.Ck
        sys.params["mu"] = profile.mu


def apply_kinematic(sys: Any, profile: CarProfile) -> None:
    """Bound a kinematic rung: its speed and steer states, or its speed-and-steer input."""
    limits = profile.limits
    v_max = limits.v_max
    delta_max = limits.delta_max
    v_dot_max = limits.v_dot_max
    delta_dot_max = limits.delta_dot_max

    if sys.n >= 5:
        # States: the speed and the steer angle
        set_state_bounds(sys.state, 3, 0.0, v_max)
        set_state_bounds(sys.state, 4, -delta_max, delta_max)

        # Inputs: the acceleration and the steer rate, on their own ports or on u
        if "a_x" in sys.inputs:
            set_port_bounds(sys.inputs["a_x"], [-v_dot_max], [v_dot_max])
            set_port_bounds(sys.inputs["delta_dot"], [-delta_dot_max], [delta_dot_max])
        elif "u" in sys.inputs:
            labels = getattr(sys.inputs["u"], "labels", [])
            if labels and labels[0] == "a_x":
                set_port_bounds(
                    sys.inputs["u"],
                    [-v_dot_max, -delta_dot_max],
                    [v_dot_max, delta_dot_max],
                )

    # A speed-and-steer rung: the input is the speed and the steer angle
    elif "u" in sys.inputs:
        set_port_bounds(sys.inputs["u"], [0.0, -delta_max], [v_max, delta_max])


def apply_dynamic_six(sys: Any, profile: CarProfile) -> None:
    """Bound the six-state dynamic bicycle's wheel-rate and steer inputs."""
    limits = profile.limits
    delta_max = limits.delta_max
    w_rear_max = limits.w_rear_max

    # Inputs: the rear wheel rate and the steer angle, on their own ports or on u
    if "w_rear" in sys.inputs:
        set_port_bounds(sys.inputs["w_rear"], [0.0], [w_rear_max])
        set_port_bounds(sys.inputs["delta"], [-delta_max], [delta_max])
    elif "u" in sys.inputs and sys.inputs["u"].dim == 2:
        labels = getattr(sys.inputs["u"], "labels", [])
        if labels and labels[0] == "w_rear":
            set_port_bounds(sys.inputs["u"], [0.0, -delta_max], [w_rear_max, delta_max])


def apply_dynamic_eight(sys: Any, profile: CarProfile) -> None:
    """Bound an eight-state rung: its wheel-rate and steer states, its rate or torque inputs."""
    limits = profile.limits
    delta_max = limits.delta_max
    w_rear_max = limits.w_rear_max
    w_rear_dot_max = limits.w_rear_dot_max
    delta_dot_max = limits.delta_dot_max
    tau_max = limits.tau_rear_max
    tau_min = limits.tau_rear_min

    # States: the rear wheel rate and the steer angle
    set_state_bounds(sys.state, 6, 0.0, w_rear_max)
    set_state_bounds(sys.state, 7, -delta_max, delta_max)

    # Inputs: the wheel acceleration or the rear torque, and the steer rate, on their
    # own ports or on u
    if "w_rear_dot" in sys.inputs:
        set_port_bounds(sys.inputs["w_rear_dot"], [-w_rear_dot_max], [w_rear_dot_max])
        set_port_bounds(sys.inputs["delta_dot"], [-delta_dot_max], [delta_dot_max])
    elif "tau_rear" in sys.inputs:
        set_port_bounds(sys.inputs["tau_rear"], [tau_min], [tau_max])
        set_port_bounds(sys.inputs["delta_dot"], [-delta_dot_max], [delta_dot_max])
    elif "u" in sys.inputs:
        labels = getattr(sys.inputs["u"], "labels", [])
        if labels and labels[0] == "w_rear_dot":
            set_port_bounds(
                sys.inputs["u"],
                [-w_rear_dot_max, -delta_dot_max],
                [w_rear_dot_max, delta_dot_max],
            )
        elif labels and labels[0] == "tau_rear":
            set_port_bounds(
                sys.inputs["u"], [tau_min, -delta_dot_max], [tau_max, delta_dot_max]
            )


def apply_dynamic_nine(sys: Any, profile: CarProfile) -> None:
    """Bound a nine-state rung: its wheel-rate and steer states, its torque or power commands."""
    limits = profile.limits
    delta_max = limits.delta_max
    w_rear_max = limits.w_rear_max
    tau_max = limits.tau_rear_max
    tau_min = limits.tau_rear_min
    P_peak = profile.engine_power_peak

    # States: the rear wheel rate and the steer angle
    set_state_bounds(sys.state, 6, 0.0, w_rear_max)
    set_state_bounds(sys.state, 7, -delta_max, delta_max)

    # Inputs: the torque or the power command, and the steer command, on their own
    # ports or on u; a power command also bounds the power state
    if "tau_cmd" in sys.inputs:
        set_port_bounds(sys.inputs["tau_cmd"], [tau_min], [tau_max])
        set_port_bounds(sys.inputs["delta_cmd"], [-delta_max], [delta_max])
    elif "P_cmd" in sys.inputs:
        set_port_bounds(sys.inputs["P_cmd"], [-P_peak], [P_peak])
        set_port_bounds(sys.inputs["delta_cmd"], [-delta_max], [delta_max])
        set_state_bounds(sys.state, 8, -P_peak, P_peak)
    elif "u" in sys.inputs:
        labels = getattr(sys.inputs["u"], "labels", [])
        if labels and labels[0] == "tau_cmd":
            set_port_bounds(
                sys.inputs["u"], [tau_min, -delta_max], [tau_max, delta_max]
            )
        elif labels and labels[0] == "P_cmd":
            set_port_bounds(sys.inputs["u"], [-P_peak, -delta_max], [P_peak, delta_max])
            set_state_bounds(sys.state, 8, -P_peak, P_peak)


# The bound applicator of each plant class, by class name
_APPLY_BY_CLASS: dict[str, Any] = {
    "KinematicBicycle": apply_kinematic,
    "BicycleKin": apply_kinematic,
    "BicycleAcc": apply_kinematic,
    "DynamicBicycle": apply_dynamic_six,
    "BicycleDynRate": apply_dynamic_eight,
    "BicycleDynTauRate": apply_dynamic_eight,
    "BicycleDynServo": apply_dynamic_nine,
    "BicycleDynEngine": apply_dynamic_nine,
}


def set_state_bounds(state: Any, i: int, lower: float, upper: float) -> None:
    """Bound one state component: ``lower ≤ x_i ≤ upper``."""
    state.lower_bound[i] = lower
    state.upper_bound[i] = upper


def set_port_bounds(
    port: Any, lower: float | np.ndarray, upper: float | np.ndarray
) -> None:
    """Bound an input port: ``lower ≤ u ≤ upper``, as float arrays."""
    port.lower_bound = np.asarray(lower, dtype=float)
    port.upper_bound = np.asarray(upper, dtype=float)


# The limits of a profile


def make_limits(
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
    # τ_max = P_max / (N ω_nom), ω_nom = max(|v_nom| / r_r, 1): the power-limited torque
    tau_max = nominal_actuator_torque(
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        r_r=r_r,
        v_nom=v_nom,
    )

    # ẇ_r,max = τ_max / (J_w + m r_r²), the no-slip wheel/vehicle couple
    w_rear_dot = nominal_actuator_wheel_accel(
        mass=mass,
        r_r=r_r,
        Jw_rear=Jw_rear,
        engine_power_peak=engine_power_peak,
        transmission_ratio=transmission_ratio,
        v_nom=v_nom,
    )

    # a_long,max = r_r ẇ_r,max
    a_long = r_r * w_rear_dot

    # The ratings, rounded; ω_r,max = v_x,max / r_r
    w_rear_dot_r = round(w_rear_dot)
    w_rear_max = round(vx_max / r_r)
    tau_r = round_torque(tau_max)
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


def round_torque(tau: float) -> float:
    """A torque rating: to 10 Nm from 100 Nm up, to 0.1 Nm below."""
    if abs(tau) >= 100.0:
        return float(round(tau, -1))

    return float(round(tau, 1))


# The registry of named profiles, built once at import from the profiles above
CAR_PROFILES: dict[str, CarProfile] = {
    "passenger_car": passenger_car_profile(),
    "racecar": racecar_profile(),
    "udes_1_5": udes_1_5_profile(),
}
