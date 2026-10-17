"""Motor maps: the servo cascade's acceleration and yaw-rate references as throttle and steer."""

import numpy as np

from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class AccelerationToThrottle(System):
    """Map target longitudinal acceleration to normalized throttle.

    Uses rear wheel rate and ``transmission_ratio`` so the power-limited
    torque estimate matches :meth:`EngineBicycle.engine_torque_from_throttle`.
    """

    def __init__(
        self,
        r_r: float,
        engine_power_peak: float,
        mass: float,
        transmission_ratio: float = 1.0,
    ):
        super().__init__()
        self.name = "Acceleration to throttle"

        # Parameters: the rear wheel radius [m], the engine's peak power [W], the
        # mass [kg] and the gear ratio
        self.r_r = float(r_r)
        self.engine_power_peak = float(engine_power_peak)
        self.mass = float(mass)
        self.transmission_ratio = float(transmission_ratio)

        # Ports: the acceleration reference and the rear wheel rate in, the throttle out
        self.add_input_port("accel_ref", nominal_value=np.array([0.0]))
        self.add_input_port("w_rear", nominal_value=np.array([0.0]))
        self.add_output_port(
            "throttle",
            dim=1,
            function=self.h_throttle,
            dependencies=("accel_ref", "w_rear"),
        )

    def h_throttle(self, x, u, t=0.0, params=None):
        r_r = self.r_r
        engine_power_peak = self.engine_power_peak
        mass = self.mass
        transmission_ratio = self.transmission_ratio

        accel_ref = float(u[0])
        w_rear = float(u[1])

        # τ_required = F_rear r_r, the rear torque of the force F_rear = m a_ref
        F_rear = mass * accel_ref
        tau_rear_required = F_rear * r_r

        # τ_max = P_peak / ω_engine, the power-limited torque, ω_engine = max(N ω_r, 1)
        w_engine = max(w_rear * transmission_ratio, 1.0)
        max_engine_torque = engine_power_peak / w_engine

        # throttle = clip(τ_required / τ_max, 0, 1)
        throttle = tau_rear_required / max_engine_torque
        throttle = np.clip(throttle, 0.0, 1.0)

        throttle_out = np.array([throttle], dtype=float)

        return throttle_out


class YawRateToSteering(System):
    """Kinematic map from desired yaw rate to steer feedforward."""

    def __init__(
        self,
        max_steer: float,
        min_steer: float,
        length_vehicle: float,
        name: str = "Yaw rate to steering",
    ):
        super().__init__(0)
        self.name = name

        # Parameters: the wheelbase [m] and the steer limits [rad]
        self.L = float(length_vehicle)
        self.max_steer = float(max_steer)
        self.min_steer = float(min_steer)

        # Ports: the yaw-rate reference and the forward speed in, the steer out
        self.add_input_port("yawrate_ref", nominal_value=np.array([0.0]))
        self.add_input_port("vx", nominal_value=np.array([0.0]))
        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=("yawrate_ref", "vx"),
        )

    def h_delta(self, x, u, t=0.0, params=None):
        L = self.L
        max_steer = self.max_steer
        min_steer = self.min_steer

        yawrate_ref = float(u[0])
        vx = float(u[1])

        # δ = atan(ψ̇_ref L / v_x), the kinematic bicycle's steer, with v_x ≥ 10⁻⁶
        vx_num = max(vx, 1e-6)
        kinematic_factor = vx_num / L
        delta = np.arctan(yawrate_ref / kinematic_factor)

        # Steer limits: δ_min ≤ δ ≤ δ_max
        delta = np.clip(delta, a_min=min_steer, a_max=max_steer)

        delta_out = np.array([delta], dtype=float)

        return delta_out
