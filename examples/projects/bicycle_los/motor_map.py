"""Static maps for the bicycle servo cascade."""

import numpy as np

from minilink.core.system import System


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
        self.r_r = float(r_r)
        self.engine_power_peak = float(engine_power_peak)
        self.mass = float(mass)
        self.transmission_ratio = float(transmission_ratio)

        self.add_input_port("accel_ref", nominal_value=np.array([0.0]))
        self.add_input_port("w_rear", nominal_value=np.array([0.0]))
        self.add_output_port(
            "throttle",
            dim=1,
            function=self.h_throttle,
            dependencies=("accel_ref", "w_rear"),
        )

    def h_throttle(self, x, u, t=0.0, params=None):
        accel_ref = float(u[0])
        w_rear = float(u[1])

        F_rear = self.mass * accel_ref
        tau_rear_required = F_rear * self.r_r

        w_engine = max(w_rear * self.transmission_ratio, 1.0)
        max_engine_torque = self.engine_power_peak / w_engine
        throttle = tau_rear_required / max_engine_torque
        throttle = np.clip(throttle, 0.0, 1.0)

        return np.array([throttle], dtype=float)


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
        self.L = float(length_vehicle)
        self.max_steer = float(max_steer)
        self.min_steer = float(min_steer)

        self.add_input_port("yawrate_ref", nominal_value=np.array([0.0]))
        self.add_input_port("vx", nominal_value=np.array([0.0]))
        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=("yawrate_ref", "vx"),
        )

    def h_delta(self, x, u, t=0.0, params=None):
        yawrate_ref = float(u[0])
        vx = float(u[1])

        vx_num = max(vx, 1e-6)
        kinematic_factor = vx_num / self.L
        delta = np.arctan(yawrate_ref / kinematic_factor)
        delta = np.clip(delta, a_min=self.min_steer, a_max=self.max_steer)

        return np.array([delta], dtype=float)
