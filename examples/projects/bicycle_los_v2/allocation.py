"""Actuator allocation: rear force and steer to plant inputs."""

import numpy as np

from minilink.core.system import System


class Allocation(System):
    """Map rear longitudinal force to throttle; pass steer through.

    ::

        tau_rear_required = F_rear * r_r
        w_engine          = max(w_rear * transmission_ratio, 1.0)
        tau_max           = engine_power_peak / w_engine
        throttle          = clip(tau_rear_required / tau_max, 0, 1)
        delta_out         = delta

    Ports ``F_rear``, ``delta``, ``y`` → ``throttle``, ``delta``.
    """

    def __init__(
        self,
        r_r: float,
        engine_power_peak: float,
        transmission_ratio: float = 1.0,
        name: str = "Allocation",
    ):
        super().__init__(0)
        self.name = name
        self.r_r = float(r_r)
        self.engine_power_peak = float(engine_power_peak)
        self.transmission_ratio = float(transmission_ratio)

        self.add_input_port("F_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "throttle",
            dim=1,
            function=self.h_throttle,
            dependencies=("F_rear", "delta", "y"),
        )
        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=("F_rear", "delta", "y"),
        )

    def h_throttle(self, x, u, t=0.0, params=None):
        F_rear = float(u[0])
        w_rear = float(u[8])  # y[6] after F_rear(1) + delta(1)

        tau_rear_required = F_rear * self.r_r
        w_engine = max(w_rear * self.transmission_ratio, 1.0)
        tau_max = self.engine_power_peak / w_engine
        throttle = tau_rear_required / tau_max
        throttle = float(np.clip(throttle, 0.0, 1.0))
        return np.array([throttle], dtype=float)

    def h_delta(self, x, u, t=0.0, params=None):
        delta = float(u[1])
        return np.array([delta], dtype=float)
