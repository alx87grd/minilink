"""Actuator allocation: rear force and steer to plant inputs."""

import numpy as np

from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


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

        # Parameters: the rear wheel radius, the engine's peak power, the gear ratio
        self.r_r = float(r_r)
        self.engine_power_peak = float(engine_power_peak)
        self.transmission_ratio = float(transmission_ratio)

        # Ports: the servo commands and the plant output in, the plant's inputs out
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
        r_r = self.r_r
        engine_power_peak = self.engine_power_peak
        transmission_ratio = self.transmission_ratio

        F_rear = float(u[0])
        w_rear = float(u[8])  # The plant's y[6]: u stacks F_rear, delta, then y

        # τ_required = F_rear r_r, the rear torque the force asks for
        tau_rear_required = F_rear * r_r

        # τ_max = P_peak / ω_engine, the power-limited torque, ω_engine = max(N ω_r, 1)
        w_engine = max(w_rear * transmission_ratio, 1.0)
        tau_max = engine_power_peak / w_engine

        # throttle = clip(τ_required / τ_max, 0, 1)
        throttle = float(np.clip(tau_rear_required / tau_max, 0.0, 1.0))

        throttle_out = np.array([throttle], dtype=float)

        return throttle_out

    def h_delta(self, x, u, t=0.0, params=None):
        delta = float(u[1])

        # The steer passes through: δ_out = δ
        delta_out = np.array([delta], dtype=float)

        return delta_out
