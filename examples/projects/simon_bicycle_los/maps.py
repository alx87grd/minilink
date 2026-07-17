"""Static maps for Simon's bicycle servo cascade."""

import numpy as np

from minilink.core.system import System


class AccToThr(System):
    """Map target longitudinal acceleration to normalized throttle."""

    def __init__(
        self,
        r_r: float,
        engine_power_peak: float,
        mass: float,
    ):
        super().__init__()
        self.name = "Acceleration to throttle map"
        self.r_r = r_r
        self.engine_power_peak = engine_power_peak
        self.mass = mass

        self.add_input_port("acc_targ", nominal_value=np.array([0.0]))
        self.add_input_port("w_motor", nominal_value=np.array([0.0]))
        self.add_output_port(
            "thr",
            dim=1,
            function=self.h_thr,
            dependencies=("acc_targ", "w_motor"),
        )

    def h_thr(self, x, u, t=0.0, params=None):
        acc_targ = float(u[0])
        w_motor = float(u[1])

        F_rear = self.mass * acc_targ
        tau_rear_required = F_rear * self.r_r
        w_motor_num = max(w_motor, 1.0)
        max_engine_torque = self.engine_power_peak / w_motor_num
        thr = tau_rear_required / max_engine_torque
        thr = np.clip(thr, 0.0, 1.0)

        return np.array([thr], dtype=float)


class AngularSpeedToSteeringMap(System):
    """Map desired yaw rate to a kinematic steering feedforward angle."""

    def __init__(
        self,
        max_steer: float,
        min_steer: float,
        length_vehicle: float,
        name: str = "Angular speed to steering",
    ):
        super().__init__(0)
        self.name = name
        self.L = length_vehicle
        self.max_steer = max_steer
        self.min_steer = min_steer

        self.add_input_port("r_targ", nominal_value=np.array([0.0]))
        self.add_input_port("vx_meas", nominal_value=np.array([0.0]))
        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=("r_targ", "vx_meas"),
        )

    def h_delta(self, x, u, t=0.0, params=None):
        r_targ = float(u[0])
        vx_meas = float(u[1])

        vx_meas_num = max(vx_meas, 1e-6)
        cinematic_factor = vx_meas_num / self.L
        delta = np.arctan(r_targ / cinematic_factor)
        delta = np.clip(delta, a_min=self.min_steer, a_max=self.max_steer)

        return np.array([delta], dtype=float)
