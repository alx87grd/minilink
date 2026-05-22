import numpy as np

from minilink.core.system import System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)


class ThrMap(System):
    """Map desired rear longitudinal force to normalized throttle.

    Input
    -----
    F_rear
        Desired rear longitudinal tire force [N]

    Output
    ------
    thr
        Normalized throttle command in [0, 1]

    Mapping
    -------
    tau_rear_required = F_rear * r_r

    thr = tau_rear_required / tau_rear_max
    """

    def __init__(
        self,
        vehicle: DynamicBicycleRearWheelDriveEngine,
        name: str = "Throttle map",
    ):
        super().__init__(0, 2, 1)

        self.name = name

        self.r_r = float(vehicle.r_r)
        self.engine_power_peak = vehicle.engine_power_peak
        self.transmission_ratio = vehicle.transmission_ratio

        self.inputs = {}
        self.add_input_port(
            1,
            "F_rear",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "w_rear",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "thr",
            function=self.h_thr,
            dependencies=["F_rear", "w_rear"],
        )

    def h_thr(self, x, u, t=0.0, params=None):
        F_rear = float(u[0])
        w_rear = float(u[1])

        # Ici j'assume que la roue ne glisse pas
        tau_rear_required = F_rear * self.r_r

        w_rear_num = max(w_rear, 1.0)
        w_moteur = w_rear_num * self.transmission_ratio

        # print(f"w_rear: {w_rear}, w_rear_num: {w_rear_num}")
        max_engine_torque = self.engine_power_peak / w_moteur

        thr = tau_rear_required / max_engine_torque

        thr = np.clip(thr, 0.0, 1.0)

        # print(f"thr: {thr}")
        # print(
        #     f"tau_rear_required: {tau_rear_required}, max_engine_torque: {max_engine_torque}rpm, thr: {thr}"
        # )

        # print(f"w_rear: {w_rear}, w_rear_num: {w_rear_num}, w_moteur: {w_moteur}")
        # print(f"F_rear: {F_rear}")

        return np.array([thr], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class AccToRearForce(System):
    """Map desired longitudinal acceleration to rear wheel longitudinal force.

    Input
    -----
    acc
        Desired longitudinal acceleration [m/s^2]

    Output
    ------
    F_rear
        Desired rear longitudinal tire force [N]

    Mapping
    -------
    F_rear = mass * acc
    """

    def __init__(
        self,
        vehicle: DynamicBicycleRearWheelDriveEngine,
        name: str = "Acceleration to rear force",
    ):
        # 0 states
        # 1 scalar input: acc
        # 1 scalar output: F_rear
        super().__init__(0, 1, 1)

        self.name = name

        self.mass = float(vehicle.mass)

        self.inputs = {}

        self.add_input_port(
            1,
            "acc_targ",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "F_rear",
            function=self.h_force,
            dependencies=["acc_targ"],
        )

    def h_force(self, x, u, t=0.0, params=None):
        acc_targ = float(u[0])

        F_rear = self.mass * acc_targ
        return np.array([F_rear], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
