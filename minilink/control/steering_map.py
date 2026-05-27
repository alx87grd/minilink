import numpy as np

from minilink.core.system import System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)


class AngularSpeedToSteeringMap(System):
    """Map desired longitudinal acceleration to rear wheel longitudinal force.

    Input
    -----
    r
        Desired angular speed [rad/s]

    Output
    ------
    Steering
        Steering angle to produce r [rad]

    Mapping
    -------
    F_rear = mass * acc
    """

    def __init__(
        self,
        vehicle: DynamicBicycleRearWheelDriveEngine,
        name: str = "Angular speed to steering",
    ):
        # 0 states
        # 1 scalar input: acc
        # 1 scalar output: F_rear
        super().__init__(0, 2, 1)

        self.name = name
        self.L = vehicle.L
        self.max_steer = vehicle.max_steer
        self.min_steer = vehicle.min_steer

        self.inputs = {}

        self.add_input_port(
            1,
            "r_targ",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "vx_meas",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "delta",
            function=self.h_force,
            dependencies=["r_targ", "vx_meas"],
        )

    def h_force(self, x, u, t=0.0, params=None):
        r_targ = float(u[0])
        vx_meas = float(u[1])

        vx_meas_num = max(vx_meas, 1e-6)

        cinematic_factor = vx_meas_num / self.L
        delta = np.arctan(r_targ / cinematic_factor)

        delta = np.clip(delta, max=self.max_steer, min=self.min_steer)
        return np.array([delta], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
