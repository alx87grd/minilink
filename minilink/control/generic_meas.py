import numpy as np

from minilink.core.system import DynamicSystem, System


class Measurement(System):
    """Extract longitudinal speed from vehicle output vector.

    For DynamicBicycleRearWheelDriveEngine:

        y = [
            X, Y, theta,
            phi_rear, phi_front,
            vx, vy, r,
            w_rear, w_front,
            tau_engine, delta_act
        ]

    """

    def __init__(self, name: str, y_size: int = 12, index: int = 5, show=False):
        super().__init__(0, y_size, 1)

        self.name = name
        self.print = show

        self.y_size = int(y_size)
        self.index = int(index)

        self.inputs = {}
        self.add_input_port(
            self.y_size,
            "y",
            nominal_value=np.zeros(self.y_size),
        )

        self.outputs = {}
        self.add_output_port(
            1,
            "meas",
            function=self.h_meas,
            dependencies=["y"],
        )

    def h_meas(self, x, u, t=0.0, params=None):
        if self.print:
            print(f"Meas: {u[self.index]}")
        return np.array([u[self.index]], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class AccelerationMeasurement(DynamicSystem):
    """Estimate longitudinal acceleration from vehicle longitudinal speed.

    For DynamicBicycleRearWheelDriveEngine:

        y = [
            X, Y, theta,
            phi_rear, phi_front,
            vx, vy, r,
            w_rear, w_front,
            tau_engine, delta_act
        ]

    Therefore vx is at index 5.

    This block estimates acceleration as a filtered derivative:

        a_x ~= (vx - vx_filt) / tau

    where vx_filt is a first-order filtered version of vx.
    """

    def __init__(
        self,
        y_size: int = 12,
        speed_index: int = 5,
        tau: float = 0.05,
    ):
        # 1 state:
        # x[0] = filtered vx
        #
        # Input:
        # y = full vehicle output vector
        #
        # Output:
        # meas = estimated longitudinal acceleration
        super().__init__(1, y_size, 1)

        self.name = "Acceleration measurement"

        self.y_size = int(y_size)
        self.speed_index = int(speed_index)
        self.tau = float(tau)

        self.state.labels = ["vx_filt"]
        self.state.units = ["m/s"]

        self.x0 = np.array([0.0], dtype=float)

        self.inputs = {}
        self.add_input_port(
            self.y_size,
            "y",
            nominal_value=np.zeros(self.y_size),
        )

        self.outputs = {}
        self.add_output_port(
            1,
            "meas",
            function=self.h_meas,
            dependencies=["y"],
        )

    def f(self, x, u, t=0.0, params=None):
        vx = float(u[self.speed_index])
        vx_filt = float(x[0])

        tau = max(self.tau, 1e-9)

        d_vx_filt = (vx - vx_filt) / tau

        return np.array([d_vx_filt], dtype=float)

    def h_meas(self, x, u, t=0.0, params=None):
        vx = float(u[self.speed_index])
        vx_filt = float(x[0])

        tau = max(self.tau, 1e-9)

        acc = (vx - vx_filt) / tau

        # print(f"acc: {acc}")
        return np.array([acc], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
