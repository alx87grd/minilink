"""Longitudinal-acceleration estimator for the bicycle vehicle model."""

import numpy as np

from minilink.core.system import DynamicSystem


class AccelerationMeasurement(DynamicSystem):
    """Estimate longitudinal acceleration from the vehicle longitudinal speed.

    For ``DynamicBicycleRearWheelDriveEngine`` the output vector is::

        y = [
            X, Y, theta,
            phi_rear, phi_front,
            vx, vy, r,
            w_rear, w_front,
            tau_engine, delta_act
        ]

    so ``vx`` sits at index 5. Acceleration is estimated as a filtered
    derivative ``a_x ~= (vx - vx_filt) / tau`` where ``vx_filt`` is a
    first-order filtered version of ``vx``.

    Parameters
    ----------
    y_size : int
        Dimension of the incoming vehicle output vector.
    speed_index : int
        Index of the longitudinal speed component in the output vector.
    tau : float
        Filter time constant.
    """

    def __init__(
        self,
        y_size: int = 12,
        speed_index: int = 5,
        tau: float = 0.05,
    ):
        super().__init__(1)

        self.name = "Acceleration measurement"

        self.y_size = int(y_size)
        self.speed_index = int(speed_index)
        self.tau = float(tau)

        self.state.labels = ["vx_filt"]
        self.state.units = ["m/s"]
        self.x0 = np.array([0.0], dtype=float)

        self.inputs = {}
        self.add_input_port("y", nominal_value=np.zeros(self.y_size))

        self.outputs = {}
        self.add_output_port(
            "meas",
            dim=1,
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
        return np.array([acc], dtype=float)
