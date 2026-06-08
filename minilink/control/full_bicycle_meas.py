import numpy as np

from minilink.core.system import System


class BicycleMeasurement(System):
    """A simple measurement block that outputs a single element of the input vector."""

    def __init__(self, name: str, y_size: int = 10):
        super().__init__(0)

        self.name = name

        self.y_size = int(y_size)

        self.inputs = {}
        self.add_input_port(
            "y",
            nominal_value=np.zeros(self.y_size),
        )

        self.outputs = {}
        self.add_output_port(
            "X_meas",
            dim=1,
            function=self.x_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "y_meas",
            dim=1,
            function=self.y_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "theta_meas",
            dim=1,
            function=self.theta_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "vx_meas",
            dim=1,
            function=self.vx_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "vy_meas",
            dim=1,
            function=self.vy_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "r_meas",
            dim=1,
            function=self.r_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "w_r_meas",
            dim=1,
            function=self.w_r_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "w_f_meas",
            dim=1,
            function=self.w_f_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "torque_eng_meas",
            dim=1,
            function=self.torque_eng_meas,
            dependencies=["y"],
        )

        self.add_output_port(
            "delta_meas",
            dim=1,
            function=self.delta_meas,
            dependencies=["y"],
        )

    def x_meas(self, x, u, t=0.0, params=None):
        return np.array([u[0]], dtype=float)

    def y_meas(self, x, u, t=0.0, params=None):
        return np.array([u[1]], dtype=float)

    def theta_meas(self, x, u, t=0.0, params=None):
        return np.array([u[2]], dtype=float)

    def vx_meas(self, x, u, t=0.0, params=None):
        return np.array([u[3]], dtype=float)

    def vy_meas(self, x, u, t=0.0, params=None):
        return np.array([u[4]], dtype=float)

    def r_meas(self, x, u, t=0.0, params=None):
        return np.array([u[5]], dtype=float)

    def w_r_meas(self, x, u, t=0.0, params=None):
        return np.array([u[6]], dtype=float)

    def w_f_meas(self, x, u, t=0.0, params=None):
        return np.array([u[7]], dtype=float)

    def torque_eng_meas(self, x, u, t=0.0, params=None):
        return np.array([u[8]], dtype=float)

    def delta_meas(self, x, u, t=0.0, params=None):
        return np.array([u[9]], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
