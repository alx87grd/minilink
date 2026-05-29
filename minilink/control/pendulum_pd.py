import numpy as np

from minilink.core.system import StaticSystem


class PendulumPDController(StaticSystem):
    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port(
            "y",
            nominal_value=[0.0, 0.0],
            labels=["theta", "theta_dot"],
            units=["rad", "rad/s"],
        )
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=["torque"],
            units=["Nm"],
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        r, y = self.get_port_values_from_u(u, "r", "y")
        theta = y[0]
        theta_dot = y[1]
        Kp = params["Kp"]
        Kd = params["Kd"]

        tau = Kp * (r[0] - theta) - Kd * theta_dot
        return np.array([tau])
