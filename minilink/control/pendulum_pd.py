import numpy as np

from minilink.core.framework import StaticSystem


class PendulumPDController(StaticSystem):
    def __init__(self):
        super().__init__(3, 1)

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(2, "y", nominal_value=np.array([0.0, 0.0]))
        self.inputs["y"].labels = ["theta", "theta_dot"]
        self.inputs["y"].units = ["rad", "rad/s"]

        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])
        self.outputs["u"].labels = ["torque"]
        self.outputs["u"].units = ["Nm"]

    ######################################################################
    def ctl(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        Kp = params["Kp"]
        Kd = params["Kd"]

        ref = u[0]
        theta = u[1]
        theta_dot = u[2]

        torque = Kp * (ref - theta) - Kd * theta_dot

        u = np.array([torque])

        return u
