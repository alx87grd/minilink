from minilink.core.framework import DynamicSystem, StaticSystem
import numpy as np


######################################################################
class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(2, 1, 2)

        self.params = {"g": 9.81, "m": 1.0, "l": 1.0}

        self.name = "Pendulum"

        self.state.labels = ["theta", "theta_dot"]
        self.state.units = ["rad", "rad/s"]

        self.inputs = {}
        self.add_input_port(1, "u", nominal_value=np.array([0.0]))
        self.add_input_port(1, "w", nominal_value=np.array([0.0]))
        self.add_input_port(1, "v", nominal_value=np.array([0.0]))
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]

        self.outputs = {}
        self.add_output_port(self.p, "y", function=self.h, dependencies=["v"])

    ######################################################################
    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        g = params["g"]
        m = params["m"]
        l = params["l"]

        theta = x[0]

        signals = self.get_port_values_from_u(u)
        u = signals["u"][0]
        w = signals["w"][0]

        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -g / l * np.sin(theta) + 1 / (m * l**2) * (u + w)

        return dx

    ######################################################################
    def h(self, x, u, t=0, params=None):

        signals = self.get_port_values_from_u(u)
        v = signals["v"]

        y = np.zeros(self.p)

        y[0] = x[0] + v[0]
        y[1] = x[1] + v[0]

        return y


######################################################################
class PDController(StaticSystem):
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


######################################################################
class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)

        self.params = {"k": 1.0}

        self.name = "Integrator"

        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params
        k = params["k"]

        dx = np.zeros(self.n)
        dx[0] = k * u[0]

        return dx

    def h(self, x, u, t=0, params=None):

        y = np.zeros(self.p)
        y[0] = x[0]

        return y


######################################################################
class PropController(StaticSystem):
    def __init__(self):
        super().__init__(2, 1)

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(1, "y", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])

    ######################################################################
    def ctl(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        Kp = params["Kp"]

        r = u[0]
        y = u[1]

        u = Kp * (r - y)

        u = np.array([u])

        return u
