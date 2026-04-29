import numpy as np

from minilink.core.system import DynamicSystem, StaticSystem


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)

        self.params = {"k": 1.0}

        self.name = "Integrator"

        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        k = params["k"]

        dx = np.zeros(self.n)
        dx[0] = k * u[0]
        return dx

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


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

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]

        r = u[0]
        y = u[1]

        u_cmd = Kp * (r - y)
        return np.array([u_cmd])
