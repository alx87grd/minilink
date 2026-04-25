import numpy as np

from minilink.core.framework import DynamicSystem, StaticSystem


######################################################################
class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)

        self.params = {"k": 1.0}

        self.name = "Integrator"

        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    ######################################################################
    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params
        k = params["k"]

        return u[: self.n] * k

    #######################################################################
    def h(self, x, u, t=0, params=None):

        return x[: self.p]


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

        return Kp * (u[0:1] - u[1:2])
