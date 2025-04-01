import numpy as np
from framework import System


######################################################################
class Source(System):

    def __init__(self, p):

        System.__init__(self, 0, 0, p)

        self.name = "Source"
        self.params = {"value": np.zeros(p)}

        self.inputs = {}
        self.outputs = {}
        self.add_output_port(self.p, "y", function=self.h)

    ###################################################################
    def h(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        y = params["value"]
        return y


######################################################################
class Step(Source):

    def __init__(
        self, initial_value=np.zeros(1), final_value=np.zeros(1), step_time=1.0
    ):

        p = initial_value.shape[0]
        Source.__init__(self, p)

        self.name = "Step"
        self.params = {
            "initial_value": initial_value,
            "final_value": final_value,
            "step_time": step_time,
        }

    ###################################################################
    def h(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        if t < params["step_time"]:
            y = params["initial_value"]
        else:
            y = params["final_value"]

        return y


######################################################################
class WhiteNoise(Source):

    def __init__(self, p=1):

        Source.__init__(self, p)

        self.random_generator = np.random.default_rng()

        self.name = "WhiteNoise"
        self.params = {
            "var": 1.0,
            "mean": 0.0,
            "seed": 0,
        }

    ###################################################################
    def h(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        seed = params["seed"] + int(t * 1000000000)
        mu = params["mean"]
        sigma = np.sqrt(params["var"])

        random_generator = np.random.default_rng(seed)
        # ToDo: check if this is the best way to generate random numbers quickly
        y = random_generator.normal(mu, sigma, self.p)

        return y
