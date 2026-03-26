import numpy as np
from scipy.interpolate import interp1d

from minilink.core.framework import System


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

        self.name = "WhiteNoise"
        self.params = {
            "var": 1.0,
            "mean": 0.0,
            "seed": 0,
            "sample_period": 0.01,
        }

        self._refresh_config = None
        self._noise_time = None
        self._noise_samples = None
        self._interpolators = None

        # Keep source ready to use without an explicit refresh() call.
        self.refresh()

    ###################################################################
    def refresh(self, t0=-100.0, tf=100.0, sample_period=None):
        if tf < t0:
            t0, tf = tf, t0

        if sample_period is None:
            sample_period = float(self.params["sample_period"])
        else:
            sample_period = float(sample_period)

        if sample_period <= 0:
            raise ValueError("sample_period must be > 0")

        mean = float(self.params["mean"])
        var = max(0.0, float(self.params["var"]))
        seed = int(self.params["seed"])
        sigma = np.sqrt(var)

        n_steps = int(np.ceil((tf - t0) / sample_period)) + 1
        noise_time = np.linspace(t0, tf, n_steps)

        rng = np.random.default_rng(seed)
        white = rng.normal(loc=mean, scale=sigma, size=(self.p, n_steps))

        # Keep continuity shaping internal and tied to sample_period.
        tau = sample_period
        alpha = np.exp(-sample_period / tau)
        shaped = np.empty_like(white)
        shaped[:, 0] = white[:, 0]
        for k in range(1, n_steps):
            shaped[:, k] = alpha * shaped[:, k - 1] + (1.0 - alpha) * white[:, k]

        self._interpolators = [
            interp1d(
                noise_time,
                shaped[i, :],
                kind="linear",
                bounds_error=False,
                fill_value=(shaped[i, 0], shaped[i, -1]),
                assume_sorted=True,
            )
            for i in range(self.p)
        ]
        self._noise_time = noise_time
        self._noise_samples = shaped
        self._refresh_config = {
            "t0": float(t0),
            "tf": float(tf),
            "sample_period": float(sample_period),
            "mean": mean,
            "var": var,
            "seed": seed,
        }

    ###################################################################
    def h(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        y = np.array([interp(float(t)) for interp in self._interpolators])

        return y
