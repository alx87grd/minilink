import numpy as np
from scipy.interpolate import interp1d

from minilink.core.framework import System


def _array_module(a):
    """Return ``jax.numpy`` if *a* is a JAX tracer/array, else ``numpy``."""
    if type(a).__module__.startswith("jax"):
        import jax.numpy as jnp
        return jnp
    return np


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

    ###################################################################
    def show_signal(self, t0=None, tf=None, n_pts=1000, ax=None):
        """
        Plot the source output signal over a time range.

        Parameters
        ----------
        t0 : float, optional
            Start time. Defaults to ``self.params["t0"]`` when available, else 0.0.
        tf : float, optional
            End time. Defaults to ``self.params["tf"]`` when available, else 10.0.
        n_pts : int, optional
            Number of evaluation points used for plotting.
        ax : matplotlib.axes.Axes, optional
            Existing axis to draw on. If None, a new figure is created.
        """
        import matplotlib.pyplot as plt

        self.refresh()

        if t0 is None:
            t0 = 0.0
        if tf is None:
            tf = 10.0
        if n_pts < 2:
            raise ValueError("n_pts must be >= 2")

        t = np.linspace(t0, tf, int(n_pts))
        y = np.array([self.h(np.array([]), np.array([]), ti) for ti in t]).T

        if ax is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 3))
        else:
            fig = ax.figure

        for i in range(self.p):
            label = f"{self.name}[{i}]" if self.p > 1 else self.name
            ax.plot(t, y[i, :], linewidth=1.2, label=label)

        ax.set_xlabel("time [s]")
        ax.set_ylabel("value")
        ax.set_title(f"{self.name} signal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        return fig, ax


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

        xp = _array_module(t)
        return xp.where(
            t < params["step_time"],
            params["initial_value"],
            params["final_value"],
        )


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
            "t0": -100.0,
            "tf": 100.0,
        }

        # Keep source ready to use without an explicit refresh() call.
        self.refresh()

    ###################################################################
    def refresh(self):
        t0 = float(self.params["t0"])
        tf = float(self.params["tf"])
        if tf < t0:
            t0, tf = tf, t0

        sample_period = float(self.params["sample_period"])

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
        white[:, 0] = mean
        white[:, -1] = mean

        self._interpolators = [
            interp1d(
                noise_time,
                white[i, :],
                kind="linear",
                bounds_error=False,
                fill_value=(mean, mean),
                assume_sorted=True,
            )
            for i in range(self.p)
        ]

    ###################################################################
    def h(self, x, u, t=0, params=None):

        if params is None:
            params = self.params
        else:
            raise ValueError(
                "The block needs to be refreshed to reflect changes in parameters"
            )

        y = np.array([interp(float(t)) for interp in self._interpolators])

        return y

    ###################################################################


if __name__ == "__main__":
    noise = WhiteNoise(1)
    # Baseline
    noise.params["t0"] = 0.0
    noise.params["tf"] = 10.0
    noise.params["sample_period"] = 0.01
    noise.params["mean"] = 0.0
    noise.params["var"] = 1.0
    noise.params["seed"] = 1

    fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
    ax.set_title("Baseline")

    # Change one parameter at a time to visualize each effect.
    demo_changes = [
        ("mean", 10.0),
        ("var", 8.0),
        ("sample_period", 0.05),
        ("sample_period", 0.2),
        ("sample_period", 0.5),
        ("sample_period", 1.0),
        ("sample_period", 2.0),
        ("sample_period", 5.0),
    ]

    for key, value in demo_changes:
        noise.params[key] = value
        noise.refresh()
        fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
        ax.set_title(f"Changed {key} -> {value}")
