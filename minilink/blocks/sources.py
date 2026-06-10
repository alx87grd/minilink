import numpy as np
from scipy.interpolate import interp1d

from minilink.core.backends import array_module
from minilink.core.system import System


class Source(System):
    def __init__(self, p):

        System.__init__(self, 0)

        self.name = "Source"
        self.params = {"value": np.zeros(p)}

        self.add_output_port("y", dim=p, function=self.h)

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return params["value"]

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
            Existing axis to draw on. If None, a new figure is created and shown
            (same policy as
            :func:`minilink.graphical.signals.plot_time_signals`);
            if an axis is passed, the caller controls display.
        """
        import matplotlib.pyplot as plt

        from minilink.graphical.common.environment import (
            allow_tall_stacked_figures,
            is_blocking_needed,
        )
        from minilink.graphical.common.matplotlib_style import (
            DPI_FIGURE,
            FONT_SIZE,
            signal_stack_figsize,
            style_trajectory_subplot,
        )

        self.refresh()

        if t0 is None:
            t0 = 0.0
        if tf is None:
            tf = 10.0
        if n_pts < 2:
            raise ValueError("n_pts must be >= 2")

        t = np.linspace(t0, tf, int(n_pts))
        y = np.zeros((self.p, t.size), dtype=float)
        empty_x = np.array([])
        empty_u = np.array([])
        for i, ti in enumerate(t):
            y[:, i] = np.asarray(self.h(empty_x, empty_u, ti), dtype=float).reshape(
                self.p
            )

        _created = ax is None
        if _created:
            fig, ax = plt.subplots(
                1,
                1,
                figsize=signal_stack_figsize(
                    1, allow_tall=allow_tall_stacked_figures()
                ),
                dpi=DPI_FIGURE,
                frameon=True,
            )
            manager = getattr(fig.canvas, "manager", None)
            set_window_title = getattr(manager, "set_window_title", None)
            if callable(set_window_title):
                set_window_title("Signal: " + self.name)
        else:
            fig = ax.figure

        for i in range(self.p):
            label = f"{self.name}[{i}]" if self.p > 1 else self.name
            ax.plot(t, y[i, :], "b", linewidth=1.5, alpha=0.8, label=label)

        ax.set_ylabel("value", fontsize=FONT_SIZE, multialignment="center")
        style_trajectory_subplot(ax)
        ax.legend(loc="upper right")
        ax.set_xlabel("Time [s]", fontsize=FONT_SIZE)
        ax.set_title(f"{self.name} signal", fontsize=FONT_SIZE)
        if _created:
            plt.show(block=is_blocking_needed())

        return fig, ax


class Step(Source):
    def __init__(self, initial_value=None, final_value=None, step_time=1.0):
        if initial_value is None:
            initial_value = np.zeros(1)
        if final_value is None:
            final_value = np.zeros(1)

        initial_value = np.asarray(initial_value, dtype=float).reshape(-1)
        final_value = np.asarray(final_value, dtype=float).reshape(-1)
        if final_value.shape != initial_value.shape:
            raise ValueError("initial_value and final_value must have the same shape")

        p = initial_value.shape[0]
        Source.__init__(self, p)

        self.name = "Step"
        self.params = {
            "initial_value": initial_value,
            "final_value": final_value,
            "step_time": step_time,
        }

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        xp = array_module(t)
        return xp.where(
            t < params["step_time"],
            params["initial_value"],
            params["final_value"],
        )


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

        self._interpolators = []
        for i in range(self.p):
            interpolator = interp1d(
                noise_time,
                white[i, :],
                kind="linear",
                bounds_error=False,
                fill_value=(mean, mean),
                assume_sorted=True,
            )
            self._interpolators.append(interpolator)

    def h(self, x, u, t=0, params=None):
        if params is None:
            params = self.params
        else:
            raise ValueError(
                "The block needs to be refreshed to reflect changes in parameters"
            )

        y = np.zeros(self.p)
        for i, interpolator in enumerate(self._interpolators):
            y[i] = float(interpolator(float(t)))

        return y


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
