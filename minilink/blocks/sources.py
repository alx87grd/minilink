"""Source blocks: constant, step, sine, white noise and a replayed trajectory as a signal."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.distributions import child_seed, sample_index, standard_normal
from minilink.core.options import require_one_of
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class Source(System):
    """Constant source: ``y = value`` for all ``t``.

    Also the base class for time-signal sources; subclasses override
    :meth:`h` with their own ``y = h(t; p)``. ``port`` names the output
    signal: ``y`` by default, or the input it drives (``Step(port="r")``,
    ``WhiteNoise(port="d")``), which is how ``autowire`` finds it.
    """

    def __init__(self, p, *, port="y"):
        super().__init__()

        # Parameters
        self.params = {"value": np.zeros(p)}

        # Ports: the one output, ``y`` by default, or the input it drives (``port="r"``)
        self.port = port
        self.add_output_port(port, dim=p, function=self.h)

    @property
    def dim(self):
        """The signal's dimension, whatever the output port is named."""
        return self.outputs[self.port].dim

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        value = params["value"]

        return value

    def show_signal(self, t0=None, tf=None, n_pts=1000, ax=None):
        """
        Plot the source output signal over a time range.

        Parameters
        ----------
        t0 : float, optional
            Start time (default 0.0).
        tf : float, optional
            End time (default 10.0).
        n_pts : int, optional
            Number of evaluation points used for plotting.
        ax : matplotlib.axes.Axes, optional
            Existing axis to draw on. If None, a new figure is created and shown
            (same policy as
            :func:`minilink.graphical.signals.plot_time_signals`);
            if an axis is passed, the caller controls display.
        """
        import matplotlib.pyplot as plt

        # Lazy import: the graphics load on first use, not with the core
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

        # The time range, 0 to 10 s by default
        if t0 is None:
            t0 = 0.0
        if tf is None:
            tf = 10.0
        require_plot_points(n_pts)

        # The signal sampled on the time grid, one row per channel
        p = self.dim
        t = np.linspace(t0, tf, int(n_pts))
        y = np.zeros((p, t.size), dtype=float)
        empty_x = np.array([])
        empty_u = np.array([])
        for i, ti in enumerate(t):
            y_i = self.h(empty_x, empty_u, ti)
            y[:, i] = np.asarray(y_i, dtype=float).reshape(p)

        # A new figure when no axis is given, titled after the block
        created = ax is None
        if created:
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

        # One curve per channel
        for i in range(p):
            label = f"{self.name}[{i}]" if p > 1 else self.name
            ax.plot(t, y[i, :], "b", linewidth=1.5, alpha=0.8, label=label)

        ax.set_ylabel("value", fontsize=FONT_SIZE, multialignment="center")
        style_trajectory_subplot(ax)
        ax.legend(loc="upper right")
        ax.set_xlabel("Time [s]", fontsize=FONT_SIZE)
        ax.set_title(f"{self.name} signal", fontsize=FONT_SIZE)

        # A figure made here is shown here; a passed axis leaves display to the caller
        if created:
            plt.show(block=is_blocking_needed())

        return fig, ax


class Step(Source):
    """Step source: ``y = initial_value`` if ``t < step_time``, else ``final_value``."""

    def __init__(
        self, initial_value=None, final_value=None, step_time=1.0, *, port="y"
    ):
        # The levels before and after the step: flat vectors of one shape, zero by default
        if initial_value is None:
            initial_value = np.zeros(1)
        if final_value is None:
            final_value = np.zeros(1)
        initial_value = np.asarray(initial_value, dtype=float).reshape(-1)
        final_value = np.asarray(final_value, dtype=float).reshape(-1)
        require_matching_levels(initial_value, final_value)

        p = initial_value.shape[0]
        Source.__init__(self, p, port=port)

        # Parameters
        self.params = {
            "initial_value": initial_value,
            "final_value": final_value,
            "step_time": step_time,
        }

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(t)

        step_time = params["step_time"]
        initial_value = params["initial_value"]
        final_value = params["final_value"]

        # y = initial value before the step time, final value after
        y = xp.where(t < step_time, initial_value, final_value)

        return y


class Sine(Source):
    """Sine source: ``y = offset + amplitude sin(omega t + phase)``, ``omega`` in rad/s.

    ``amplitude`` sets the dimension; ``omega``, ``phase`` and ``offset`` are
    scalars or arrays of the same shape. A frequency ``f`` in hertz is
    ``omega = 2 * np.pi * f``.
    """

    def __init__(self, amplitude=1.0, omega=1.0, phase=0.0, offset=0.0, *, port="y"):
        # The amplitude sets the dimension
        amplitude = np.asarray(amplitude, dtype=float).reshape(-1)
        p = amplitude.shape[0]

        Source.__init__(self, p, port=port)

        # Parameters: the frequency, phase and offset, one per channel
        self.params = {
            "amplitude": amplitude,
            "omega": as_channels(omega, p),
            "phase": as_channels(phase, p),
            "offset": as_channels(offset, p),
        }

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(t)

        a = params["amplitude"]
        omega = params["omega"]
        phi = params["phase"]
        y0 = params["offset"]

        # y = y₀ + a sin(ω t + φ)
        y = y0 + a * xp.sin(omega * t + phi)

        return y


class WhiteNoise(Source):
    """White noise of intensity ``psd``, held over each ``sample_period``.

    The intensity is the two-sided spectral density of the signal (a one-sided
    estimate such as ``scipy.signal.welch``'s default reads twice it). The
    per-sample covariance is the intensity over the period, so the physics does
    not change with the period; a per-sample variance over a period is an
    intensity of variance times period. The zero-order hold keeps each sample
    for its whole period; ``hold="linear"`` interpolates between samples, with a
    mid-period variance of half the per-sample one. A fixed step that divides the
    period keeps RK4's stages inside one sample except the last stage of a
    boundary step, an effect first order in the step. The sample counter is
    32 bits: the signal repeats after 2³² samples. ``seed=None`` gives the mean
    (zero); the seed is the block's default realization, :meth:`realize` draws
    another. A scalar or vector ``psd`` is a diagonal intensity, a matrix a full
    one.
    """

    HOLDS = ("zoh", "linear")
    is_random = True

    def __init__(
        self, p=1, *, psd=1.0, sample_period=0.01, seed=0, hold="zoh", port="y"
    ):
        require_one_of("hold", hold, self.HOLDS)
        require_seed(seed)
        require_positive_period(sample_period)

        Source.__init__(self, p, port=port)
        self.hold = hold

        # Parameters: the realization's seed, the sample period Δ and the intensity W
        self.params = {
            "seed": seed,
            "sample_period": float(sample_period),
            "psd": as_intensity(psd, p),
        }

        # Solver hints: the sample period
        self.refresh()

    @property
    def psd(self):
        """The intensity ``W``: a ``(p,)`` diagonal or a ``(p, p)`` matrix."""
        return self.params["psd"]

    def h(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        seed = params["seed"]
        W = params["psd"]
        period = params["sample_period"]
        hold = self.hold
        p = self.dim

        # The backend of the time and of the intensity
        xp = array_module(t, W)

        # Without a seed, the mean: no draw
        if seed is None:
            w_mean = xp.zeros(p)
            return w_mean

        # The sample index of the held train: w(t) = w_k on [kΔ, (k+1)Δ)
        k = sample_index(t, period)
        s = t / period - k

        # Σ = W / Δ: the per-sample covariance of an intensity W held over Δ
        Sigma = W / period
        L = intensity_root(Sigma, p)

        # w_k = Σ^{1/2} ε_k, with ε_k = F(seed, k) ~ N(0, I); the linear hold blends in w_{k+1}
        w = L @ standard_normal(seed, k, p)
        if hold == "linear":
            w_next = L @ standard_normal(seed, k + 1, p)
            w = (1 - s) * w + s * w_next

        return w

    def realize(self, key):
        """The params of one realization: a fresh seed under ``key``, or the mean for ``None``."""
        seed = None if key is None else child_seed(key, "seed")
        return dict(self.params, seed=seed)

    def refresh(self):
        """Publish the sample period as the solver hint of the block."""
        period = float(self.params["sample_period"])
        self.solver_info["sample_period"] = period
        self.solver_info["smallest_time_constant"] = period


class TrajectorySource(Source):
    """Replay a stored signal ``y(t)`` sampled at times ``t``.

    The open-loop policy ``u = pi(t)``: feed a planned input trajectory into a
    diagram, ``source >> plant``. Between samples the signal is interpolated
    linearly (``interpolation="linear"``, what a collocation or shooting
    transcription assumes) or held at the previous sample
    (``"previous"``, the zero-order hold of a sampled controller). Values
    outside ``[t[0], t[-1]]`` hold the nearest endpoint. ``values`` has shape
    ``(p, N)`` (or ``(N,)`` for a scalar signal) aligned with the ``N``
    sample times in ``t``.
    """

    INTERPOLATIONS = ("linear", "previous")

    def __init__(self, t, values, *, interpolation="linear", port="y"):
        # The samples: N times, and a (p, N) array of values
        t = np.asarray(t, dtype=float).reshape(-1)
        values = np.asarray(values, dtype=float)
        if values.ndim == 1:
            values = values.reshape(1, -1)
        require_sample_shape(values, t)
        require_one_of("interpolation", interpolation, self.INTERPOLATIONS)

        Source.__init__(self, values.shape[0], port=port)

        # The samples and the rule between them
        self.sample_times = t
        self.sample_values = values
        self.interpolation = interpolation

    @classmethod
    def from_trajectory(cls, traj, signal="u", *, interpolation="linear"):
        """Build a source that replays a :class:`Trajectory` signal (``u`` or ``x``)."""
        return cls(traj.t, getattr(traj, signal), interpolation=interpolation)

    def h(self, x, u, t=0, params=None):
        """The sample at ``t``: linear between samples, or the previous one; endpoints held (traces on JAX)."""
        xp = array_module(x, u, t)

        t_k = xp.asarray(self.sample_times)
        y_k = xp.asarray(self.sample_values)

        # Linear: y(t) = y_k + (t − t_k) (y_{k+1} − y_k) / (t_{k+1} − t_k), clamped at the ends
        if self.interpolation == "linear":
            y = xp.stack([xp.interp(t, t_k, y_k[i]) for i in range(self.dim)])

        # Previous: y(t) = y_k for t_k ≤ t < t_{k+1}, clamped at the ends
        else:
            k = xp.clip(xp.searchsorted(t_k, t, side="right") - 1, 0, t_k.size - 1)
            y = y_k[:, k]

        return y


###############################################################################
# Internal machinery
###############################################################################


def require_plot_points(n_pts):
    """Refuse a time grid of fewer than two points."""
    if n_pts < 2:
        raise ValueError("n_pts must be >= 2")


def require_matching_levels(initial_value, final_value):
    """Refuse step levels of different shapes."""
    if final_value.shape != initial_value.shape:
        raise ValueError("initial_value and final_value must have the same shape")


def as_channels(value, p):
    """A scalar or a ``(p,)`` array as its own ``(p,)`` float vector, one entry per channel."""
    return np.broadcast_to(np.asarray(value, dtype=float), (p,)).copy()


def require_seed(seed):
    """Refuse a negative seed (``None`` is the mean)."""
    if seed is not None and int(seed) < 0:
        raise ValueError(f"seed must be a non-negative integer or None, got {seed!r}")


def require_positive_period(sample_period):
    """Refuse a sample period that is not positive."""
    if float(sample_period) <= 0.0:
        raise ValueError("sample_period must be > 0")


def as_intensity(psd, p):
    """The intensity as a ``(p,)`` diagonal (a scalar broadcast) or a ``(p, p)`` matrix."""
    W = np.asarray(psd, dtype=float)
    if W.ndim == 0:
        W = np.full(p, float(W))
    if W.shape not in ((p,), (p, p)):
        raise ValueError(
            f"psd must be a scalar, a ({p},) diagonal or a ({p}, {p}) matrix"
        )
    if np.any(W.diagonal() < 0.0) if W.ndim == 2 else np.any(W < 0.0):
        raise ValueError("psd must be non-negative")
    return W


def intensity_root(W, p):
    """A square root ``L`` with ``L Lᵀ = W``: element-wise for a diagonal, symmetric for a matrix."""
    xp = array_module(W)
    W = xp.asarray(W)

    if W.ndim < 2:
        # L = diag(√W)
        L = xp.diag(xp.sqrt(W) * xp.ones(p))
    else:
        # W = V Λ Vᵀ, so L = V √Λ Vᵀ (eigenvalues floored at zero against roundoff)
        values, vectors = xp.linalg.eigh(W)
        L = (vectors * xp.sqrt(xp.maximum(values, 0.0))) @ vectors.T

    return L


def require_sample_shape(values, t):
    """Refuse values that do not hold one column per sample time."""
    if values.shape[1] != t.size:
        raise ValueError("values must have shape (p, len(t))")


if __name__ == "__main__":
    noise = WhiteNoise(1, psd=0.01, sample_period=0.01, seed=1)
    fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
    ax.set_title("Baseline")

    # The intensity is fixed: a longer period holds smaller samples for longer
    for period in (0.05, 0.2, 1.0):
        noise.params["sample_period"] = period
        fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
        ax.set_title(f"sample_period = {period}")
