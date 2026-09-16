"""Feedback profile: state — full-state linear feedback, constant or scheduled gain."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.feedback import Controller


class StateFeedbackController(Controller):
    """Full-state feedback ``u = ubar - K (x - r)``.

    Ports: the plant state ``x`` (dimension ``n``) and a reference ``r`` (also
    ``n``); when ``r`` is left unconnected it holds ``xbar``, so the law
    regulates to ``xbar``. ``K`` has shape ``(m, n)``; ``ubar`` is the
    feedforward command. This is the block :func:`minilink.control.lqr.lqr`
    returns.

    This is *state* feedback on the full ``x`` port. For *output* feedback on a
    measured ``y`` (and ``@``-operator wiring), use
    :class:`~minilink.control.output.ProportionalController`.
    """

    feedback_profile = "state"

    def __init__(self, K, xbar=None, ubar=None):
        super().__init__()
        self.name = "State Feedback Controller"

        K = np.atleast_2d(np.asarray(K, dtype=float))
        m, n = K.shape
        xbar = (
            np.zeros(n) if xbar is None else np.asarray(xbar, dtype=float).reshape(-1)
        )
        ubar = (
            np.zeros(m) if ubar is None else np.asarray(ubar, dtype=float).reshape(-1)
        )
        self.params = {"K": K, "ubar": ubar}

        self.add_input_port("x", dim=n)
        self.add_input_port("r", dim=n, nominal_value=xbar)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("x", "r"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]
        ubar = params["ubar"]

        n = K.shape[1]
        x_meas = u[:n]
        r = u[n:]

        u_cmd = ubar - K @ (x_meas - r)

        return u_cmd


class TimeVaryingStateFeedbackController(Controller):
    """Full-state feedback with a gain schedule ``u = ubar - K(t) (x - r)``.

    ``t`` holds ``N`` increasing sample times and ``K`` the matching gains,
    shape ``(N, m, n)``. Between samples the gain is interpolated linearly;
    before the first sample ``K[0]`` applies, and past the last sample the
    block keeps ``K[-1]`` — or applies ``K_after`` when one is given (for a
    finite-horizon design, the stationary gain). Ports are those of
    :class:`StateFeedbackController`. This is the block
    :func:`minilink.control.lqr.lqr_finite_horizon` returns.
    """

    feedback_profile = "state"

    def __init__(self, t, K, xbar=None, ubar=None, K_after=None):
        super().__init__()
        self.name = "Time-Varying State Feedback Controller"

        t = np.asarray(t, dtype=float).reshape(-1)
        K = np.asarray(K, dtype=float)
        if K.ndim != 3 or K.shape[0] != t.size or t.size < 2:
            raise ValueError("K must have shape (N, m, n) with N = len(t) >= 2")
        if np.any(np.diff(t) <= 0.0):
            raise ValueError("t must be strictly increasing")
        _, m, n = K.shape
        xbar = (
            np.zeros(n) if xbar is None else np.asarray(xbar, dtype=float).reshape(-1)
        )
        ubar = (
            np.zeros(m) if ubar is None else np.asarray(ubar, dtype=float).reshape(-1)
        )
        K_after = (
            K[-1]
            if K_after is None
            else np.atleast_2d(np.asarray(K_after, dtype=float))
        )
        self.params = {"t": t, "K": K, "K_after": K_after, "ubar": ubar}

        self.add_input_port("x", dim=n)
        self.add_input_port("r", dim=n, nominal_value=xbar)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("x", "r"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        t_schedule = params["t"]
        K_schedule = params["K"]
        K_after = params["K_after"]
        ubar = params["ubar"]
        xp = array_module(u)

        n = K_schedule.shape[2]
        x_meas = u[:n]
        r = u[n:]

        # gain at time t: interpolated on the schedule, K_after past its end
        K = interpolate_schedule(xp, t_schedule, K_schedule, t)
        K = xp.where(t > t_schedule[-1], xp.asarray(K_after), K)

        u_cmd = ubar - K @ (x_meas - r)

        return u_cmd

    def plot_gain_schedule(self, ax=None, show=True):
        """Draw every entry of the gain schedule ``K(t)`` against time."""
        return plot_gain_schedule(
            self.params["t"], self.params["K"], title=self.name, ax=ax, show=show
        )


class TrajectoryFeedbackController(Controller):
    """Feedback around a reference trajectory ``u = u_d(t) - K(t) (x - x_d(t))``.

    ``trajectory`` is the reference, a :class:`~minilink.core.trajectory.Trajectory`
    with ``N`` samples of ``x_d`` and ``u_d``, and ``K`` holds the matching
    gains, shape ``(N, m, n)``. Reference and gains are interpolated linearly
    between samples; past the last sample the block holds the final point and
    gain, so it keeps regulating where the trajectory ends — usually an
    equilibrium. The measurement is the full state and the reference is the
    trajectory itself, so there is no reference port. This is the block
    :func:`minilink.control.lqr.trajectory_lqr` returns.
    """

    feedback_profile = "state"

    def __init__(self, trajectory, K):
        super().__init__()
        self.name = "Trajectory Feedback Controller"

        t = np.asarray(trajectory.t, dtype=float).reshape(-1)
        x_d = np.asarray(trajectory.x, dtype=float).T  # (N, n)
        u_d = np.asarray(trajectory.u, dtype=float).T  # (N, m)
        K = np.asarray(K, dtype=float)
        N, n = x_d.shape
        m = u_d.shape[1]
        if K.shape != (N, m, n):
            raise ValueError(
                f"K must have shape (N, m, n) = {(N, m, n)}, got {K.shape}"
            )
        if N < 2 or np.any(np.diff(t) <= 0.0):
            raise ValueError(
                "the trajectory needs at least two strictly increasing samples"
            )
        self.params = {"t": t, "x_d": x_d, "u_d": u_d, "K": K}

        self.add_input_port("x", dim=n)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies="all")

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        t_schedule = params["t"]
        x_d = params["x_d"]
        u_d = params["u_d"]
        K_schedule = params["K"]
        xp = array_module(u)
        x_meas = u

        # reference point and gain at time t, held at the trajectory's end
        x_ref = interpolate_schedule(xp, t_schedule, x_d, t)
        u_ref = interpolate_schedule(xp, t_schedule, u_d, t)
        K = interpolate_schedule(xp, t_schedule, K_schedule, t)

        u_cmd = u_ref - K @ (x_meas - x_ref)

        return u_cmd

    def plot_gain_schedule(self, ax=None, show=True):
        """Draw every entry of the gain schedule ``K(t)`` against time."""
        return plot_gain_schedule(
            self.params["t"], self.params["K"], title=self.name, ax=ax, show=show
        )


# Public functions


def interpolate_schedule(xp, t_samples, values, t):
    """Value of a sampled schedule at time ``t``: linear between samples, held outside.

    ``t_samples`` holds ``N`` increasing sample times and ``values`` the
    samples, shape ``(N, ...)``; the result has the shape of one sample.
    ``xp`` is the array module of the caller (NumPy or JAX).
    """
    ts = xp.asarray(t_samples)
    values = xp.asarray(values)

    # interval [t_i, t_i+1] holding t, clipped to the schedule
    i = xp.clip(xp.searchsorted(ts, t, side="right") - 1, 0, len(t_samples) - 2)
    w = xp.clip((t - ts[i]) / (ts[i + 1] - ts[i]), 0.0, 1.0)
    value = (1.0 - w) * values[i] + w * values[i + 1]

    return value


def plot_gain_schedule(t, K, *, title="Gain schedule", ax=None, show=True):
    """Draw every entry of a gain schedule ``K(t)``, shape ``(N, m, n)``, against time."""
    import matplotlib.pyplot as plt

    _, m, n = K.shape
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 3))
    for i in range(m):
        for j in range(n):
            ax.plot(t, K[:, i, j], label=f"$K_{{{i + 1},{j + 1}}}$")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("K(t)")
    ax.set_title(title)
    ax.legend(ncol=max(1, m * n // 2), fontsize=8)
    ax.grid(True, alpha=0.3)
    if show and plt.get_backend().lower() != "agg":
        plt.show()
    return ax
