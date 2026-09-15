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

        return ubar - K @ (x_meas - r)


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

        # interval [t_i, t_i+1] holding t, clipped to the schedule
        ts = xp.asarray(t_schedule)
        Ks = xp.asarray(K_schedule)
        i = xp.clip(xp.searchsorted(ts, t, side="right") - 1, 0, len(t_schedule) - 2)

        # linear interpolation inside the interval, K[0] before it, K_after past the end
        w = xp.clip((t - ts[i]) / (ts[i + 1] - ts[i]), 0.0, 1.0)
        K = (1.0 - w) * Ks[i] + w * Ks[i + 1]
        K = xp.where(t > ts[-1], xp.asarray(K_after), K)

        return ubar - K @ (x_meas - r)
