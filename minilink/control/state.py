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

    ``t`` holds ``N`` sample times and ``K`` the matching gains, shape
    ``(N, m, n)``; between samples the last scheduled gain is held, and past
    the final sample the final gain applies. Ports are those of
    :class:`StateFeedbackController`. This is the block
    :func:`minilink.control.lqr.lqr_finite_horizon` returns.
    """

    feedback_profile = "state"

    def __init__(self, t, K, xbar=None, ubar=None):
        super().__init__()
        self.name = "Time-Varying State Feedback Controller"

        t = np.asarray(t, dtype=float).reshape(-1)
        K = np.asarray(K, dtype=float)
        if K.ndim != 3 or K.shape[0] != t.size:
            raise ValueError("K must have shape (N, m, n) with N = len(t)")
        _, m, n = K.shape
        xbar = (
            np.zeros(n) if xbar is None else np.asarray(xbar, dtype=float).reshape(-1)
        )
        ubar = (
            np.zeros(m) if ubar is None else np.asarray(ubar, dtype=float).reshape(-1)
        )
        self.params = {"t": t, "K": K, "ubar": ubar}

        self.add_input_port("x", dim=n)
        self.add_input_port("r", dim=n, nominal_value=xbar)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("x", "r"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        t_schedule = params["t"]
        K_schedule = params["K"]
        ubar = params["ubar"]
        xp = array_module(u)

        n = K_schedule.shape[2]
        x_meas = u[:n]
        r = u[n:]

        # gain in force at time t: the last scheduled sample at or before t
        i = xp.searchsorted(xp.asarray(t_schedule), t, side="right") - 1
        K = xp.asarray(K_schedule)[xp.clip(i, 0, len(t_schedule) - 1)]

        return ubar - K @ (x_meas - r)
