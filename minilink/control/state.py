"""Feedback profile: state — full-state linear feedback."""

import numpy as np

from minilink.core.system import StaticSystem


class StateFeedbackController(StaticSystem):
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
