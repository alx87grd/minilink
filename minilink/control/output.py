"""Feedback profile: error — static output-error proportional control."""

import numpy as np

from minilink.core.feedback import Controller, ErrorDriven


class ProportionalController(ErrorDriven, Controller):
    """Output-error proportional control ``u = K e`` (SISO or MIMO).

    ``K`` is a scalar (SISO, a ``1×1`` gain) or an ``(m, p)`` matrix mapping a
    ``p``-vector tracking error to an ``m``-vector command. The gain lives in
    ``params["K"]`` (always stored as a matrix) so it can be tuned or
    differentiated.

    ``ports="reference"`` (default) declares ``r`` and ``y`` and computes
    ``e = r - y`` inside; ``ports="error"`` declares one input ``e`` — the
    compensator form, where ``@`` inserts the Error block. For *state*
    feedback with a feedforward offset — the form LQR produces — use
    :class:`~minilink.control.state.StateFeedbackController`.
    """

    feedback_profile = "error"

    def __init__(self, K=1.0, *, ports="reference"):
        super().__init__()
        self.name = "P Controller"

        K = np.atleast_2d(np.asarray(K, dtype=float))
        self.params = {"K": K}
        m, p = K.shape

        self.add_error_ports(ports, p)
        self.add_output_port(
            "u", dim=m, function=self.ctl, dependencies=self.error_dependencies
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]
        e = self.error(u)
        return K @ e
