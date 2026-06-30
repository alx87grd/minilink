"""Feedback profile: output — static output-error proportional control."""

import numpy as np

from minilink.core.system import StaticSystem


class ProportionalController(StaticSystem):
    """Output-error proportional control ``u = K (r - y)`` (SISO or MIMO).

    ``K`` is a scalar (SISO, a ``1×1`` gain) or an ``(m, p)`` matrix mapping a
    ``p``-vector tracking error to an ``m``-vector command; reference ``r`` and
    measurement ``y`` share dimension ``p``. The gain lives in ``params["K"]``
    (always stored as a matrix) so it can be tuned or differentiated.

    This is *output* feedback on the measured ``y`` port (it composes with the
    ``@`` operator). For *state* feedback with a feedforward offset — the form
    LQR produces — use :class:`~minilink.control.state.StateFeedbackController`.
    """

    feedback_profile = "output"

    def __init__(self, K=1.0):
        super().__init__()
        self.name = "P Controller"

        K = np.atleast_2d(np.asarray(K, dtype=float))
        self.params = {"K": K}
        m, p = K.shape

        self.add_input_port("r", dim=p)
        self.add_input_port("y", dim=p)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]

        p = K.shape[1]
        r = u[:p]
        y = u[p:]

        return K @ (r - y)
