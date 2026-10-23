"""Discrete-time wiring blocks: the unit delay."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StepSystem

###############################################################################
# Public API
###############################################################################


class UnitDelay(StepSystem):
    """Unit delay ``z⁻¹``: ``x_{k+1} = u_k``, ``y_k = x_k``.

    The output is the input one tick late. The continuous zero-order hold of a
    sampled signal is not a block: the sampled loop and the rollouts hold their
    sequences.
    """

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

        # Initial state: the delay holds zero
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(u)

        # The delay: x_{k+1} = u_k
        x_next = xp.asarray(u, dtype=float).reshape(1).copy()

        return x_next

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)

        # y_k = x_k, the held value
        y = xp.asarray(x, dtype=float).reshape(1).copy()

        return y
