"""Discrete-time step wiring blocks."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StepSystem


class ZOHHold(StepSystem):
    """Command latch: ``x_{k+1} = u_k``, ``y_k = x_k``.

    Teaches the zero-order hold / hold-register pattern on the step side of
    hybrid boundaries (plant-side holds are handled by :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`).
    """

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "ZOHHold"
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(u)
        return xp.asarray(u, dtype=float).reshape(1).copy()

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()
