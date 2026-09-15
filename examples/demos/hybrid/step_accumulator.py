"""Discrete accumulator ``x_{k+1} = x_k + u_k``."""

import numpy as np

from minilink import StepSystem
from minilink.core.backends import array_module


class DiscreteAccumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "DiscreteAccumulator"
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


plant = DiscreteAccumulator()
u_seq = np.ones((20, 1))
plant.compute_rollout(n_steps=20, u=u_seq)
plant.plot_rollout()
