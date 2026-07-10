"""Scalar logistic map with parameter ``r``.

Run from the repo root::

    python examples/scripts/step/demo_step_logistic_map.py
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StepSystem


class LogisticMap(StepSystem):
    def __init__(self, r=3.9):
        super().__init__(n=1, output_dim=1, y_dependencies=())
        self.name = "LogisticMap"
        self.params = {"r": float(r)}
        self.x0 = np.array([0.2])

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        r = params["r"]
        xp = array_module(x)
        x0 = xp.asarray(x, dtype=float).reshape(1)[0]
        return xp.array([r * x0 * (1.0 - x0)])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


plant = LogisticMap(r=3.9)
plant.compute_rollout(n_steps=30)
plant.plot_rollout()
