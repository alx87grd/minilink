"""Pure Fibonacci recurrence on a two-state step leaf.

Run from the repo root::

    python examples/scripts/step/demo_step_fibonacci.py
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StepSystem


class Fibonacci(StepSystem):
    def __init__(self):
        super().__init__(n=2, expose_state=True)
        self.name = "Fibonacci"
        self.x0 = np.array([0.0, 1.0])
        self.state.labels = ("F_k", "F_{k+1}")

    def step(self, x, u, k=0, params=None):
        xp = array_module(x)
        x = xp.asarray(x, dtype=float).reshape(2)
        return xp.array([x[1], x[0] + x[1]])


fib = Fibonacci()
fib.compute_rollout(n_steps=20)
fib.plot_rollout()
