"""Multi-rate filter + controller with a continuous integrator plant."""

import numpy as np

from minilink import Integrator, P, Step, StepSystem, closed_loop
from minilink.core.backends import array_module

DT_BASE = 0.01
TF = 2.0
PLANT_DT_INNER = 0.002


class DiscreteLowPass(StepSystem):
    """First-order low-pass: ``x_{k+1} = α x_k + (1-α) u_k``, ``y = x``."""

    def __init__(self, alpha=0.25):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        alpha = params["alpha"]
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


# The continuous plant: the loop integrates it in steps of PLANT_DT_INNER (the run's dt)
plant = Integrator()

# A 10 Hz gain on a 100 Hz filtered measurement, the reference a step down at 1 s
loop = closed_loop(
    P(10.0, ports="reference") % (10 * DT_BASE),
    plant,
    reference=Step(np.array([1.0]), np.array([0.0]), step_time=1.0),
    filter=DiscreteLowPass(alpha=0.6) % DT_BASE,
)

loop.plot_diagram()

loop.compute_trajectory(tf=TF, dt=PLANT_DT_INNER)
loop.plot_trajectory(show=True)
