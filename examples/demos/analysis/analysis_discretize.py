"""Discretize a continuous plant to a StepSystem (Euler / RK4)."""

import numpy as np

from minilink import Pendulum, discretize

plant = Pendulum()
plant.x0 = np.array([0.5, 0.0])

disc = discretize(plant, dt=0.05, method="rk4")
print(
    f"continuous n={plant.n}  ->  discrete {type(disc).__name__}  dt={disc.params['dt']}"
)

u_seq = np.zeros((40, plant.m))
disc.compute_rollout(n_steps=40, u=u_seq)
disc.plot_rollout()
