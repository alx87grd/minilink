"""Compare finite-difference and JAX linearization on a tiny plant.

Run from the repo root::

    python examples/scripts/analysis/demo_linearize_fd_vs_jax.py
"""

import numpy as np

from minilink.analysis.linearize import linearize_matrices
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum

np.set_printoptions(precision=4, suppress=True)

plant = JaxCartPole()
xbar = np.array([0.0, 0.0, 0.0, 0.0])
ubar = np.array([0.0])


A, B, C, D = linearize_matrices(plant, xbar, ubar, method="fd")
print("JaxCartPole (fd):")
print("A =\n", A)

A, B, C, D = linearize_matrices(plant, xbar, ubar, method="jax")
print("JaxCartPole (jax):")
print("A =\n", A)

plant = DoublePendulum()
xbar = np.array([0.0, 0.0, 0.0, 0.0])
ubar = np.array([0.0, 0.0])

A, B, C, D = linearize_matrices(plant, xbar, ubar, method="fd")
print("DoublePendulum (fd):")
print("A =\n", A)

A, B, C, D = linearize_matrices(plant, xbar, ubar, method="jax")
print("DoublePendulum (jax):")
print("A =\n", A)
