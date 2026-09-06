"""Compare finite-difference and JAX linearization on a tiny plant."""

import numpy as np

from minilink import DoublePendulum, JaxCartPole
from minilink.analysis.linearize import linearize_matrices

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
