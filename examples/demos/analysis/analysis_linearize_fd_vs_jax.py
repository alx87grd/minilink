"""Compare finite-difference and JAX linearization on a tiny plant."""

import numpy as np

from minilink import CartPole, DoublePendulum
from minilink.analysis.linearize import linearize_matrices

np.set_printoptions(precision=4, suppress=True)

plant = CartPole()
xbar = np.array([0.0, 0.0, 0.0, 0.0])
ubar = np.array([0.0])


A, B, C, D = linearize_matrices(plant, xbar, ubar, method="fd")
print("CartPole (fd):")
print("A =\n", A)

A, B, C, D = linearize_matrices(plant, xbar, ubar, method="jax")
print("CartPole (jax):")
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
