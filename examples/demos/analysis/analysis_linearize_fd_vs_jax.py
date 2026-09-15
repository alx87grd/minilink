"""Compare finite-difference and exact (JAX) Jacobians on two plants."""

import numpy as np

from minilink import CartPole, DoublePendulum

np.set_printoptions(precision=4, suppress=True)

for plant in (CartPole(), DoublePendulum()):
    x_bar = np.zeros(plant.n)
    A_fd = plant.jacobian("f", "x", x_bar, method="fd")
    A_jax = plant.jacobian("f", "x", x_bar, method="jax")
    print(f"{plant.name}: A (finite differences) =\n{A_fd}")
    print(f"{plant.name}: max |A_jax - A_fd| = {np.max(np.abs(A_jax - A_fd)):.2e}\n")
