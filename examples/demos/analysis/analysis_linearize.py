"""Linearize a nonlinear plant about an operating point."""

import numpy as np

from minilink import InvertedPendulum

plant = InvertedPendulum()

# InvertedPendulum measures theta from the upward vertical:
#   theta = 0   -> upright (unstable equilibrium for this model)
#   theta = pi  -> hanging down (stable equilibrium)
operating_points = {
    "upright (unstable)": np.array([0.0, 0.0]),
    "down (stable)": np.array([np.pi, 0.0]),
}

for label, x_bar in operating_points.items():
    lin = plant.linearize(x_bar)
    print(f"--- {label} ---")
    print("operating point x_bar:", np.round(x_bar, 2))
    print("A =\n", np.round(lin.A(), 4))
    print("B =\n", np.round(lin.B(), 4))
    print("C =\n", np.round(lin.C(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lin.A()), 2))
    print()

# The matrices are Jacobians of f and h; ask for any of them directly, at any point.
x = np.array([0.3, 0.1])
print("df/dx at x =", x, "\n", np.round(plant.jacobian("f", "x", x), 4))
print("df/du =\n", np.round(plant.jacobian("f", "u", x), 4))
print(
    "df/dparams:",
    {k: np.round(v, 3) for k, v in plant.jacobian("f", "params", x).items()},
)
