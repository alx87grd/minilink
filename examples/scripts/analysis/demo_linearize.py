"""Linearize a nonlinear plant about an operating point.

Run from the repo root::

    python examples/scripts/analysis/demo_linearize.py
"""

import numpy as np

from minilink.analysis.linearize import linearize
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

plant = InvertedPendulum()

# InvertedPendulum measures theta from the upward vertical:
#   theta = 0   -> upright (unstable equilibrium for this model)
#   theta = pi  -> hanging down (stable equilibrium)
operating_points = {
    "upright (unstable)": np.array([0.0, 0.0]),
    "down (stable)": np.array([np.pi, 0.0]),
}

for label, x_bar in operating_points.items():
    lti = linearize(plant, x_bar)
    print(f"--- {label} ---")
    print("operating point x_bar:", np.round(x_bar, 2))
    print("A =\n", np.round(lti.A(), 4))
    print("B =\n", np.round(lti.B(), 4))
    print("C =\n", np.round(lti.C(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 2))
    print()
