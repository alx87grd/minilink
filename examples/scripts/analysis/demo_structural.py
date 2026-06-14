"""Controllability and observability of a linearized plant.

Run from the repo root::

    python examples/scripts/analysis/demo_structural.py
"""

import numpy as np

from minilink.analysis.linearize import linearize
from minilink.analysis.structural import controllability, observability
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

plant = InvertedPendulum()
lti = linearize(plant, x_bar=[0.0, 0.0])

A = lti.A()
B = lti.B()
C = lti.C()

ctrl = controllability(A, B)
obs = observability(A, C)

print("A =\n", np.round(A, 4))
print("controllable:", ctrl.is_full_rank, f"(rank {ctrl.rank}/{ctrl.n})")
print("observable:  ", obs.is_full_rank, f"(rank {obs.rank}/{obs.n})")
