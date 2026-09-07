"""Controllability and observability of a linearized plant."""

import numpy as np

from minilink import InvertedPendulum, controllability, observability

plant = InvertedPendulum()
lin = plant.linearize([0.0, 0.0])

ctrl = controllability(lin)
obs = observability(lin)

print("A =\n", np.round(lin.A(), 4))
print("controllable:", ctrl.is_full_rank, f"(rank {ctrl.rank}/{ctrl.n})")
print("observable:  ", obs.is_full_rank, f"(rank {obs.rank}/{obs.n})")
