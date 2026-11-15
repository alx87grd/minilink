"""Controllability and observability of a plant, linearized at an operating point."""

import numpy as np

from minilink import InvertedPendulum, controllability, observability

plant = InvertedPendulum()
x_bar = [0.0, 0.0]

# The plant is linearized about its command port, then its measured output
ctrb, ctrb_rank = controllability(plant, x_bar)
obsv, obsv_rank = observability(plant, x_bar)

print("A =\n", np.round(plant.linearize(x_bar).A(), 4))
print(f"controllability matrix, rank {ctrb_rank} of {plant.n}:\n", ctrb)
print(f"observability matrix, rank {obsv_rank} of {plant.n}:\n", obsv)
