"""Controllability and observability of a plant, linearized at an operating point."""

import numpy as np

from minilink import InvertedPendulum, controllability, observability

plant = InvertedPendulum()
x_bar = [0.0, 0.0]

ctrl = controllability(plant, x_bar)  # the plant is linearized about its command port
obs = observability(plant, x_bar)  # and its measured output

print("A =\n", np.round(plant.linearize(x_bar).A(), 4))
print(ctrl)
print(obs)
