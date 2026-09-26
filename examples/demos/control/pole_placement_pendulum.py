"""Pole placement on the inverted pendulum: choose the closed-loop poles, get the gain."""

import numpy as np

from minilink import InvertedPendulum, place_at_operating_point

plant = InvertedPendulum()
x_bar = np.array([0.0, 0.0])  # upright

# A double pole at -3: a critically damped return to the upright
ctl = place_at_operating_point(plant, x_bar, poles=[-3.0, -3.0])

lin = plant.linearize(x_bar)
K = ctl.params["K"]
print("K =", K)
print("eig(A - B K) =", np.linalg.eigvals(lin.A() - lin.B() @ K))


diagram = ctl @ plant

plant.x0 = np.array([0.4, 0.0])

diagram.compute_trajectory(tf=4.0)
diagram.plot_trajectory()
diagram.animate()
