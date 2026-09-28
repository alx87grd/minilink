"""Pole placement on the inverted pendulum: choose the closed-loop poles, get the gain."""

import numpy as np

from minilink import InvertedPendulum, place_at_operating_point

plant = InvertedPendulum()
x_bar = np.array([0.0, 0.0])  # upright

poles = [-3.0 - 3j, -3.0 + 3j]
# poles=[-3.0, -3.0]
# poles = [-10, -1]

ctl = place_at_operating_point(plant, x_bar, poles=poles)

lin = plant.linearize(x_bar)
K = ctl.params["K"]


print("K =", K)
print("eig(A - B K) =", np.linalg.eigvals(lin.A() - lin.B() @ K))


diagram = ctl @ plant

diagram.plot_pzmap()

plant.x0 = np.array([0.4, 0.0])

diagram.compute_trajectory(tf=4.0)
diagram.plot_trajectory()
diagram.animate()
