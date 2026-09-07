"""Root locus of the inverted pendulum: angle feedback alone against a PD compensator."""

import numpy as np

from minilink import InvertedPendulum, TransferFunction

plant = InvertedPendulum()
x_bar = np.array([0.0, 0.0])  # upright

# u = -K theta: the two real poles meet at the origin and leave along the
# imaginary axis, so no gain on the angle alone stabilizes the upright pole.
plant.plot_pzmap(x_bar)
plant.plot_root_locus(x_bar)

# Add rate feedback, u = -K (theta + dtheta / 2): the zero at s = -2 pulls both
# branches into the left half-plane above a critical gain.
G = plant.transfer_function(x_bar)
L = TransferFunction(np.polymul([0.5, 1.0], G.numerator), G.denominator)
L.name = "PD-compensated inverted pendulum"
L.plot_root_locus()

gains, roots = L.root_locus()
stable = np.all(roots.real < 0.0, axis=1)
print(f"stable above K = {gains[np.argmax(stable)]:.3g}")
