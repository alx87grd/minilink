"""Root locus of the inverted pendulum: angle feedback alone against a lead compensator."""

import numpy as np

from minilink import InvertedPendulum, Lead

plant = InvertedPendulum()
plant.x0 = np.array([0.0, 0.0])  # upright: the operating point every tool defaults to

# u = -K theta: the two real poles meet at the origin and leave along the
# imaginary axis, so no gain on the angle alone stabilizes the upright pole.
plant.plot_pzmap()
plant.plot_root_locus()

# A lead compensator (s + 2) / (s + 20) adds rate feedback: its zero pulls
# both branches into the left half-plane above a critical gain.
L = Lead(K=1.0, z=2.0, p=20.0) >> plant
L.plot_root_locus()

gains, roots = L.root_locus()
stable = np.all(roots.real < 0.0, axis=1)
print(f"stable above K = {gains[np.argmax(stable)]:.3g}")
