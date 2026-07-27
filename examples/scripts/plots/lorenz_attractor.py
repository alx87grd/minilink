"""Lorenz attractor — nonlinear chaotic simulation.

Run from repo root::

    PYTHONPATH=. MPLBACKEND=Agg python examples/scripts/plots/lorenz_attractor.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.dynamics.catalog.equations.oscillators import Lorenz

TF = 40.0
N_STEPS = 4000

sys = Lorenz()
sys.x0 = np.array([1.0, 1.0, 1.0])

traj = sys.compute_trajectory(tf=TF, n_steps=N_STEPS, verbose=False)
sys.plot_trajectory(traj)

x, y, z = traj.x[0], traj.x[1], traj.x[2]
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(x, y, z, linewidth=0.6, color="#4c78a8")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("Lorenz attractor")
plt.show()
