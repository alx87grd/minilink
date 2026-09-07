"""Flat demo for trajectory computation and plotting options."""

import numpy as np

from minilink import Pendulum

sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0 = np.array([0.8, 0.0])

traj = sys.compute_trajectory(tf=6.0, verbose=False)

# Choose one plot_trajectory line.
result = sys.plot_trajectory(traj, signals=("x", "u"), backend="matplotlib", show=True)
result = sys.plot_trajectory(traj, signals=("x",), backend="matplotlib", show=True)
result = sys.plot_trajectory(traj, signals=("u",), backend="matplotlib", show=True)
# Plotly lines require the plotting extra.
result = sys.plot_trajectory(traj, signals=("x", "u"), backend="plotly", show=True)
result = sys.plot_trajectory(traj, signals=("x",), backend="plotly", show=True)
result = sys.plot_trajectory(traj, signals=("u",), backend="plotly", show=True)
