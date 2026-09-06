"""Animate one pendulum trajectory with each renderer."""

import numpy as np

from minilink import Pendulum

sys = Pendulum()
sys.x0[0] = 2.0

traj = sys.compute_forced(u=lambda t: 3.0 * np.sin(10.0 * t), tf=5.0, show=False)
sys.plot_trajectory(backend="plotly")

sys.animate(renderer="matplotlib")
sys.animate(renderer="matplotlib", native=False)
try:
    sys.animate(renderer="meshcat", show=False)
    sys.animate(renderer="meshcat", native=False, show=False)
except ImportError:
    pass  # meshcat is an optional extra
# sys.animate(renderer="pygame")
# sys.animate(renderer="plotly", is_3d=True)
