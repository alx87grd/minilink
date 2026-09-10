"""Animate one pendulum trajectory with each renderer."""

import numpy as np

from minilink import Pendulum

sys = Pendulum()
sys.x0[0] = 2.0

sys.compute_forced(u=lambda t: 3.0 * np.sin(10.0 * t), tf=5.0)
sys.plot_trajectory(backend="plotly")

sys.animate(renderer="matplotlib")
sys.animate(renderer="matplotlib", native=False)
sys.animate(renderer="meshcat")
sys.animate(renderer="meshcat", native=False)
# sys.animate(renderer="pygame")
# sys.animate(renderer="plotly", is_3d=True)
