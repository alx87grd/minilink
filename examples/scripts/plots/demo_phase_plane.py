"""Phase-plane vector-field plotting demo."""

import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

pendulum = Pendulum()
pendulum.state.lower_bound = np.array([-2.0 * np.pi, -6.0])
pendulum.state.upper_bound = np.array([2.0 * np.pi, 6.0])
pendulum.x0 = np.array([-np.pi, 4.0])
pendulum_traj = pendulum.compute_trajectory(tf=5.0, show=False, verbose=False)

# pendulum.traj = None
pendulum.plot_phase_plane()
pendulum.plot_phase_plane(streamplot=True)
pendulum.plot_phase_plane(pendulum_traj)
