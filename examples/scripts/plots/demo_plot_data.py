"""Signal-vs-signal plotting demo (experimental ``plot_data`` facade).

Plots one trajectory signal against another instead of against time. Here the
pendulum angular rate is plotted versus its angle (a phase-space path).
"""

import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

pendulum = Pendulum()
pendulum.x0 = np.array([-np.pi, 4.0])
traj = pendulum.compute_trajectory(tf=5.0, show=False, verbose=False)

# theta_dot vs theta, read by signal label (see Pendulum state.labels).
pendulum.plot_data(traj, signals=("x",), x_label="theta", y_labels=("theta_dot",))
