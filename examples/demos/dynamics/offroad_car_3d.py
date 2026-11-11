"""An off-road side-by-side in 3-D, open loop: a slow weave over rolling bumps.

No controller: the power and steering commands are a fixed schedule. Watch the soft
springs soak up the bumps, the body pitch and roll, and the tall car lean in the
turns. Run with JAX (the 3-D car's stiff tires make the NumPy path slow).
"""

import numpy as np

from minilink import OffRoadCar3D
from minilink.dynamics.catalog.vehicles.terrain import Bumps

TF = 12.0
N_SAMPLES = 1201

car = OffRoadCar3D(terrain=Bumps(), wheel_angles=True)
car.params["terrain"]["amplitude"] = 0.12  # [m]
car.params["terrain"]["wavelength"] = 4.0  # [m]

# The schedule: steady power, the steering weaving left and right
t = np.linspace(0.0, TF, N_SAMPLES)
power = np.full_like(t, 25e3)  # [W]
steer = 0.12 * np.sin(2.0 * np.pi * t / 8.0)  # [rad]
u = np.vstack([power, steer])

traj = car.compute_trajectory(tf=TF, u=u, n_samples=N_SAMPLES, backend="jax")

# The bumpy ground under the drive, a margin around it
X, Y = traj.x[0], traj.x[1]
car.scene_bounds = (X.min() - 6, X.max() + 6, Y.min() - 6, Y.max() + 6)

# car.plot_trajectory()
car.animate(traj, renderer="meshcat")
