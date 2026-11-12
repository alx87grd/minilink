"""An off-road side-by-side in 3-D, open loop: a weave over rolling hills."""

import numpy as np

from minilink import OffRoadCar3D
from minilink.dynamics.catalog.vehicles.terrain import Bumps

TF = 12.0
N_SAMPLES = 1201

car = OffRoadCar3D(terrain=Bumps(), wheel_angles=True)
car.params["terrain"]["amplitude"] = 0.8  # [m] hills, crest to trough 1.6 m
car.params["terrain"]["wavelength"] = 30.0  # [m]
car.params["terrain"]["friction"] = 1.0  # [-] road grip: 1 dry, 0.7 wet, 0.1 ice

# The schedule, with no controller: steady power, the steering weaving left and
# right. The soft springs follow the hills, the body pitches up the climbs and the
# tall car leans in the turns
t = np.linspace(0.0, TF, N_SAMPLES)
power = np.full_like(t, 12e3)  # [W]
steer = 0.10 * np.sin(2.0 * np.pi * t / 8.0)  # [rad]
u = np.vstack([power, steer])

traj = car.compute_trajectory(tf=TF, u=u, n_samples=N_SAMPLES)

# The hills under the drive, a margin around it
x, y = traj.x[0], traj.x[1]
car.scene_bounds = (x.min() - 6, x.max() + 6, y.min() - 6, y.max() + 6)

# car.plot_trajectory()
car.animate(traj, renderer="meshcat")
