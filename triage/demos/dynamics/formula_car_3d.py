"""A formula car in 3-D, open loop: a straight, a lift, then a long left-hander."""

import numpy as np

from minilink import FormulaCar3D

TF = 10.0
N_SAMPLES = 1001

car = FormulaCar3D(wheel_angles=True)
car.params["terrain"]["friction"] = 1.0  # [-] road grip: 1 dry, 0.7 wet, 0.1 ice

# The schedule, with no controller: build speed on the straight, lift for a second,
# then a steady left turn; the stiff racer stays flat through it. Without downforce
# the slicks hold about 7 kN at the rear: more power than that spins the car.
t = np.linspace(0.0, TF, N_SAMPLES)
power = np.where(t < 5.0, 100e3, np.where(t < 6.0, 0.0, 40e3))  # [W]
steer = np.where(t < 5.0, 0.0, 0.03)  # [rad]
u = np.vstack([power, steer])

traj = car.compute_trajectory(tf=TF, u=u, n_samples=N_SAMPLES)

# A grid on the ground under the drive, a margin around it
x, y = traj.x[0], traj.x[1]
car.scene_bounds = (x.min() - 10, x.max() + 10, y.min() - 10, y.max() + 10)

# car.plot_trajectory()
car.animate(traj, renderer="meshcat")
