"""A formula car in 3-D, open loop: a straight, a lift, then a long left-hander.

No controller: the power and steering commands are a fixed schedule. Watch the
stiff racer stay flat through the corner, its four slicks steering and spinning. Run
with JAX (the 3-D car's stiff tires make the NumPy path slow).
"""

import numpy as np

from minilink import FormulaCar3D

TF = 10.0
N_SAMPLES = 1001

car = FormulaCar3D(wheel_angles=True)

# The schedule: build speed on the straight, lift for a second, then a steady left
# turn. Without downforce the slicks hold about 7 kN at the rear: more power than that
# spins the car.
t = np.linspace(0.0, TF, N_SAMPLES)
power = np.where(t < 5.0, 100e3, np.where(t < 6.0, 0.0, 40e3))  # [W]
steer = np.where(t < 5.0, 0.0, 0.03)  # [rad]
u = np.vstack([power, steer])

traj = car.compute_trajectory(tf=TF, u=u, n_samples=N_SAMPLES, backend="jax")

# A floor under the drive, a margin around it
x, y = traj.x[0], traj.x[1]
car.scene_bounds = (x.min() - 10, x.max() + 10, y.min() - 10, y.max() + 10)

# car.plot_trajectory()
car.animate(traj, renderer="meshcat")
