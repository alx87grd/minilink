"""A Kalman filter reconstructs the pendulum's angle and speed from the angle alone."""

import numpy as np

from minilink import Gain, Pendulum, Sine, kalman

Q_N = np.diag([1e-2, 1.0])  # process noise intensity on [theta, dtheta]
R_N = np.array(
    [[1e-4]]
)  # measurement noise intensity on the angle: the sensor is trusted

# The plant measured through its angle only: the sensor is part of the plant box
pendulum = Pendulum()
plant = pendulum >> Gain([[1.0, 0.0]])
torque = Sine(amplitude=0.5, omega=np.pi, port="u")

# The filter, designed on the plant linearized at its nominal state (hanging)
observer = kalman(plant, Q_N, R_N)
print(observer)

# Plant and observer driven by the same torque; the observer sees the angle only
diagram = (torque + plant + observer).autowire()
diagram.plot_diagram()

pendulum.x0 = np.array(
    [0.5, 0.0]
)  # the observer starts at rest: its estimate converges
diagram.compute_trajectory(tf=6.0)
diagram.plot_trajectory(signals=((pendulum, "x"), (observer, "x")))
