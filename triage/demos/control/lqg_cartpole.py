"""LQG balance of a cart-pole: a Kalman filter estimates the state, an LQR gain acts on it, under noise."""

import numpy as np

from minilink import NoisyCartPole, WhiteNoise, closed_loop, kalman, lqr

X_UP = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin
Q = np.diag([1.0, 10.0, 1.0, 1.0])  # LQR weights
R = np.array([[0.1]])
W = 1e-2  # intensity of the disturbance force on the cart
V = 1e-4  # intensity of the measurement noise on each state

plant = NoisyCartPole()
plant.x0 = np.array([-0.5, np.pi + 0.2, 0.0, 0.0])  # a disturbed start

# The noise the filter is designed for: Q_n = B_w W B_wᵀ with B_w = ∂f/∂d, R_n = V I
B_w = plant.jacobian("f", "d", X_UP)
Q_N = W * B_w @ B_w.T + 1e-9 * np.eye(4)
R_N = V * np.eye(4)

# The two designs on the linear model at the upright: the gain and the filter
gain = lqr(plant, Q, R, x_bar=X_UP)
observer = kalman(plant, Q_N, R_N, x_bar=X_UP)
print(observer)

# The LQG loop by role; the plant's own d and n are driven by two noise sources
loop = closed_loop(
    gain,
    plant,
    observer=observer,
    disturbance=WhiteNoise(1, psd=W, seed=1),
    noise=WhiteNoise(4, psd=V, seed=2),
)
loop.plot_diagram()

loop.compute_trajectory(tf=8.0)
loop.plot_trajectory()
loop.animate()
