"""LQG balance of a cart-pole: a Kalman filter estimates the state, an LQR gain acts on it, under noise."""

import numpy as np

from minilink import CartPoleWithNoisePort, WhiteNoise, lqg

X_UP = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin
Q = np.diag([1.0, 10.0, 1.0, 1.0])  # LQR weights
R = np.array([[0.1]])
W = 1e-2  # intensity of the disturbance force on the cart
V = 1e-4  # intensity of the measurement noise on each state

plant = CartPoleWithNoisePort()

# The noise the filter is designed for: Q_n = B_w W B_wᵀ with B_w = ∂f/∂d, R_n = V I
B_w = plant.jacobian("f", "d", X_UP)
Q_N = W * B_w @ B_w.T + 1e-9 * np.eye(4)
R_N = V * np.eye(4)

# One design on the linear model at the upright: the gain and the filter in one box
controller = lqg(plant, Q, R, Q_N, R_N, x_bar=X_UP)
print(controller)

loop = controller @ plant
noisy = (
    WhiteNoise(1, psd=W, seed=1, port="d")
    + WhiteNoise(4, psd=V, seed=2, port="n")
    + loop
).autowire()
noisy.plot_diagram()

plant.x0 = np.array([-0.5, np.pi + 0.2, 0.0, 0.0])  # a disturbed start
noisy.compute_trajectory(tf=8.0)
noisy.plot_trajectory()
noisy.animate()
