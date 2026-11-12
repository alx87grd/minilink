"""Cart-pole LQG: a Kalman filter and an LQR gain balancing under noise, scored over many runs."""

import numpy as np

from minilink import (
    NoisyCartPole,
    PlanningProblem,
    QuadraticCost,
    WhiteNoise,
    closed_loop,
    kalman,
    lqr,
)
from minilink.control import lqg

TF = 8.0  # [s]
N_TRIALS = 200  # runs of the Monte Carlo

# The plant: a cart-pole with a disturbance force d on the cart and a noise n on each
# of its four measurements, started 0.5 m and 0.2 rad off its upright

x_up = np.array([0.0, np.pi, 0.0, 0.0])

plant = NoisyCartPole()
plant.x0 = np.array([-0.5, np.pi + 0.2, 0.0, 0.0])

# The noise: white, of intensity w on the force and v on each measurement; the filter
# is designed for it, Q_n = B_w w B_wᵀ with B_w = ∂f/∂d, and R_n = v I

w = 1e-2  # [N^2 s]
v = 1e-4

B_w = plant.jacobian("f", "d", x_up)
Q_n = w * B_w @ B_w.T + 1e-9 * np.eye(4)
R_n = v * np.eye(4)

# The two designs on the linear model at the upright: the gain and the filter

Q = np.diag([1.0, 10.0, 1.0, 1.0])
R = np.array([[0.1]])

gain = lqr(plant, Q, R, x_bar=x_up)
observer = kalman(plant, Q_n, R_n, x_bar=x_up)

# One run: the loop by role, a noise source on each of the plant's two noise ports

loop = closed_loop(
    gain,
    plant,
    observer=observer,
    disturbance=WhiteNoise(1, psd=w, seed=1),
    noise=WhiteNoise(4, psd=v, seed=2),
)
loop.plot_diagram()

loop.compute_trajectory(tf=TF)
loop.plot_trajectory()
loop.animate()

# Many runs: the same task as a problem, the two noises its disturbances, each drawn
# anew at every trial; the compensator is the gain on the filter's estimate, one block

cost = QuadraticCost.from_system(
    plant, Q=Q, R=np.diag([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]), x_bar=x_up
)
problem = PlanningProblem(
    plant,
    x0=plant.x0,
    cost=cost,
    horizon=TF,
    disturbances={
        "d": WhiteNoise(1, psd=w, sample_period=0.01, seed=1),
        "n": WhiteNoise(4, psd=v, sample_period=0.01, seed=2),
    },
)
compensator = lqg(plant, Q, R, Q_n, R_n, x_bar=x_up)

runs = problem.compile(compensator, dt=0.01, backend="jax").evaluate(
    n_trials=N_TRIALS, key=1
)
print(runs)
