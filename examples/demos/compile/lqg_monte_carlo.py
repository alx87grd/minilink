"""Monte Carlo of an LQG loop: noise sources on the plant's ports, a seed per trial, the mean against its prediction."""

import numpy as np

from minilink import Gaussian, NoisyPendulum, PlanningProblem, QuadraticCost, WhiteNoise
from minilink.analysis import linearize
from minilink.blocks import NoiseSource
from minilink.control import lqg

TF = 5.0
DT = 0.01  # the noise blocks' sample period and the run's step, s
W = 0.04  # the torque disturbance's white-noise intensity, (N·m)²·s
SENSOR_STD = 1.0  # the rate sensor's datasheet noise, per sample, rad/s
N_TRIALS = 400

# The plant: a pendulum with a torque disturbance port d and a rate-noise port n
plant = NoisyPendulum()
Q, r = np.diag([10.0, 1.0]), 0.1
V = (
    SENSOR_STD**2 * DT
)  # the sensor's intensity: its per-sample variance over its period

# The LQG compensator: an LQR gain on a stationary Kalman estimate, one box
box = lqg(plant, Q, np.array([[r]]), Q_n=np.diag([0.0, W]), R_n=np.diag([1e-4, V]))

# The task: the plant regulated at rest, the cost on its state and its command, the noise
# two source blocks the problem wires onto their ports
cost = QuadraticCost.from_system(plant, Q=Q, R=np.diag([r, 0.0, 0.0]))
disturbances = {
    "d": WhiteNoise(1, psd=W, sample_period=DT, seed=1),
    "n": NoiseSource(Gaussian([0.0], [SENSOR_STD]), sample_period=DT, seed=2),
}
problem = PlanningProblem(
    plant, cost=cost, x_start=np.zeros(2), tf=TF, disturbances=disturbances
)

# Monte Carlo: N_TRIALS closed-loop runs, each source with its own seed per trial
runs = problem.compile(box, dt=DT, backend="jax").evaluate(n_trials=N_TRIALS, key=1)
J = np.asarray(runs.J)
sem = J.std(ddof=1) / np.sqrt(N_TRIALS)

# The prediction: E[J] = ∫₀ᵀ tr(M P(t)) dt, Ṗ = A P + P Aᵀ + N, from a zero start
lin = linearize(plant)
K = box.subsystems["gain"].params["K"]
L = box.subsystems["observer"].params["L"]
A, C = lin.A(), lin.C()
B_u, B_d, D_n = lin.B()[:, :1], lin.B()[:, 1:2], lin.D()[:, 2:3]
A_cl = np.block([[A, -B_u @ K], [L @ C, A - B_u @ K - L @ C]])
B_w = np.vstack([B_d, np.zeros((2, 1))])
B_v = np.vstack([np.zeros((2, 1)), L @ D_n])
N = W * B_w @ B_w.T + V * B_v @ B_v.T
M = np.block([[Q, np.zeros((2, 2))], [np.zeros((2, 2)), r * K.T @ K]])

h = 1e-3
P = np.zeros((4, 4))
expected = 0.0
for _ in range(round(TF / h)):
    # Ṗ = A P + P Aᵀ + N by an Euler step; E[J] += tr(M P) h
    P_next = P + h * (A_cl @ P + P @ A_cl.T + N)
    expected += 0.5 * h * (np.trace(M @ P) + np.trace(M @ P_next))
    P = P_next

print(f"Monte Carlo: E[J] = {J.mean():.4f} ± {sem:.4f} over {N_TRIALS} trials")
print(f"prediction:  E[J] = {expected:.4f} (the loop's covariance from a zero start)")
print(
    f"ratio {J.mean() / expected:.3f}, {abs(J.mean() - expected) / sem:.1f} standard errors"
)

# The same loop with its computer sampled: the noise blocks realized per trial alike
sampled = problem.compile(box % DT, backend="jax").evaluate(n_trials=N_TRIALS, key=1)
print(f"sampled computer: E[J] = {np.mean(sampled.J):.4f}")
