"""Planar rocket landing learned by the pure-JAX PPO: gimballed thrust, ground termination."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Rocket, Trajectory
from minilink.experimental.ppo_jax import PPO

TRAINING_TIMESTEPS = 4_000_000  # lands from every test start by ~3.8M steps (~80 s)
DT = 0.05  # control period, one RK4 step per period
TF = 15.0  # episode duration
GIMBAL = 0.05  # rad, nozzle deflection limit
THRUST_TO_WEIGHT = 2.0  # max thrust as a multiple of the weight

# Plant: catalog planar rocket, y up, ground at y = 0, c.g. one metre above
# the nozzle. Inputs are the thrust magnitude (one-sided: no negative thrust)
# and the gimbal angle. The catalog inertia (100 kgm2 for 1000 kg) makes the
# rocket tumble within a second at a 0.05 rad gimbal; a rocket-like value keeps
# the attitude loop learnable by random exploration.
plant = Rocket()
plant.params["inertia"] = 1000.0
weight = plant.params["mass"] * plant.params["gravity"]
plant.inputs["u"].lower_bound = np.array([0.0, -GIMBAL])
plant.inputs["u"].upper_bound = np.array([THRUST_TO_WEIGHT * weight, GIMBAL])
# The training box: touching the ground (y < 0) ends the episode with the
# terminal cost h (a crash); the other faces are far enough to be irrelevant.
plant.state.lower_bound = np.array([-100.0, 0.0, -1.0, -50.0, -50.0, -5.0])
plant.state.upper_bound = np.array([100.0, 200.0, 1.0, 50.0, 50.0, 5.0])
plant.x0 = np.array([0.0, 20.0, 0.0, 0.0, 0.0, 0.0])

X_LANDED = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])  # upright on the pad, at rest


# Cost: J = int g dt + h(x_f), quadratic about the landed state with a heavy
# weight on attitude, plus a fixed crash penalty when the episode terminates
# on the ground. The 0.1 scale keeps the returns of the order of 100, which
# the critic fits quickly; the crash must cost more than a gentle landing.
class LandingCost(CostFunction):
    Q = 0.1 * np.diag([1.0, 1.0, 10.0, 0.1, 0.1, 1.0])
    R = 0.1 * np.diag([1e-8, 1.0])

    def g(self, x, u, t=0.0, params=None):
        dx = x - X_LANDED
        return dx @ self.Q @ dx + u @ self.R @ u

    def h(self, x, t=0.0, params=None):
        return 100.0  # crash


cost = LandingCost()

# Observation features: error to the landed state, positions and speeds
# scaled so that a few metres map to order one. The law is still u = pi(x).
FEATURE_SCALE = np.array([0.25, 0.25, 1.0, 0.25, 0.25, 1.0])


def features(x):
    return (x - X_LANDED) * FEATURE_SCALE


# PPO: Gaussian starts around 20 m altitude with lateral offsets, a gentle
# initial exploration (log-std -1) so the attitude does not tumble, a long
# discount horizon for the slow lateral dynamics, 64 plants in parallel.
ppo = PPO(
    plant,
    cost,
    dt=DT,
    tf=TF,
    reset_mode="gaussian",
    x0_std=np.array([10.0, 8.0, 0.3, 2.0, 2.0, 0.3]),
    domain_exit="terminate",
    features=features,
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=1e-3,
    gamma=0.995,
    log_std_init=-1.0,
    seed=0,
)
ppo_ctl = ppo.controller

t0 = time.time()
ppo.learn(TRAINING_TIMESTEPS)
print(f"\nTrained {ppo.num_timesteps} steps in {time.time() - t0:.1f} s")

# Learning curve: mean return of the exploration episodes
steps = [h["timesteps"] for h in ppo.history]
ep_return = [h["ep_return_mean"] for h in ppo.history]
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(steps, ep_return)
ax.set_xlabel("timesteps")
ax.set_ylabel("mean episode return")
ax.grid(True, alpha=0.3)

# The learned thrust law vs (altitude, vertical speed) with the rocket upright
ppo_ctl.plot_control_law(x_axis=1, y_axis=4, u_axis=0)

# Closed loop: a landing from 30 m altitude, 10 m to the side
plant.x0 = np.array([10.0, 30.0, 0.0, 0.0, 0.0, 0.0])
cl_sys = ppo_ctl @ plant
cl_sys.name = "Rocket with PPO controller (JAX)"
traj = cl_sys.compute_trajectory(tf=TF, dt=0.01)
cl_sys.plot_trajectory(traj)

# Realized cost J and the touchdown quality, inputs rebuilt from the policy
u_sim, _ = ppo.predict(traj.x.T, deterministic=True)
plant_traj = cost.evaluate_trajectory(Trajectory(t=traj.t, x=traj.x, u=u_sim.T))
final = traj.x[:, -1] - X_LANDED
print("Total trajectory cost J =", round(float(plant_traj.signals["cost"][0, -1]), 1))
print("Final position error:", round(float(np.hypot(final[0], final[1])), 2), "m")
print("Final attitude:", round(float(final[2]), 3), "rad")
print("Final speed:", round(float(np.hypot(final[3], final[4])), 2), "m/s")
print(
    "Lowest altitude of the c.g.:",
    round(float(traj.x[1].min()), 2),
    "m (1.0 = on the pad)",
)

plt.show()
cl_sys.animate(traj)
