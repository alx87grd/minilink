"""Underactuated pendulum swing-up learned in seconds by the pure-JAX PPO."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Pendulum, Trajectory
from minilink.core.backends import array_module
from minilink.experimental.ppo_jax import PPO

TRAINING_TIMESTEPS = 120_000  # swing-up learned by ~65k steps; margin to settle
DT = 0.05  # control period, one RK4 step per period
TF = 10.0  # episode duration
TORQUE = 4.0  # Nm, well below m g l = 9.81 Nm: the pendulum cannot be lifted directly

# Plant: catalog pendulum (m = 1 kg, l = 1 m, I = 1 kgm2), theta = 0 hanging,
# theta = pi upright. The torque limit makes it underactuated, so the only
# way up is to pump energy by swinging back and forth in phase with gravity.
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
plant.state.upper_bound = np.array([4 * np.pi, 20.0])
plant.x0 = np.zeros(2)  # hanging at rest


# Cost: J = int g dt with g = (1 + cos theta) + 0.01 dtheta^2 + 0.01 tau^2.
# The angle term is zero upright, two hanging, and periodic; the velocity
# and effort weights are small enough not to punish the swing itself.
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        xp = array_module(x)
        theta, dtheta = x
        return (1.0 + xp.cos(theta)) + 0.01 * dtheta**2 + 0.01 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


cost = SwingUpCost()


# Observation features: (cos theta, sin theta, scaled rate) so the law is
# periodic in the angle. The learned controller is still u = pi(x).
def features(x):
    xp = array_module(x)
    theta, dtheta = x
    return xp.array([xp.cos(theta), xp.sin(theta), 0.1 * dtheta])


# PPO tuned for speed on this plant: 64 plants in parallel with short 32-step
# rollouts (2048 samples per update), a small 32-unit network, a learning
# rate ten times the usual default, and a 0.97 discount (about 1.7 s horizon,
# enough for one swing). Initial angles cover the whole circle.
ppo = PPO(
    plant,
    cost,
    dt=DT,
    tf=TF,
    reset_mode="uniform",
    x0_lb=np.array([-np.pi, -1.0]),
    x0_ub=np.array([np.pi, 1.0]),
    features=features,
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=3e-3,
    gamma=0.97,
    hidden=(32, 32),
    seed=0,
)
ppo_ctl = ppo.controller

t0 = time.time()
ppo.learn(TRAINING_TIMESTEPS)
print(f"\nTrained {ppo.num_timesteps} steps in {time.time() - t0:.1f} s")

# Learning curve: mean return of the exploration episodes (-J of a 10 s episode)
steps = [h["timesteps"] for h in ppo.history]
ep_return = [h["ep_return_mean"] for h in ppo.history]
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(steps, ep_return)
ax.set_xlabel("timesteps")
ax.set_ylabel("mean episode return")
ax.grid(True, alpha=0.3)

# The learned law over the whole phase plane: torque vs (theta, dtheta)
ppo_ctl.plot_control_law(x_axis=0, y_axis=1, u_axis=0)

# Closed loop from hanging, a tiny tip to break the symmetry of the exact equilibrium
plant.x0 = np.array([0.05, 0.0])
cl_sys = ppo_ctl @ plant
cl_sys.name = "Pendulum with PPO controller (JAX)"
traj = cl_sys.compute_trajectory(tf=TF, dt=0.01)
cl_sys.plot_trajectory(traj)

# Realized cost J along the swing-up, inputs rebuilt from the policy
u_sim, _ = ppo.predict(traj.x.T, deterministic=True)
plant_traj = cost.evaluate_trajectory(Trajectory(t=traj.t, x=traj.x, u=u_sim.T))
angle_error = np.abs(np.mod(traj.x[0], 2 * np.pi) - np.pi)
print("Total trajectory cost J =", round(float(plant_traj.signals["cost"][0, -1]), 1))
print(
    "Angle error to upright, last 2 s:",
    round(float(angle_error[-200:].max()), 3),
    "rad",
)
print(
    "Time to reach upright:", round(float(traj.t[np.argmax(angle_error < 0.1)]), 2), "s"
)

plt.show()
cl_sys.animate(traj)
