"""Cart-pole swing-up learned by the pure-JAX PPO on the compiled plant."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import CartPole, CostFunction, Trajectory
from minilink.core.backends import array_module
from minilink.experimental.ppo_jax import PPO

TRAINING_TIMESTEPS = 3_000_000  # about 30 s on a laptop CPU (16 vmapped plants)
DT = 0.05  # control period, one RK4 step per period
TF = 10.0  # episode duration

# Plant: catalog cart-pole, theta = 0 hanging, theta = pi upright, force |F| <= 10 N.
# The state bounds define the training box: an episode ends when the cart or
# the pole angle leaves it (bootstrapped, as in the Gymnasium bridge).
plant = CartPole()
plant.inputs["u"].lower_bound = np.array([-10.0])
plant.inputs["u"].upper_bound = np.array([10.0])
plant.state.lower_bound = np.array([-5.0, -4 * np.pi, -20.0, -30.0])
plant.state.upper_bound = np.array([5.0, 4 * np.pi, 20.0, 30.0])
plant.x0 = np.zeros(4)  # hanging at rest


# Cost: J = int g dt with g = (1 + cos theta) + 0.1 x^2 + small velocity and effort
# penalties. The angle term is zero upright, two hanging, and periodic, so the
# policy is rewarded for any route to the top; the velocity weight stays small
# so the fast swing needed to pump energy is not punished.
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        xp = array_module(x)
        x_cart, theta, dx, dtheta = x
        return (
            (1.0 + xp.cos(theta))
            + 0.1 * x_cart**2
            + 0.001 * (dx**2 + dtheta**2)
            + 0.001 * u[0] ** 2
        )

    def h(self, x, t=0.0, params=None):
        return 0.0


cost = SwingUpCost()


# Observation features: the networks see (cos theta, sin theta) instead of the
# raw angle, so the law is periodic and never extrapolates past the training
# range; velocities are scaled to order one. The learned law is still u = pi(x).
def features(x):
    xp = array_module(x)
    x_cart, theta, dx, dtheta = x
    return xp.array([x_cart, xp.cos(theta), xp.sin(theta), 0.2 * dx, 0.1 * dtheta])


# PPO: uniform initial angles over the full circle (the policy must handle
# every phase of the swing), 16 plants simulated in parallel per rollout.
ppo = PPO(
    plant,
    cost,
    dt=DT,
    tf=TF,
    reset_mode="uniform",
    x0_lb=np.array([-1.0, -np.pi, -1.0, -1.0]),
    x0_ub=np.array([1.0, np.pi, 1.0, 1.0]),
    features=features,
    n_envs=16,
    n_steps=256,
    batch_size=512,
    gamma=0.99,
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

# The learned law: force vs (theta, dtheta) with the cart pinned at rest
ppo_ctl.plot_control_law(x_axis=1, y_axis=3, u_axis=0)

# Closed loop from hanging, a tiny tip to break the symmetry of the exact equilibrium
plant.x0 = np.array([0.0, 0.05, 0.0, 0.0])
cl_sys = ppo_ctl @ plant
cl_sys.name = "Cart-pole with PPO controller (JAX)"
traj = cl_sys.compute_trajectory(tf=TF, dt=0.01)
cl_sys.plot_trajectory(traj)

# Realized cost J along the closed-loop flight, inputs rebuilt from the policy
u_sim, _ = ppo.predict(traj.x.T, deterministic=True)
plant_traj = cost.evaluate_trajectory(Trajectory(t=traj.t, x=traj.x, u=u_sim.T))
angle_error = np.abs(np.mod(traj.x[1], 2 * np.pi) - np.pi)
print("Total trajectory cost J =", round(float(plant_traj.signals["cost"][0, -1]), 1))
print(
    "Angle error to upright, last 2 s:",
    round(float(angle_error[-200:].max()), 3),
    "rad",
)
print("Cart excursion max |x| =", round(float(np.abs(traj.x[0]).max()), 2), "m")

plt.show()
cl_sys.animate(traj)
