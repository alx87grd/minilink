"""Learn to fly twice: stable-baselines3 PPO vs. the pure-JAX PPO, training timed."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Drone2D, Trajectory
from minilink.experimental.ppo_jax import PPO as JaxPPO
from minilink.interfaces.gymnasium import SB3Controller, Sys2Gym

training_timesteps = 100000  # same budget as the notebook (20k + 80k)


# Dynamics: same plant as examples/teaching/reinforcement_learning/drone_ppo_learn_to_fly.ipynb
class NormalizedDrone2D(Drone2D):
    """Planar drone with thrust inputs normalized between -1 and 1."""

    def __init__(self):
        super().__init__()

        # Parameters
        self.params["mass"] = 1.0  # kg
        self.params["inertia"] = 0.1  # kgm2

        # Normalized inputs
        self.inputs["u"].lower_bound = np.array([-1.0, -1.0])
        self.inputs["u"].upper_bound = np.array([+1.0, +1.0])
        self.inputs["u"].units = ["%", "%"]

        self.weight = self.params["gravity"] * self.params["mass"]
        self.thrust2weight = 1.2

        # Min/max states
        self.state.upper_bound = np.array([10, 10, 2 * np.pi, 10, 10, 10])
        self.state.lower_bound = -self.state.upper_bound

    def thrust(self, u):
        """Map a normalized input to thruster forces in Newtons."""
        return self.weight * ((self.thrust2weight - 1.0) * u + np.array([0.5, 0.5]))

    def f(self, x, u, t=0.0, params=None):
        return super().f(x, self.thrust(u), t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # Draw the thrust arrows using the de-normalized forces
        return super().get_dynamic_geometry(x, self.thrust(u), t, params)


plant = NormalizedDrone2D()


# Cost function: same quadratic running cost about hover
class CustomCostFunction(CostFunction):
    """
    J = int( g(x,u,t) * dt ) + h( x(T) , T )
    """

    def __init__(self):
        self.EPS = 0.1

        # Target state
        self.x_target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Quadratic cost weights
        self.Q = np.diag([1.0, 1.0, 6.0, 0.1, 0.1, 0.1])
        self.R = np.diag([0.001, 0.001])

        # Optional zone of zero cost if ||dx|| < EPS
        self.ontarget_check = False

    def g(self, x, u, t=0.0, params=None):
        """Quadratic additive running cost"""
        dx = x - self.x_target

        dJ = dx.T @ self.Q @ dx + u.T @ self.R @ u

        if self.ontarget_check:
            if np.linalg.norm(dx) < self.EPS:
                dJ = 0.0

        return dJ

    def h(self, x, t=0.0, params=None):
        """Terminal cost function with zero value"""
        return 0.0


cost = CustomCostFunction()

# Same exploration setup for both: Gaussian initial states around hover
plant.x0 = np.zeros(6)
x0_std = np.array([5.0, 5.0, 1.0, 1.0, 1.0, 0.2])

# --- stable-baselines3 PPO through the Gymnasium bridge ---
from stable_baselines3 import PPO  # noqa: E402

env = Sys2Gym(plant, cost, dt=0.05)  # note the time step used for discrete time
env.reset_mode = "gaussian"
env.x0_std = x0_std

nn = PPO("MlpPolicy", env, verbose=1)
sb3_ctl = SB3Controller(nn, sys=plant)

t0 = time.time()
nn.learn(training_timesteps)
sb3_time = time.time() - t0

# --- pure-JAX PPO on the compiled plant (no gym, no torch) ---
ppo = JaxPPO(plant, cost, dt=0.05, reset_mode="gaussian", x0_std=x0_std)
jax_ctl = ppo.controller

t0 = time.time()
ppo.learn(training_timesteps)
jax_time = time.time() - t0  # includes JIT compilation of the first iteration

# --- Looking at the policies: T1 vs (theta, omega) ---
sb3_ctl.plot_control_law(x_axis=2, y_axis=5, u_axis=0)
jax_ctl.plot_control_law(x_axis=2, y_axis=5, u_axis=0)

# --- Testing the closed-loop systems from the same offset initial state ---
plant.x0 = np.array([-1.0, -2.0, 1.0, 0.0, 0.0, 0.0])

cl_sb3 = sb3_ctl @ plant
cl_sb3.name = "Drone with SB3 PPO controller"
traj_sb3 = cl_sb3.compute_trajectory(tf=10.0, dt=0.01)
cl_sb3.plot_trajectory(traj_sb3)

cl_jax = jax_ctl @ plant
cl_jax.name = "Drone with JAX PPO controller"
traj_jax = cl_jax.compute_trajectory(tf=10.0, dt=0.01)
cl_jax.plot_trajectory(traj_jax)

# --- Performance: realized cost J along each closed-loop trajectory ---
u_sb3, _ = nn.predict(traj_sb3.x.T.astype(np.float32), deterministic=True)
J_sb3 = cost.evaluate_trajectory(Trajectory(t=traj_sb3.t, x=traj_sb3.x, u=u_sb3.T))

u_jax, _ = ppo.predict(traj_jax.x.T, deterministic=True)
J_jax = cost.evaluate_trajectory(Trajectory(t=traj_jax.t, x=traj_jax.x, u=u_jax.T))

print("\nTraining budget:", training_timesteps, "steps")
print(f"SB3 PPO : {sb3_time:6.1f} s  ({training_timesteps / sb3_time:6.0f} steps/s)")
print(f"JAX PPO : {jax_time:6.1f} s  ({training_timesteps / jax_time:6.0f} steps/s)")
print("Closed-loop cost J from x0 = [-1, -2, 1, 0, 0, 0]:")
print("SB3 PPO :", round(float(J_sb3.signals["cost"][0, -1]), 1))
print("JAX PPO :", round(float(J_jax.signals["cost"][0, -1]), 1))

plt.show()
cl_jax.animate(traj_jax)
