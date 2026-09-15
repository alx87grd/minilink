"""Pendulum swing-up by both algorithm families: on-policy PPO and off-policy SAC, scored on one yardstick."""

import time

import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Pendulum
from minilink.control import angle_features
from minilink.planning import (
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

DT = 0.05  # control period of the learned laws
TORQUE = 4.0  # Nm, below m g l = 9.81 Nm: the pendulum must pump energy

# Plant: catalog pendulum, theta = 0 hanging, theta = pi upright, torque limited
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
plant.state.upper_bound = np.array([4 * np.pi, 20.0])


# Cost: J = int g dt with g zero upright, two hanging, periodic in the angle
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        return (1.0 + jnp.cos(theta)) + 0.01 * dtheta**2 + 0.01 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


# One task for both: infinite horizon, starts anywhere on the circle with small rates
problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
)
features = angle_features(angles=[0], scales={1: 0.1})

# On-policy PPO: many cheap samples, each used for a few epochs then discarded
ppo = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=features,
    hidden=(32, 32),
    algorithm="ppo",
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=3e-3,
    gamma=0.97,
    verbose=0,
)

# Off-policy SAC: fewer samples, replayed many times, one gradient step per plant step
sac = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=features,
    hidden=(64, 64),
    algorithm="sac",
    n_envs=8,
    n_steps=16,
    gamma=0.98,
    learning_starts=2000,
    verbose=0,
)

# Train each to a swing-up, then score both laws on the same 100 random starts
evaluator = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1)
fig, ax = plt.subplots(figsize=(8, 3))
for name, planner, budget in (("PPO", ppo, 120_000), ("SAC", sac, 30_000)):
    t0 = time.time()
    planner.solve(timesteps=budget)
    report = evaluator.evaluate(planner.get_controller())
    print(f"{name}: {planner.num_timesteps} plant steps in {time.time() - t0:.1f} s")
    print(f"  Monte Carlo {report}")
    planner.plot_learning_curve(ax=ax)
    planner.get_controller().plot_control_law(x_axis=0, y_axis=1, u_axis=0)
ax.legend(["PPO", "SAC"])
ax.set_title("mean episode return during training")

# The SAC law closes the loop on the continuous plant like any controller block
plant.x0 = np.array([0.05, 0.0])  # hanging, a tiny tip to break the symmetry
cl_sys = sac.get_controller() @ plant
cl_sys.name = "Pendulum with the SAC law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
plt.show()
cl_sys.animate(traj)
