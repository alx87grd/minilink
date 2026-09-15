"""The policy-gradient ladder on a pendulum: REINFORCE, actor-critic and PPO on one axis, scored on one yardstick."""

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

DT = 0.05
TORQUE = 12.0  # Nm, above m g l = 9.81 Nm: the pendulum can lift itself, a task every rung learns

# Plant: catalog pendulum, theta = 0 hanging, theta = pi upright; the angle is unbounded
# (the policy sees it through periodic features), the speed box is the only exit
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-1e3, -20.0])
plant.state.upper_bound = np.array([1e3, 20.0])


# Cost: zero upright, two hanging, periodic in the angle
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        return (1.0 + jnp.cos(theta)) + 0.01 * dtheta**2 + 0.001 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
)
features = angle_features(angles=[0], scales={1: 0.1})

# The ladder: Monte Carlo returns and no critic; a critic's TD errors and one step per batch;
# the same with a clipped ratio and several passes. Each rung gets the budget it needs.
ladder = {
    "REINFORCE": (
        dict(algorithm="reinforce", n_envs=16, learning_rate=1e-2, episode_length=4.0),
        200_000,
    ),
    "actor-critic": (
        dict(algorithm="actor_critic", n_envs=16, n_steps=32, learning_rate=3e-3),
        300_000,
    ),
    "PPO": (
        dict(
            algorithm="ppo", n_envs=64, n_steps=32, batch_size=256, learning_rate=3e-3
        ),
        100_000,
    ),
}

evaluator = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1)
fig, ax = plt.subplots(figsize=(8, 3))
laws = {}
for name, (settings, budget) in ladder.items():
    planner = ReinforcementLearningPlanner(
        problem, dt=DT, features=features, hidden=(32, 32), gamma=0.97, **settings
    )
    solution = planner.solve(timesteps=budget)
    print(f"{name:13s} {solution.solver}")
    planner.plot_learning_curve(ax=ax)
    laws[name] = planner.get_controller()
ax.legend(list(ladder))
ax.set_title("mean episode return during training")

# One yardstick for the three laws, and their torque maps
for name, law in laws.items():
    print(f"{name:13s} Monte Carlo over the task's starts:", evaluator.evaluate(law))
    law.plot_control_law(x_axis=0, y_axis=1, u_axis=0, show=False)
plt.show()

# The PPO law closes the loop on the continuous plant like any controller block
plant.x0 = np.array([0.05, 0.0])
cl_sys = laws["PPO"] @ plant
cl_sys.name = "Pendulum with the PPO law"
traj = cl_sys.compute_trajectory(tf=6.0, dt=0.01)
cl_sys.plot_trajectory(traj)
cl_sys.animate(traj)
