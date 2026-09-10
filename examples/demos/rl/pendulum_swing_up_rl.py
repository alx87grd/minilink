"""Pendulum swing-up as a stochastic planning problem solved by reinforcement learning."""

import jax.numpy as jnp
import numpy as np

from minilink import CostFunction, Pendulum
from minilink.control import angle_features
from minilink.planning import (
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

TRAINING_TIMESTEPS = 120_000  # swing-up learned by ~65k steps; margin to settle
DT = 0.05  # control period of the learned law
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
        return (
            (1.0 + jnp.cos(theta)) + 0.01 * dtheta**2 + 0.01 * u[0] ** 2
        )  # traces under JAX

    def h(self, x, t=0.0, params=None):
        return 0.0


# The problem: infinite horizon, starts anywhere on the circle with small rates
problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
)

# The planner: PPO on 64 plants in parallel, a small network on periodic
# features (cos theta, sin theta, scaled rate), a fast learning rate
planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=angle_features(angles=[0], scales={1: 0.1}),
    hidden=(32, 32),
    algorithm="ppo",
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=3e-3,
    gamma=0.97,
)
plan = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(f"\n{plan.metadata.message} in {plan.metadata.solve_time_s:.1f} s")
planner.plot_learning_curve()

# The learned law is a controller block: draw it, score it, close the loop
ppo_ctl = planner.get_controller()
ppo_ctl.plot_control_law(x_axis=0, y_axis=1, u_axis=0)  # torque vs (theta, dtheta)

report = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1).evaluate(ppo_ctl)
print("Monte Carlo over the task's starts:", report)

plant.x0 = np.array([0.05, 0.0])  # hanging, a tiny tip to break the symmetry
cl_sys = ppo_ctl @ plant
cl_sys.name = "Pendulum with the learned law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
angle_error = np.abs(np.mod(traj.x[0], 2 * np.pi) - np.pi)
print(
    "Time to reach upright:", round(float(traj.t[np.argmax(angle_error < 0.1)]), 2), "s"
)
print(
    "Angle error to upright, last 2 s:",
    round(float(angle_error[-200:].max()), 3),
    "rad",
)
cl_sys.animate(traj)
