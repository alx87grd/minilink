"""Pendulum swing-up as a planning problem: PPO and SAC through the RL planner, scored by Monte Carlo."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Pendulum
from minilink.core.backends import array_module
from minilink.planning import (
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

DT = 0.05  # control period
TORQUE = 4.0  # Nm, below m g l = 9.81 Nm: the pendulum must pump

# Plant: catalog pendulum, theta = 0 hanging, pi upright, torque limited
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
plant.state.upper_bound = np.array([4 * np.pi, 20.0])


# Cost: J = int g dt, g zero upright, two hanging, periodic in the angle;
# infinite horizon (tf = inf), undiscounted: the planners pick their own gamma.
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        xp = array_module(x)
        theta, dtheta = x
        return (1.0 + xp.cos(theta)) + 0.01 * dtheta**2 + 0.01 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


# The task: uniform starts over the whole circle with small rates
problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
)


# Policy features: periodic angle, scaled rate (the law is still u = pi(x))
def features(x):
    xp = array_module(x)
    theta, dtheta = x
    return xp.array([xp.cos(theta), xp.sin(theta), 0.1 * dtheta])


# Two algorithms, one planner: on-policy PPO (fast wall time, more steps) and
# off-policy SAC (fewer steps, one gradient step per plant step).
planners = {
    "PPO": ReinforcementLearningPlanner(
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
    ),
    "SAC": ReinforcementLearningPlanner(
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
    ),
}
budgets = {"PPO": 120_000, "SAC": 30_000}

evaluator = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1)
fig, ax = plt.subplots(figsize=(8, 3))
for name, planner in planners.items():
    t0 = time.time()
    planner.solve(timesteps=budgets[name])
    print(f"{name}: {planner.num_timesteps} steps in {time.time() - t0:.1f} s")
    print(f"  {planner.env.describe()}")

    # Score the learned law on 100 random starts of the same task
    report = evaluator.evaluate(planner.get_controller())
    print(f"  Monte Carlo {report}")

    # Deterministic swing-up from hanging, on the training environment
    plan = planner.solve_trajectory_from(np.array([0.05, 0.0]))
    angle_error = np.abs(np.mod(plan.trajectory.x[0], 2 * np.pi) - np.pi)
    print(
        f"  from hanging: J = {plan.metadata.cost:.1f}, final angle error {angle_error[-1]:.3f} rad"
    )

    planner.plot_learning_curve(ax=ax)
    planner.get_controller().plot_control_law(x_axis=0, y_axis=1, u_axis=0)

ax.legend(list(planners))
ax.set_title("exploration return during training")

# The closed loop with the PPO law, simulated on the continuous plant
ctl = planners["PPO"].get_controller()
plant.x0 = np.array([0.05, 0.0])
cl_sys = ctl @ plant
cl_sys.name = "Pendulum with the learned law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)

plt.show()
cl_sys.animate(traj)
