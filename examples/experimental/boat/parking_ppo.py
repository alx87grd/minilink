"""Boat parking learned by PPO: the stochastic parking problem solved by the reinforcement learning planner."""

import numpy as np
from parking import CURRENT, X_DOCK, X_START, ParkingBoat, dock_features, parking_cost

from minilink import (
    Gaussian,
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
)

TRAINING_TIMESTEPS = 3_000_000
DT = 0.1  # [s] control period
EPISODE = 60.0  # [s]
GAMMA = 0.995  # a 20 s discount horizon: the dock is a few hull lengths away
INFEASIBLE_COST = 2000.0  # leaving the box: more than any run that stays in it

boat = ParkingBoat(CURRENT)

# Starts anywhere a boat might come from, within the hull's reach of the dock
problem = StochasticPlanningProblem(
    boat,
    cost=parking_cost(boat),
    tf=np.inf,
    x0_distribution=Gaussian(
        [-10.0, -5.0, X_DOCK[2], 0.0, 0.0, 0.0], [12.0, 12.0, 1.5, 0.5, 0.5, 0.2]
    ),
    X=boat.state.box,
    infeasible_cost=INFEASIBLE_COST,
)


planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=dock_features,
    hidden=(64, 64),
    algorithm="ppo",
    n_envs=64,
    n_steps=64,
    batch_size=512,
    learning_rate=3e-4,
    gamma=GAMMA,
    episode_length=EPISODE,
    log_std_init=-0.5,
)
solution = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(solution.solver)
planner.plot_learning_curve()

ppo_ctl = planner.get_controller()
ppo_ctl.plot_control_law(x_axis=0, y_axis=1, u_axis=0)  # surge thrust vs (x, y)

report = MonteCarloEvaluator(
    problem, dt=DT, n_trials=100, seed=1, episode_length=EPISODE
).evaluate(ppo_ctl)
print(report)

boat.x0 = X_START.copy()
cl = ppo_ctl @ boat
cl.name = "Boat with the learned parking law"
traj = cl.compute_trajectory(tf=EPISODE, dt=0.05)
cl.plot_trajectory(traj)
cl.animate(traj)
