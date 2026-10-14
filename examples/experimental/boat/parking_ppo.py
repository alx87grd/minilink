"""Boat parking learned by PPO: the stochastic parking problem solved by the reinforcement learning planner."""

import numpy as np
from parking import (
    CURRENT,
    POLICY_HIDDEN,
    POLICY_WEIGHTS,
    X_START,
    ParkingBoat,
    dock_features,
    parking_cost,
)

from minilink import (
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)
from minilink.graphical.catalog.boat_skin import boat_skin_3d

TRAINING_TIMESTEPS = 1_500_000  # about two minutes on a laptop CPU
DT = 0.1  # [s] control period
TF = 40.0  # [s] a finite horizon: the episode ends at tf with the terminal pose cost
START_RADIUS = 25.0  # [m] training starts anywhere in this box round the dock
DOCK_WELL = 1.0  # a bounded reward for the last metres, where the quadratic is flat
TERMINAL = 10.0  # the pose error at the end of the episode, in units of g
INFEASIBLE_COST = 50.0  # leaving the box: more than any run that stays in it
GAMMA = 0.99  # a 10 s discount horizon at DT

boat = ParkingBoat(CURRENT)

problem = StochasticPlanningProblem(
    boat,
    cost=parking_cost(boat, terminal=TERMINAL, well=DOCK_WELL),
    tf=TF,
    x0_distribution=Uniform(
        [-START_RADIUS, -START_RADIUS, -np.pi, -0.5, -0.5, -0.2],
        [START_RADIUS, START_RADIUS, np.pi, 0.5, 0.5, 0.2],
    ),
    X=boat.state.box,
    infeasible_cost=INFEASIBLE_COST,
)

planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=dock_features,
    hidden=POLICY_HIDDEN,
    algorithm="ppo",
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=1e-3,
    gamma=GAMMA,
    log_std_init=-1.5,  # gentle exploration: a 10 t hull answers slowly
)
solution = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(solution.solver)
planner.plot_learning_curve()

ppo_ctl = planner.get_controller()
np.savez(
    POLICY_WEIGHTS, **ppo_ctl.params["mlp"]
)  # parking_compare.py rebuilds the law from these
ppo_ctl.plot_control_law(x_axis=0, y_axis=1, u_axis=0)  # surge thrust vs (x, y)

report = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1).evaluate(ppo_ctl)
print(report)

boat.x0 = X_START.copy()
cl = ppo_ctl @ boat
cl.name = "Boat with the learned parking law"
traj = cl.compute_trajectory(tf=60.0, dt=0.05)
cl.plot_trajectory(traj)
cl.animate(traj)

boat.skin = boat_skin_3d  # the hull on the water, in the browser
cl.animate(traj, renderer="meshcat", native=False)
