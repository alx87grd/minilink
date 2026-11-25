"""One pendulum swing-up, four planners, one judge: each nominal trajectory beside its simulation, then over many starts."""

import numpy as np

from minilink import (
    BallSet,
    DynamicProgrammingPlanner,
    KinodynamicExtender,
    LQRPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    TrajectoryOptimizationPlanner,
    Uniform,
    lqr_trajectory,
)

########################################################
### Tunables
########################################################

TF = 5.0  # [s]
N_TRIALS = 100

########################################################
### The plant
########################################################

plant = Pendulum()
plant.state.lower = np.array([-2.0 * np.pi, -8.0])
plant.state.upper = np.array([2.0 * np.pi, 8.0])

########################################################
### The task
########################################################

x_up = np.array([np.pi, 0.0])

cost = QuadraticCost.from_system(plant, x_bar=x_up, Q=np.eye(2), R=0.1 * np.eye(1))
problem = PlanningProblem(
    plant,
    x0=np.zeros(2),
    cost=cost,
    X=plant.state.box,
    U=plant.input_box,
    Xf=BallSet(x_up, 0.3),
    horizon=TF,
)

########################################################
### Four planners and one tracked plan
########################################################

lqr = LQRPlanner(problem).solve()

vi = DynamicProgrammingPlanner(
    problem.replace(horizon=None),
    x_grid=(101, 101),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    max_iterations=2000,
    out_of_bound_cost=500.0,
).solve()

trajopt = TrajectoryOptimizationPlanner(
    problem, n_steps=25, transcription="direct_collocation"
).solve()

# MPC

# from minilink.control import ModelPredictiveController

# mpc_problem = problem.replace(horizon=2.0, Xf=None)
# mpc_planner = TrajectoryOptimizationPlanner(
#     problem, n_steps=20, transcription="direct_collocation"
# )
# mpc = ModelPredictiveController(mpc_planner, delay=False) % 0.05

# mpc_pendulum = mpc @ plant
# mpc_pendulum.plot_trajectory()

torques = [np.array([tau]) for tau in (-5.0, -2.0, 0.0, 2.0, 5.0)]
rrt = RRTPlanner(
    problem,
    extender=KinodynamicExtender(controls=torques, horizon=0.3, n_steps=6),
    seed=0,
    max_iterations=20000,
).solve()

tracked = lqr_trajectory(plant, trajopt.trajectory, Q=np.diag([10.0, 1.0]), R=np.eye(1))

########################################################
### Nominal trajectories compared
########################################################

comparison = problem.compare(
    LQR=lqr,
    VI=vi,
    TrajOpt=trajopt,
    RRT=rrt,
    TrajOpt_tracked=tracked,
    # MPC=mpc,
)
print(comparison)
comparison.plot_nominal_trajectory()
comparison.plot_simulated_trajectory()

########################################################
### Monte Carlo comparison over random starts
########################################################

problem_x0 = problem.replace(x0=Uniform([-0.2, -0.5], [0.2, 0.5]))

comparison_x0 = problem_x0.compare(
    LQR=lqr,
    VI=vi,
    TrajOpt=trajopt,
    RRT=rrt,
    TrajOpt_tracked=tracked,
    # MPC=mpc,
    # dt=0.01,
    n_trials=N_TRIALS,
)
print(comparison_x0)
comparison_x0.plot_simulated_trajectory()
