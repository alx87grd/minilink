"""One pendulum swing-up, four planners, one judge: LQR, value iteration, trajectory optimization and RRT."""

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
    compare,
)

TF = 5.0  # [s]

# The plant: theta = 0 hanging; leaving the state box is a failure

plant = Pendulum()
plant.state.lower = np.array([-2.0 * np.pi, -8.0])
plant.state.upper = np.array([2.0 * np.pi, 8.0])

# The problem: J = ∫ (x − x̄)ᵀ Q (x − x̄) + uᵀ R u dt, from rest hanging to a ball
# around the upright, inside the state and input boxes

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

# LQR: u = −K (x − x̄), the gain of the model linearized at the upright

lqr = LQRPlanner(problem).solve(evaluate=True)

# Value iteration: J(x) = min_u [ g(x, u) dt + J(x + f(x, u) dt) ], swept to convergence

vi = DynamicProgrammingPlanner(
    problem.replace(horizon=None),
    x_grid=(101, 101),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    max_iterations=2000,
    out_of_bound_cost=500.0,
).solve()

# Trajectory optimization: min over x(t), u(t) of J, the dynamics as collocation constraints

trajopt = TrajectoryOptimizationPlanner(
    problem, n_steps=25, transcription="direct_collocation"
).solve(evaluate=True)

# RRT: a tree grown by short constant-torque rollouts until a branch enters the goal

torques = [np.array([tau]) for tau in (-5.0, -2.0, 0.0, 2.0, 5.0)]
rrt = RRTPlanner(
    problem,
    extender=KinodynamicExtender(controls=torques, horizon=0.3, n_steps=6),
    seed=0,
    max_iterations=20000,
).solve(evaluate=True)

# One judge: each answer run on the same problem, scored the same way

comparison = compare(problem, LQR=lqr, VI=vi, TrajOpt=trajopt, RRT=rrt, dt=0.01)
print(comparison)

comparison.plot_trajectory()
vi.animate()
