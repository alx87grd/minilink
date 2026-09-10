"""Cart-pole swing-up with JAX-backed direct collocation."""

import numpy as np

from minilink import (
    CartPole,
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
)

PRINT_SOLVE_REPORT = True  # Minilink's pre/post solve report
LIVE_PLOT = False  # redraw the iterate trajectory during the solve

sys = CartPole()
sys.inputs["u"].lower_bound[0] = -10.0
sys.inputs["u"].upper_bound[0] = 10.0

x_start = np.array([-2.0, 1.0, 0.0, 0.0])
x_goal = np.array([0.0, np.pi, 0.0, 0.0])

cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 1.0, 0.0, 0.0]),
    R=np.diag([0.01]),
    xbar=x_goal,
)
problem = PlanningProblem(
    sys=sys,
    tf=4.0,
    x_start=x_start,
    x_goal=x_goal,
    cost=cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=40,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="ipopt",
    # optimizer_method="scipy_slsqp",
    # optimizer_options={"maxiter": 500, "ftol": 1e-2},
    verbose=PRINT_SOLVE_REPORT,
    live_plot=LIVE_PLOT,
)

planner.solve()
planner.plot_solution()
planner.animate_solution()

# traj2 = traj.resample(n_samples=200)
# planner.problem.sys.animate(traj2)
