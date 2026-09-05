"""Cart-pole swing-up with JAX-backed direct collocation."""

import numpy as np

from minilink import (
    JaxCartPole,
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.live_plot import (
    LiveTrajectoryPlotCallback,
)

PRINT_SOLVE_REPORT = True  # Minilink's pre/post solve report
LIVE_PLOT = False  # Plotly live iterate updates during the solve

sys = JaxCartPole()
sys.inputs["u"].lower_bound[0] = -10.0
sys.inputs["u"].upper_bound[0] = 10.0

x_start = np.array([-2.0, 1.0, 0.0, 0.0])
x_goal = np.array([0.0, np.pi, 0.0, 0.0])

cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 1.0, 0.0, 0.0]),
    R=np.diag([0.01]),
    S=np.zeros((sys.n, sys.n)),
    xbar=x_goal,
    ubar=np.zeros(sys.m),
)
problem = PlanningProblem(
    sys=sys,
    tf=4.0,
    x_start=x_start,
    x_goal=x_goal,
    cost=cost,
)

callback = None
if LIVE_PLOT:
    callback = LiveTrajectoryPlotCallback(
        sys,
        signals=("x", "u"),
        every=1,
        pause=0.001,
        backend="plotly",
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
    callback=callback,
)

planner.solve()
planner.plot_solution()
planner.animate_solution()

# traj2 = traj.resample(n_samples=200)
# planner.problem.sys.animate(traj2)
