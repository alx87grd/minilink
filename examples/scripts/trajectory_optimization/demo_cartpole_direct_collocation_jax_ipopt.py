"""Cart-pole swing-up with JAX-backed direct collocation."""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# Demo controls.
PRINT_SOLVE_REPORT = True  # Print the Minilink TrajOpt pre/post solve report.
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT  # Print compact success/cost fallback.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.

configure_jax(enable_x64=True)

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
    x_start=x_start,
    x_goal=x_goal,
    cost=cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(
            tf=4.0,
            n_steps=20,
        )
    ),
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        optimizer_method="ipopt",
        # optimizer_method="scipy_slsqp",
        # optimizer_options={"maxiter": 500, "ftol": 1e-2},
        solve_disp=PRINT_SOLVE_REPORT,
    ),
)

traj = planner.compute_solution()

planner.plot_solution(signals=("x", "u"))

planner.problem.sys.animate(traj)

# traj2 = traj.resample(n_samples=200)
# planner.problem.sys.animate(traj2)
