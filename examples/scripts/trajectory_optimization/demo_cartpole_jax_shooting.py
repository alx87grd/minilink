"""Cart-pole stabilization with JAX-backed single shooting."""

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (
    ShootingOptions,
    ShootingTranscription,
)

# Demo controls.
PRINT_SOLVE_REPORT = True  # Print the Minilink TrajOpt pre/post solve report.
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT  # Print compact success/cost fallback.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.

configure_jax(enable_x64=True)

sys = JaxCartPole()
sys.inputs["u"].lower_bound[0] = -20.0
sys.inputs["u"].upper_bound[0] = 20.0

x_start = np.array([0.0, 0.2, 0.0, 0.0])
x_goal = np.zeros(sys.n)

cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 20.0, 0.1, 0.1]),
    R=np.diag([0.01]),
    S=np.diag([20.0, 80.0, 1.0, 1.0]),
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
    transcription=ShootingTranscription(
        ShootingOptions(
            tf=2.0,
            n_steps=31,
        )
    ),
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        solve_disp=PRINT_SOLVE_REPORT,
        optimizer_options={
            "disp": SCIPY_DISP,
            "maxiter": 200,
            "ftol": 1e-7,
        },
    ),
)

traj = planner.compute_solution()
result = planner.last_optimization_result

if PRINT_RESULT_SUMMARY:
    print(f"success: {result.success}")
    print(f"message: {result.message}")
    print(f"cost: {result.cost:.6g}")

planner.plot_solution(plot="xu")
planner.problem.sys.animate(traj)
