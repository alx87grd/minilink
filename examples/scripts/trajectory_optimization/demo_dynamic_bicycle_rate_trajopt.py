import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# --- Problem setup ---
PRINT_SOLVE_REPORT = True  # Print the Minilink TrajOpt pre/post solve report.
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT  # Print compact success/cost fallback.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.
TF = 3.0
N_STEPS = 30
U_0 = 10.0
U_TARGET = U_0 * 0.0
Y_GOAL = 2.5
HEADING_TARGET = 0.0

sys = JaxDynamicBicycleRateInputs()

x_start = np.array([0.0, 0.0, 0.0, U_0, 0.0, 0.0, U_0 / sys.params["r_r"], 0.0])
x_ref = np.array(
    [
        -0.0,
        Y_GOAL,
        HEADING_TARGET,
        U_TARGET,
        0.0,
        0.0,
        U_TARGET / sys.params["r_r"],
        0.0,
    ]
)


# Q = np.diag([0.0, 1.0, 50.0, 0.1, 0.1, 0.01, 1.0, 500.0])
# R = np.diag([1.0, 25.0])
# S = np.diag([0.0, 50.0, 500.0, 1.0, 1.0, 0.1, 1.0, 500.0])

Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0])
R = np.diag([1.0, 10.0])
S = np.diag([0.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0])


ubar = np.array([0.0, 0.0])


cost = QuadraticCost.from_system(
    sys,
    Q=Q,
    R=R,
    S=S,
    xbar=x_ref,
    ubar=ubar,
)
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    cost=cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(
            tf=TF,
            n_steps=N_STEPS,
        )
    ),
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        # optimizer_method="ipopt",
        solve_disp=PRINT_SOLVE_REPORT,
        optimizer_options={
            "disp": SCIPY_DISP,
            "maxiter": 500,
            "ftol": 1e-1,
        },
    ),
)

traj = planner.compute_solution()


planner.plot_solution(signals=("x", "u"))
sys.traj = traj
# sys.animate(renderer="meshcat")
sys.animate()
# sys.animate(renderer="plotly")
