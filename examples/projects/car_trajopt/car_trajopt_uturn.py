import numpy as np

from minilink import DynamicBicycle
from minilink.core.costs import QuadraticCost
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

# --- Problem setup ---
PRINT_SOLVE_REPORT = True  # Print the Minilink TrajOpt pre/post solve report.
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT  # Print compact success/cost fallback.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.
TF = 2.5
N_STEPS = 25
U_TARGET = 5.0
Y_GOAL = 15.0
W_REAR_MAX = 30.0
DELTA_MAX = 0.8

sys = DynamicBicycle()
sys.inputs["w_rear"].lower_bound[0] = W_REAR_MAX
sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
sys.inputs["delta"].upper_bound[0] = DELTA_MAX

x0 = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0])
x_ref = np.array([-0.0, Y_GOAL, np.pi, U_TARGET, 0.0, 0.0])


Q = np.diag([0.0, 1.0, 50.0, 0.1, 0.1, 0.01])
R = np.diag([1.0, 500.0])
S = np.diag([0.0, 50.0, 500.0, 1.0, 1.0, 0.1])

u_bar = np.array([U_TARGET / sys.params["r_r"], 0.0])


cost = QuadraticCost.from_system(
    sys,
    Q=Q,
    R=R,
    S=S,
    x_bar=x_ref,
    u_bar=u_bar,
)
problem = PlanningProblem(
    sys=sys,
    horizon=TF,
    x0=x0,
    cost=cost,
    U=sys.input_box,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=N_STEPS,
    transcription="direct_collocation",
    backend="jax",
    # method="ipopt",
    options={
        "disp": SCIPY_DISP,
        "maxiter": 500,
        "ftol": 1e-1,
    },
)

traj = planner.solve(verbose=PRINT_SOLVE_REPORT).trajectory
planner.plot_trajectory(signals=("x", "u"))
sys.traj = traj
# sys.animate(renderer="meshcat")
sys.animate()
# sys.animate(renderer="plotly")
