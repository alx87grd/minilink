"""Dynamic-bicycle lane-change with JAX-backed direct collocation.

Drives the planar :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycle`
from one straight lane to a parallel lane offset by ``Y_GOAL`` while keeping a
target longitudinal speed. Uses direct collocation with ``compile_backend="jax"``
so the optimizer receives analytic gradients/Jacobians from the program evaluator.
"""

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import JaxDynamicBicycle
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- Problem setup ---
PRINT_SOLVE_REPORT = True  # Print the Minilink TrajOpt pre/post solve report.
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT  # Print compact success/cost fallback.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.
TF = 5.0
N_STEPS = 50
U_TARGET = 5.0
Y_GOAL = 3.5
W_REAR_MAX = 80.0
DELTA_MAX = 0.6

sys = JaxDynamicBicycle()
sys.inputs["w_rear"].lower_bound[0] = 0.0
sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
sys.inputs["delta"].upper_bound[0] = DELTA_MAX

x_start = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0])
x_ref = np.array([0.0, 5.0, np.pi, U_TARGET, 0.0, 0.0])


Q = np.diag([1.0, 0.1, 5.0, 0.1, 0.1, 0.1])
R = np.diag([1e-4, 50.0])
S = np.diag([50.0, 50.0, 50.0, 1.0, 1.0, 10.0])

ubar = np.array([U_TARGET / sys.r_r, 0.0])


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
        optimizer_method="ipopt",
        solve_disp=PRINT_SOLVE_REPORT,
        optimizer_options={
            "disp": SCIPY_DISP,
            "maxiter": 2000,
            # "ftol": 1e-2,
        },
    ),
)

traj = planner.compute_solution()


planner.plot_solution(signals=("x", "u"))
sys.traj = traj
# sys.animate(renderer="meshcat")
sys.animate()
# sys.animate(renderer="plotly")
