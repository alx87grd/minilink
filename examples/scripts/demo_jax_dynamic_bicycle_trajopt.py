"""Dynamic-bicycle lane-change with JAX-backed direct collocation.

Drives the planar :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycle`
from one straight lane to a parallel lane offset by ``Y_GOAL`` while keeping a
target longitudinal speed. Uses :class:`JaxDirectCollocationTranscription` so
the optimizer receives analytic gradients/Jacobians from JAX.
"""

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import JaxQuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import JaxDynamicBicycle
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.jax_direct_collocation import (
    JaxDirectCollocationOptions,
    JaxDirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- Problem setup ---
TF = 5.0
N_STEPS = 20
U_TARGET = 5.0
Y_GOAL = 3.5
W_REAR_MAX = 80.0
DELTA_MAX = 0.6

sys = JaxDynamicBicycle()
sys.inputs["w_rear"].lower_bound[0] = 0.0
sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
sys.inputs["delta"].upper_bound[0] = DELTA_MAX

x_start = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
# Reference state used by the running and terminal cost. The lane change is
# encouraged by the cost, not enforced by an equality terminal set.
x_ref = np.array([U_TARGET * TF, Y_GOAL, 0.0, U_TARGET, 0.0, 0.0])

# State weights penalize deviation in lateral position, heading, and body slip.
Q = np.diag([0.0, 4.0, 5.0, 0.1, 1.0, 1.0])
# Penalize deviation from the cruise input (rear wheel ω that holds U_TARGET).
ubar = np.array([U_TARGET / sys.r_r, 0.0])
R = np.diag([1e-4, 50.0])
S = np.diag([0.0, 50.0, 50.0, 1.0, 10.0, 10.0])

cost = JaxQuadraticCost.from_system(
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
    transcription=JaxDirectCollocationTranscription(
        JaxDirectCollocationOptions(
            tf=TF,
            n_steps=N_STEPS,
            use_gradient=True,
            use_hessian=False,
        )
    ),
    optimizer=ScipyMinimizeOptimizer(
        options={
            "disp": True,
            "maxiter": 500,
            "ftol": 1e-2,
        }
    ),
    options=TrajectoryOptimizationOptions(compile_backend="jax"),
)

traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
if result.cost is not None:
    print(f"cost: {result.cost:.6g}")

planner.plot_solution(plot="xu")
sys.traj = traj
sys.animate(renderer="meshcat")
