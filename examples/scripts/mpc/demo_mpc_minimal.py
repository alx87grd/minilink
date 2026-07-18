"""Minimal hybrid MPC: ``ModelPredictiveController`` then ``mpc @ plant``.

Warm-start via ``warm_start=True`` (packed ``z`` on ``Computer.x``).

Run from repo root::

    python examples/scripts/mpc/demo_mpc_minimal.py
"""

import numpy as np

from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
)
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

U_TARGET = 4.0
TF_SIM = 5.0
MPC_DT = 0.2
SIM_DT = 0.01
STEP_DISP = True
REF_X_PAD = 20.0

sys = BicycleDynRate()
r_r = sys.params["r_r"]
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0])
sys.x0 = x0.copy()

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        sys=sys,
        tf=2.0,
        x_start=x0,
        cost=QuadraticCost.from_system(
            sys,
            Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
            R=np.diag([1.0, 25.0]),
            S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
            xbar=x_ref,
            ubar=np.zeros(2),
        ),
    ),
    n_steps=20,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 50, "ftol": 1.0},
)

mpc = ModelPredictiveController(
    planner, dt_mpc=MPC_DT, warm_start=True, step_disp=STEP_DISP
)
hybrid = mpc @ sys

hybrid.plot_diagram()

result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0,
    plant_dt_inner=SIM_DT,
    compile_backend="jax",
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(result, planner, reference_pad=REF_X_PAD)
)
