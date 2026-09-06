"""Minimal hybrid MPC: ``ModelPredictiveController`` then ``mpc @ plant``."""

import numpy as np

from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
from minilink.dynamics.catalog.vehicles.jax_vehicles import BicycleDynRate

U_TARGET = 4.0
TF_SIM = 5.0
MPC_DT = 0.02
SIM_DT = 0.01
VERBOSE = True
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
        ),
    ),
    n_steps=5,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 10, "ftol": 1.0},
)

mpc = ModelPredictiveController(
    planner, dt_mpc=MPC_DT, warm_start=True, verbose=VERBOSE
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
