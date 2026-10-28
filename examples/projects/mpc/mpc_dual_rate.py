"""Dual-rate MPC: replan at ``MPC_DT``, apply the plan at ``DT_BROADCAST``.

Contrasts with the single-rate ``(mpc % dt) @ plant`` (the plan's command held
over each slow period)::

    computer = (mpc % 0.5) >> (PlanFollower(planner) % 0.01)
    hybrid = computer @ plant

The plan solved from the state at one slow tick reaches the follower at the
next: one slow period of delay, as on a real computer.

Run from repo root::

    python examples/projects/mpc/mpc_dual_rate.py
"""

import numpy as np

from minilink import BicycleDynRate
from minilink.analysis import discretize
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.mpc import (
    ModelPredictiveController,
    PlanFollower,
    mpc_animation_overlays,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

U_TARGET = 4.0
TF_SIM = 5.0
TF_MPC = 2.0
MPC_DT = 0.5
DT_BROADCAST = 0.01
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
        tf=TF_MPC,
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
    backend="jax",
    record_solve_time=True,
    method="scipy_slsqp",
    optimizer_options={"maxiter": 50, "ftol": 1.0},
)

mpc = ModelPredictiveController(planner, lookahead=MPC_DT)  # delay compensation
computer = (mpc % MPC_DT) >> (PlanFollower(planner) % DT_BROADCAST)
hybrid = computer @ discretize(sys, DT_BROADCAST, substeps=round(DT_BROADCAST / SIM_DT))

hybrid.plot_diagram()

result = hybrid.compute_trajectory(tf=TF_SIM)
if STEP_DISP:
    start, end = hybrid.state_index["plant"]
    print(f"{round(TF_SIM / MPC_DT)} solves; final state {result.x[start:end, -1]}")
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(
        result,
        mpc,
        loop=hybrid,
        block="controller/DelayedPredictiveController",
        reference_pad=REF_X_PAD,
    )
)
