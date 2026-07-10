"""Minimal hybrid MPC: ``mpc % schedule`` then ``computer @ plant``.

Warm-start via :func:`mpc_stateful_controller` (packed ``z`` on ``Computer.x``).

Run from repo root::

    python examples/scripts/hybrid/demo_mpc_hybrid_minimal.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputsUY,
)
from minilink.planning.mpc import (
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    mpc_animation_overlays,
    mpc_stateful_controller,
)
from minilink.planning.mpc.warm_start import mpc_default_computer_x0
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
)

configure_jax(enable_x64=True)

U_TARGET = 4.0
TF_SIM = 5.0
MPC_DT = 0.2
SIM_DT = 0.002
STEP_DISP = True
REF_X_PAD = 20.0

sys = JaxDynamicBicycleRateInputsUY()
r_r = sys.params["r_r"]
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0])
sys.x0 = x0.copy()

planner = MPCPlanner(
    PlanningProblem(
        sys=sys,
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
    transcription=MPCDirectCollocationTranscription(
        DirectCollocationOptions(tf=2.0, n_steps=20)
    ),
    options=MPCOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 50, "ftol": 1.0},
    ),
)

mpc = mpc_stateful_controller(planner, dt_mpc=MPC_DT, step_disp=STEP_DISP)
computer = mpc % MPC_DT
hybrid = computer @ sys

hybrid.plot_diagram()

result = hybrid.compute_trajectory(
    tf=TF_SIM,
    x0_plant=x0,
    x0_computer=mpc_default_computer_x0(planner),
    plant_dt_inner=SIM_DT,
)
hybrid.plot_trajectory()
hybrid.animate(
    overlays=mpc_animation_overlays(result, planner, reference_pad=REF_X_PAD)
)
