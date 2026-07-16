"""Phase F MPC parity (hybrid ZOH, hand-loop, dual-rate).

Compare against ``benchmarks/baselines/f_mpc_parity.json`` after the
``planning.mpc`` → ``control.mpc`` move.
"""

from __future__ import annotations

import time

import numpy as np

from benchmarks.baseline import MetricRecord
from benchmarks.scenarios.common import (
    assert_simulation_finite,
    sample_trajectory_at_times,
)
from benchmarks.trajopt import jax_trajopt_available
from minilink.control.mpc import ModelPredictiveController, mpc_default_computer_x0
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputsUY,
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

# Short but real ModelPredictiveController workloads
U_TARGET = 4.0
HORIZON = 1.0
N_STEPS = 8
MAXITER = 40
FTOL = 1.0
MPC_DT = 0.2
SIM_DT = 0.05
TF_HYBRID = 1.0
TF_HAND = 1.0
DT_BROADCAST = 0.1
CHECKPOINTS = (0.0, 0.5, 1.0)

VECTOR_ATOL = 1e-4
VECTOR_RTOL = 1e-4
SUCCESS_ATOL = 1e-12


def _require_jax() -> None:
    if not jax_trajopt_available():
        raise ModuleNotFoundError("JAX is required for f_mpc_parity")


def _bicycle_planner():
    configure_jax(enable_x64=True)
    sys = JaxDynamicBicycleRateInputsUY()
    r_r = sys.params["r_r"]
    x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
    x0 = np.array(
        [0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0]
    )
    sys.x0 = x0.copy()
    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=sys,
            tf=HORIZON,
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
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=N_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": MAXITER, "ftol": FTOL, "disp": False},
        ),
    )
    return planner, sys, x0


def _nlp_solve_sum(planner) -> float:
    total = 0.0
    # Prefer last_trajectory_plan metadata when available; hybrid accumulates
    # via planner.last_solve_time_s only for the last tick — sum from history
    # is not stored. Use wall time as primary; expose last_solve as secondary.
    if planner.last_solve_time_s is not None:
        total = float(planner.last_solve_time_s)
    return total


def run_hybrid_zoh() -> list[MetricRecord]:
    """``mpc @ plant`` single-rate u_ff ZOH."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
    hybrid = mpc @ sys

    t0 = time.perf_counter()
    result = hybrid.compute_trajectory(
        tf=TF_HYBRID,
        x0_plant=x0,
        x0_computer=mpc_default_computer_x0(planner),
        plant_dt_inner=SIM_DT,
        compile_backend="jax",
    )
    wall = time.perf_counter() - t0
    traj = result.plant
    assert_simulation_finite(traj)
    success = True
    if planner.last_trajectory_plan is not None:
        success = bool(planner.last_trajectory_plan.metadata.success)
    return _metrics(
        prefix="f.hybrid_zoh",
        traj=traj,
        success=success,
        wall_s=wall,
        nlp_s=_nlp_solve_sum(planner),
        notes="mpc @ plant warm-start ZOH, short tf",
    )


def run_hand_loop() -> list[MetricRecord]:
    """Deploy-shaped ``compute_command`` closed loop (few ticks)."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    plant = JaxDynamicBicycleRateInputsUY()
    plant.x0 = x0.copy()
    plant_eval = plant.compile(backend="jax")
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)

    t_hist = [0.0]
    x_hist = [x0.copy()]
    x = x0.copy()
    t = 0.0
    u_hold = np.zeros(plant.m)
    next_mpc_t = 0.0
    nlp_sum = 0.0
    success = True

    t0 = time.perf_counter()
    while t < TF_HAND - 1e-12:
        if t >= next_mpc_t - 1e-12:
            cmd = mpc.compute_command(x, t=t)
            u_hold = cmd.u_ff.copy()
            success = success and bool(cmd.success)
            if planner.last_solve_time_s is not None:
                nlp_sum += float(planner.last_solve_time_s)
            next_mpc_t += MPC_DT
        x = plant_eval.rk4_step(x, u_hold, t, SIM_DT)
        t += SIM_DT
        t_hist.append(t)
        x_hist.append(x.copy())
    wall = time.perf_counter() - t0

    traj = Trajectory(
        t=np.asarray(t_hist),
        x=np.asarray(x_hist).T,
        u=np.zeros((plant.m, len(t_hist))),
    )
    assert_simulation_finite(traj)
    return _metrics(
        prefix="f.hand_loop",
        traj=traj,
        success=success,
        wall_s=wall,
        nlp_s=nlp_sum,
        notes="compute_command hand loop, short tf",
    )


def run_dual_rate() -> list[MetricRecord]:
    """``dual_rate_computer @ plant`` with u_nom broadcast."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
    computer = mpc.dual_rate_computer(dt_broadcast=DT_BROADCAST)
    hybrid = computer @ sys

    t0 = time.perf_counter()
    result = hybrid.compute_trajectory(
        tf=TF_HYBRID,
        x0_plant=x0,
        x0_computer=mpc_default_computer_x0(planner),
        plant_dt_inner=SIM_DT,
        compile_backend="jax",
    )
    wall = time.perf_counter() - t0
    traj = result.plant
    assert_simulation_finite(traj)
    success = True
    if planner.last_trajectory_plan is not None:
        success = bool(planner.last_trajectory_plan.metadata.success)
    return _metrics(
        prefix="f.dual_rate",
        traj=traj,
        success=success,
        wall_s=wall,
        nlp_s=_nlp_solve_sum(planner),
        notes="dual_rate_computer @ plant, short tf",
    )


def run_f_mpc_parity_suite() -> list[MetricRecord]:
    """Run all Phase F MPC parity scenarios."""
    metrics: list[MetricRecord] = []
    metrics.extend(run_hybrid_zoh())
    metrics.extend(run_hand_loop())
    metrics.extend(run_dual_rate())
    return metrics


def _metrics(
    *,
    prefix: str,
    traj: Trajectory,
    success: bool,
    wall_s: float,
    nlp_s: float,
    notes: str,
) -> list[MetricRecord]:
    samples = sample_trajectory_at_times(traj, CHECKPOINTS)
    metrics: list[MetricRecord] = [
        MetricRecord(
            id=f"{prefix}.success",
            gate="accuracy",
            direction="higher_better",
            value=1.0 if success else 0.0,
            atol=SUCCESS_ATOL,
            unit="flag",
            notes="1.0 means last/all NLP ticks reported success",
        ),
        MetricRecord(
            id=f"{prefix}.wall_s",
            gate="speed",
            direction="lower_better",
            value=float(wall_s),
            unit="s",
            notes=notes,
        ),
        MetricRecord(
            id=f"{prefix}.nlp_s",
            gate="speed",
            direction="lower_better",
            value=float(nlp_s),
            unit="s",
            notes="recorded NLP solve time (sum or last, see scenario)",
        ),
        MetricRecord(
            id=f"{prefix}.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=[float(v) for v in traj.x[:, -1].reshape(-1)],
            atol=VECTOR_ATOL,
            rtol=VECTOR_RTOL,
            unit="state",
        ),
    ]
    for t in CHECKPOINTS:
        key = f"{t:g}".replace(".", "p").replace("-", "m")
        metrics.append(
            MetricRecord(
                id=f"{prefix}.checkpoint_t{key}",
                gate="accuracy",
                direction="vector_match",
                value=[float(v) for v in samples[t]],
                atol=VECTOR_ATOL,
                rtol=VECTOR_RTOL,
                unit="state",
            )
        )
    return metrics
