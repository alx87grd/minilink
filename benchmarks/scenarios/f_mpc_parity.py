"""F MPC regression parity (the sampled loop, a hand loop on the face, dual rate).

Compare against ``benchmarks/baselines/f_mpc_parity.json`` for the landed
``planning/trajectory_optimization/mpc`` product path.
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
from minilink import BicycleDynRate
from minilink.blocks.step import UnitDelay
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.mpc import (
    ModelPredictiveController,
    PlanFollower,
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
    sys = BicycleDynRate()
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
            X=sys.state.box,
        ),
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=N_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            backend="jax",
            record_solve_time=True,
            method="scipy_slsqp",
            optimizer_options={"maxiter": MAXITER, "ftol": FTOL, "disp": False},
        ),
    )
    return planner, sys, x0


class TimedSolves:
    """The MPC face's ``solve`` wrapped to time the last solve (past the first, compiled, one) and record each status."""

    def __init__(self, pv):
        self.solve = pv.solve
        self.seconds = 0.0
        self.success = True
        pv.solve = self

    def __call__(self, params=None, z0=None):
        t0 = time.perf_counter()
        z, status = self.solve(params, z0)
        self.seconds = time.perf_counter() - t0
        self.success = self.success and bool(status["success"])
        return z, status


def plant_run(loop, traj) -> Trajectory:
    """The plant's samples of a loop's run."""
    start, end = loop.state_index["plant"]
    return Trajectory(t=traj.t, x=traj.x[start:end], u=traj.u)


def run_hybrid_zoh() -> list[MetricRecord]:
    """``(mpc % dt) @ plant``: the delayed MPC on its clock, its command held."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    mpc = ModelPredictiveController(planner, delay=True)
    solves = TimedSolves(mpc.pv)
    loop = (mpc % MPC_DT) @ sys

    t0 = time.perf_counter()
    result = loop.compute_trajectory(tf=TF_HYBRID, dt=SIM_DT)
    wall = time.perf_counter() - t0
    traj = plant_run(loop, result)
    assert_simulation_finite(traj)
    return _metrics(
        prefix="f.hybrid_zoh",
        traj=traj,
        success=solves.success,
        wall_s=wall,
        nlp_s=solves.seconds,
        notes="(mpc % dt) @ plant, the delayed MPC, short tf",
    )


def run_hand_loop() -> list[MetricRecord]:
    """A deploy-shaped host loop on the planner's face: solve, shift, command (few ticks)."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    plant = BicycleDynRate()
    plant.x0 = x0.copy()
    plant_eval = plant.compile(backend="jax")
    pv = planner.compile()
    solves = TimedSolves(pv)

    t_hist = [0.0]
    x_hist = [x0.copy()]
    x = x0.copy()
    t = 0.0
    z = pv.initial()
    u_hold = np.zeros(plant.m)
    next_mpc_t = 0.0

    t0 = time.perf_counter()
    while t < TF_HAND - 1e-12:
        if t >= next_mpc_t - 1e-12:
            z, _ = pv.solve({"x_start": x, "t_start": t}, z0=pv.shift(z, t))
            u_hold = np.asarray(pv.command(z, x, t), dtype=float)
            next_mpc_t += MPC_DT
        x = plant_eval.step(x, u_hold, t, dt=SIM_DT)
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
        success=solves.success,
        wall_s=wall,
        nlp_s=solves.seconds,
        notes="host loop on pv.solve / shift / command, short tf",
    )


def run_dual_rate() -> list[MetricRecord]:
    """``(mpc % slow) >> (PlanFollower % fast) @ plant``: the plan applied at the fast rate."""
    _require_jax()
    planner, sys, x0 = _bicycle_planner()
    mpc = ModelPredictiveController(planner)
    solves = TimedSolves(mpc.pv)
    late = UnitDelay(dim=mpc.pv.n_z).rename_ports(
        u="z"
    )  # the plan one slow period late
    late.x0 = mpc.pv.initial()
    computer = (
        (mpc % MPC_DT) >> (late % MPC_DT) >> (PlanFollower(planner) % DT_BROADCAST)
    )
    loop = computer @ sys

    t0 = time.perf_counter()
    result = loop.compute_trajectory(tf=TF_HYBRID, dt=SIM_DT)
    wall = time.perf_counter() - t0
    traj = plant_run(loop, result)
    assert_simulation_finite(traj)
    return _metrics(
        prefix="f.dual_rate",
        traj=traj,
        success=solves.success,
        wall_s=wall,
        nlp_s=solves.seconds,
        notes="(mpc % slow) >> (PlanFollower % fast) @ plant, short tf",
    )


def run_f_mpc_parity_suite() -> list[MetricRecord]:
    """Run all F MPC parity scenarios."""
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
