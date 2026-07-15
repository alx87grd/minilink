"""E4 trajopt before/after parity scenarios (TOP rebuild + MPC parametric).

Capture once before the MPC→TOP merge, then compare after. See
``docs/plans/mpc-rh-refactor/phase-E4.md``.
"""

from __future__ import annotations

import time

import numpy as np

from benchmarks.baseline import MetricRecord
from benchmarks.scenarios.common import (
    TrajoptScenarioResult,
    assert_simulation_finite,
    sample_trajectory_at_times,
)
from benchmarks.scenarios.trajopt_checks import run_showcase_cartpole_trajopt
from benchmarks.trajopt import jax_trajopt_available
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
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

# Pendulum swing-up (rebuild TOP path)
PENDULUM_TF = 2.0
PENDULUM_N_STEPS = 16
PENDULUM_MAXITER = 80
PENDULUM_FTOL = 1e-2
PENDULUM_CHECKPOINTS = (0.0, 1.0, 2.0)
PENDULUM_TAU_MAX = 20.0

# Bicycle rate-MPC (parametric TrajectoryOptimizationPlanner path)
BICYCLE_TF = 1.0
BICYCLE_N_STEPS = 8
BICYCLE_MAXITER = 60
BICYCLE_FTOL = 1e-2
BICYCLE_CHECKPOINTS = (0.0, 0.5, 1.0)
BICYCLE_U_TARGET = 4.0
BICYCLE_W_REAR_MAX = 90.0
BICYCLE_DELTA_MAX = 0.55
BICYCLE_W_REAR_DOT_MAX = 80.0
BICYCLE_DELTA_DOT_MAX = 2.0

VECTOR_ATOL = 1e-4
VECTOR_RTOL = 1e-4
SUCCESS_ATOL = 1e-12


def run_cartpole_rebuild(*, n_runs: int = 1) -> list[MetricRecord]:
    """Showcase cart-pole swing-up via TOP.solve (JAX + SciPy SLSQP)."""
    if not jax_trajopt_available():
        raise ModuleNotFoundError(
            "JAX is required for e4.cartpole_rebuild trajopt parity"
        )
    result = run_showcase_cartpole_trajopt(n_runs=n_runs)
    return _metrics_from_result(
        prefix="e4.cartpole_rebuild",
        result=result,
        notes="JaxCartPole TOP.solve, scipy_slsqp, compile_backend=jax",
    )


def run_pendulum_rebuild(*, n_runs: int = 1) -> list[MetricRecord]:
    """Small pendulum swing-up via TOP.solve (JAX + SciPy SLSQP)."""
    if not jax_trajopt_available():
        raise ModuleNotFoundError(
            "JAX is required for e4.pendulum_rebuild trajopt parity"
        )

    configure_jax(enable_x64=True)
    sys = Pendulum()
    sys.inputs["u"].lower_bound[0] = -PENDULUM_TAU_MAX
    sys.inputs["u"].upper_bound[0] = PENDULUM_TAU_MAX

    x_start = np.array([0.0, 0.0])
    x_goal = np.array([np.pi, 0.0])
    cost = QuadraticCost.from_system(
        sys,
        Q=np.diag([1.0, 0.1]),
        R=np.array([[0.01]]),
        S=np.diag([10.0, 1.0]),
        xbar=x_goal,
    )
    problem = PlanningProblem(
        sys=sys,
        tf=PENDULUM_TF,
        x_start=x_start,
        x_goal=x_goal,
        cost=cost,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=PENDULUM_N_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            optimizer_method="scipy_slsqp",
            optimizer_options={
                "maxiter": PENDULUM_MAXITER,
                "ftol": PENDULUM_FTOL,
                "disp": False,
            },
            record_solve_time=True,
            solve_disp=False,
        ),
    )

    total_times: list[float] = []
    solve_times: list[float] = []
    traj = None
    optimization_result = None

    for _ in range(n_runs):
        t0 = time.perf_counter()
        traj = planner.solve().trajectory
        total_times.append(time.perf_counter() - t0)
        optimization_result = planner.last_optimization_result
        if optimization_result is None:
            raise RuntimeError("pendulum trajopt did not store a solution")
        solve_times.append(float(optimization_result.solve_time_s))

    assert traj is not None
    assert optimization_result is not None
    assert_simulation_finite(traj)
    samples = sample_trajectory_at_times(traj, PENDULUM_CHECKPOINTS)
    result = TrajoptScenarioResult(
        checkpoint_times=PENDULUM_CHECKPOINTS,
        checkpoint_x={t: [float(v) for v in samples[t]] for t in PENDULUM_CHECKPOINTS},
        x_tf=[float(v) for v in traj.x[:, -1].reshape(-1)],
        cost=float(optimization_result.cost),
        eq_inf=0.0,
        success=bool(optimization_result.success),
        total_s=float(np.mean(total_times)),
        transcribe_s=0.0,
        solve_s=float(np.mean(solve_times)),
    )
    return _metrics_from_result(
        prefix="e4.pendulum_rebuild",
        result=result,
        notes=(
            "catalog Pendulum TOP.solve swing-up, "
            f"tf={PENDULUM_TF}, n_steps={PENDULUM_N_STEPS}, jax+slsqp"
        ),
    )


def run_bicycle_parametric(*, n_runs: int = 1) -> list[MetricRecord]:
    """Small rate-bicycle MPC tick via current TrajectoryOptimizationPlanner.solve_trajectory_from."""
    if not jax_trajopt_available():
        raise ModuleNotFoundError(
            "JAX is required for e4.bicycle_parametric trajopt parity"
        )

    configure_jax(enable_x64=True)
    sys = JaxDynamicBicycleRateInputs()
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = BICYCLE_W_REAR_MAX
    sys.state.lower_bound[7] = -BICYCLE_DELTA_MAX
    sys.state.upper_bound[7] = BICYCLE_DELTA_MAX
    sys.inputs["w_rear_dot"].lower_bound[0] = -BICYCLE_W_REAR_DOT_MAX
    sys.inputs["w_rear_dot"].upper_bound[0] = BICYCLE_W_REAR_DOT_MAX
    sys.inputs["delta_dot"].lower_bound[0] = -BICYCLE_DELTA_DOT_MAX
    sys.inputs["delta_dot"].upper_bound[0] = BICYCLE_DELTA_DOT_MAX

    r_r = float(sys.params["r_r"])
    w_rear_ref = BICYCLE_U_TARGET / r_r
    x_ref = np.array([0.0, 0.0, 0.0, BICYCLE_U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
    cost = QuadraticCost.from_system(
        sys,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=x_ref,
        ubar=np.zeros(2),
    )
    x_start = np.array(
        [
            0.0,
            1.0,
            0.0,
            BICYCLE_U_TARGET * 0.9,
            0.0,
            0.0,
            (BICYCLE_U_TARGET * 0.9) / r_r,
            0.0,
        ]
    )
    problem = PlanningProblem(sys=sys, x_start=x_start, cost=cost, tf=BICYCLE_TF)
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=BICYCLE_N_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            optimizer_method="scipy_slsqp",
            record_solve_time=True,
            optimizer_options={
                "maxiter": BICYCLE_MAXITER,
                "ftol": BICYCLE_FTOL,
                "disp": False,
            },
        ),
    )
    planner.compile_parametric_program()

    total_times: list[float] = []
    solve_times: list[float] = []
    traj = None
    cost_val = float("inf")
    success = False

    for _ in range(n_runs):
        t0 = time.perf_counter()
        plan = planner.solve_trajectory_from(x_start)
        total_times.append(time.perf_counter() - t0)
        traj = plan.trajectory
        cost_val = float(plan.metadata.cost)
        success = bool(plan.metadata.success)
        if planner.last_solve_time_s is not None:
            solve_times.append(float(planner.last_solve_time_s))
        elif plan.metadata.solve_time_s is not None:
            solve_times.append(float(plan.metadata.solve_time_s))
        else:
            solve_times.append(float(total_times[-1]))

    assert traj is not None
    assert_simulation_finite(traj)
    samples = sample_trajectory_at_times(traj, BICYCLE_CHECKPOINTS)
    result = TrajoptScenarioResult(
        checkpoint_times=BICYCLE_CHECKPOINTS,
        checkpoint_x={t: [float(v) for v in samples[t]] for t in BICYCLE_CHECKPOINTS},
        x_tf=[float(v) for v in traj.x[:, -1].reshape(-1)],
        cost=cost_val,
        eq_inf=0.0,
        success=success,
        total_s=float(np.mean(total_times)),
        transcribe_s=0.0,
        solve_s=float(np.mean(solve_times)),
    )
    return _metrics_from_result(
        prefix="e4.bicycle_parametric",
        result=result,
        notes=(
            "JaxDynamicBicycleRateInputs TrajectoryOptimizationPlanner.solve_trajectory_from, "
            f"tf={BICYCLE_TF}, n_steps={BICYCLE_N_STEPS}, jax+slsqp"
        ),
    )


def run_e4_trajopt_parity_suite() -> list[MetricRecord]:
    """Run all E4 trajopt parity scenarios and return flat metric records."""
    metrics: list[MetricRecord] = []
    metrics.extend(run_cartpole_rebuild())
    metrics.extend(run_pendulum_rebuild())
    metrics.extend(run_bicycle_parametric())
    return metrics


def _metrics_from_result(
    *,
    prefix: str,
    result: TrajoptScenarioResult,
    notes: str,
) -> list[MetricRecord]:
    metrics: list[MetricRecord] = [
        MetricRecord(
            id=f"{prefix}.success",
            gate="accuracy",
            direction="higher_better",
            value=1.0 if result.success else 0.0,
            atol=SUCCESS_ATOL,
            unit="flag",
            notes="1.0 means optimizer reported success",
        ),
        MetricRecord(
            id=f"{prefix}.cost",
            gate="speed",
            direction="lower_better",
            value=result.cost,
            unit="cost",
            notes=notes,
        ),
        MetricRecord(
            id=f"{prefix}.solve_s",
            gate="speed",
            direction="lower_better",
            value=result.solve_s,
            unit="s",
            notes="optimizer / last_solve_time_s wall time",
        ),
        MetricRecord(
            id=f"{prefix}.total_s",
            gate="speed",
            direction="lower_better",
            value=result.total_s,
            unit="s",
            notes="end-to-end wall time including compile/bind as applicable",
        ),
        MetricRecord(
            id=f"{prefix}.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=result.x_tf,
            atol=VECTOR_ATOL,
            rtol=VECTOR_RTOL,
            unit="state",
        ),
    ]
    for t in result.checkpoint_times:
        key = _checkpoint_key(t)
        metrics.append(
            MetricRecord(
                id=f"{prefix}.checkpoint_{key}",
                gate="accuracy",
                direction="vector_match",
                value=result.checkpoint_x[t],
                atol=VECTOR_ATOL,
                rtol=VECTOR_RTOL,
                unit="state",
            )
        )
    return metrics


def _checkpoint_key(t: float) -> str:
    label = f"{t:g}".replace(".", "p").replace("-", "m")
    return f"t{label}"
