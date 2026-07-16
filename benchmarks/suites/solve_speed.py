"""Layer-B solve-speed gates: standalone NLP and trajopt solve wall times.

Catches Optimizer backend regressions and slow NLP/trajopt paths (including JAX
compile + SciPy SLSQP) without running full tier-2 backend sweeps.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from benchmarks.baseline import MetricRecord
from benchmarks.optimization import (
    STANDARD_OPTIMIZATION_CASES,
    OptimizerBenchmarkVariant,
    benchmark_optimizer_backends,
)
from benchmarks.scenarios.common import assert_simulation_finite
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# Standalone NLP cases (no JAX): representative unconstrained + equality-constrained.
NLP_CASE_IDS = ("rosenbrock", "circle_eq")

# Small pendulum swing-up: NumPy compile backend isolates NLP + transcription path.
PENDULUM_TF = 2.0
PENDULUM_N_STEPS = 16
PENDULUM_MAXITER = 80
PENDULUM_FTOL = 1e-2
PENDULUM_TAU_MAX = 20.0


@dataclass(frozen=True)
class SolveSpeedSuiteConfig:
    """Tunable workload for ``run_solve_speed_suite``."""

    n_runs: int = 2
    tiny: bool = False
    include_numpy_trajopt: bool = True


def run_solve_speed_suite(
    config: SolveSpeedSuiteConfig | None = None,
) -> list[MetricRecord]:
    """Run NLP and trajopt solve-speed gates; return flat metric records."""
    cfg = config or SolveSpeedSuiteConfig()
    metrics: list[MetricRecord] = []
    metrics.extend(_optimizer_nlp_metrics(cfg))
    if cfg.include_numpy_trajopt:
        metrics.extend(_pendulum_numpy_trajopt_metrics(cfg))
    return metrics


def _optimizer_nlp_metrics(cfg: SolveSpeedSuiteConfig) -> list[MetricRecord]:
    cases = tuple(c for c in STANDARD_OPTIMIZATION_CASES if c.id in NLP_CASE_IDS)
    variant = OptimizerBenchmarkVariant(
        name="scipy-SLSQP",
        method="scipy_slsqp",
        options={
            "maxiter": 200 if not cfg.tiny else 80,
            "ftol": 1e-8,
            "disp": False,
        },
    )
    n_runs = 1 if cfg.tiny else cfg.n_runs
    result = benchmark_optimizer_backends(cases, (variant,), n_runs=n_runs)
    metrics: list[MetricRecord] = []
    for row in result.rows:
        prefix = f"optimizer.{row.case_id}.scipy_slsqp"
        metrics.append(
            MetricRecord(
                id=f"{prefix}.solve_s",
                gate="speed",
                direction="lower_better",
                value=float(row.solve_s),
                unit="s",
                notes=f"{row.case_label}; Optimizer.solve wall time",
            )
        )
        metrics.append(
            MetricRecord(
                id=f"{prefix}.success",
                gate="accuracy",
                direction="lower_better",
                value=0.0 if row.success else 1.0,
                max_allowed=0.0,
                unit="flag",
                notes="0 means NLP reported success",
            )
        )
    return metrics


def _pendulum_numpy_trajopt_metrics(cfg: SolveSpeedSuiteConfig) -> list[MetricRecord]:
    n_runs = 1 if cfg.tiny else cfg.n_runs

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
            compile_backend="numpy",
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
            raise RuntimeError("pendulum numpy trajopt did not store a solution")
        solve_times.append(float(optimization_result.solve_time_s))

    assert traj is not None
    assert optimization_result is not None
    assert_simulation_finite(traj)

    prefix = "trajopt.pendulum.numpy_slsqp"
    return [
        MetricRecord(
            id=f"{prefix}.total_s",
            gate="speed",
            direction="lower_better",
            value=float(np.mean(total_times)),
            unit="s",
            notes=(
                f"catalog Pendulum swing-up, compile_backend=numpy, "
                f"n_steps={PENDULUM_N_STEPS}, scipy_slsqp"
            ),
        ),
        MetricRecord(
            id=f"{prefix}.solve_s",
            gate="speed",
            direction="lower_better",
            value=float(np.mean(solve_times)),
            unit="s",
            notes="optimizer wall time only",
        ),
        MetricRecord(
            id=f"{prefix}.success",
            gate="accuracy",
            direction="lower_better",
            value=0.0 if optimization_result.success else 1.0,
            max_allowed=0.0,
            unit="flag",
            notes="0 means optimizer reported success",
        ),
    ]
