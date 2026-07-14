"""End-to-end integration regression: trajectory goldens + solve-time gates."""

from __future__ import annotations

from dataclasses import dataclass

from benchmarks.baseline import MetricRecord
from benchmarks.scenarios.simulation_checks import (
    run_double_pendulum_sim,
    run_showcase_pendulum_sim,
)
from benchmarks.scenarios.trajopt_checks import run_showcase_cartpole_trajopt
from benchmarks.trajopt import jax_trajopt_available


@dataclass(frozen=True)
class IntegrationCheckSuiteConfig:
    """Tunable workload sizes for ``run_integration_check_suite``."""

    sim_n_runs: int = 2
    trajopt_n_runs: int = 1
    checkpoint_atol: float = 0.01
    checkpoint_rtol: float = 0.01
    truth_x_tf_atol: float = 0.05
    truth_x_tf_rtol: float = 0.05
    rel_err_max_allowed: float = 1.0
    eq_inf_max_allowed: float = 1e-3


def run_integration_check_suite(
    config: IntegrationCheckSuiteConfig | None = None,
) -> list[MetricRecord]:
    """Run integration scenarios and return flat metric records."""
    cfg = config or IntegrationCheckSuiteConfig()
    metrics: list[MetricRecord] = []
    metrics.extend(_double_pendulum_metrics(cfg))
    metrics.extend(_showcase_pendulum_metrics(cfg))
    metrics.extend(_showcase_trajopt_metrics(cfg))
    return metrics


def _double_pendulum_metrics(cfg: IntegrationCheckSuiteConfig) -> list[MetricRecord]:
    result = run_double_pendulum_sim(n_runs=cfg.sim_n_runs)
    prefix = "sim.double_pendulum.rk4_numpy"
    return _simulation_metrics(
        prefix=prefix,
        result=result,
        cfg=cfg,
        notes="catalog DoublePendulum, tf=10 s, dt=0.01",
    )


def _showcase_pendulum_metrics(cfg: IntegrationCheckSuiteConfig) -> list[MetricRecord]:
    result = run_showcase_pendulum_sim(n_runs=cfg.sim_n_runs)
    prefix = "sim.showcase_pendulum.rk4_numpy"
    return _simulation_metrics(
        prefix=prefix,
        result=result,
        cfg=cfg,
        notes="showcase Pendulum x0[0]=2, tf=10 s, dt=0.01",
    )


def _showcase_trajopt_metrics(cfg: IntegrationCheckSuiteConfig) -> list[MetricRecord]:
    if not jax_trajopt_available():
        return []

    result = run_showcase_cartpole_trajopt(n_runs=cfg.trajopt_n_runs)
    prefix = "trajopt.showcase_cartpole.jax_slsqp"
    metrics: list[MetricRecord] = [
        MetricRecord(
            id=f"{prefix}.total_s",
            gate="speed",
            direction="lower_better",
            value=result.total_s,
            unit="s",
            notes="showcase cart-pole swing-up, scipy_slsqp, compile_backend=jax",
        ),
        MetricRecord(
            id=f"{prefix}.solve_s",
            gate="speed",
            direction="lower_better",
            value=result.solve_s,
            unit="s",
            notes="optimizer wall time only",
        ),
        MetricRecord(
            id=f"{prefix}.success",
            gate="accuracy",
            direction="lower_better",
            value=0.0 if result.success else 1.0,
            max_allowed=0.0,
            unit="flag",
            notes="0 means success",
        ),
        MetricRecord(
            id=f"{prefix}.eq_inf",
            gate="accuracy",
            direction="lower_better",
            value=result.eq_inf,
            max_allowed=cfg.eq_inf_max_allowed,
            unit="abs",
        ),
        MetricRecord(
            id=f"{prefix}.cost",
            gate="accuracy",
            direction="lower_better",
            value=result.cost,
            unit="cost",
            notes="optimum cost; current run must not exceed baseline value",
        ),
        MetricRecord(
            id=f"{prefix}.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=result.x_tf,
            atol=cfg.checkpoint_atol,
            rtol=cfg.checkpoint_rtol,
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
                atol=cfg.checkpoint_atol,
                rtol=cfg.checkpoint_rtol,
                unit="state",
            )
        )
    return metrics


def _simulation_metrics(
    *,
    prefix: str,
    result,
    cfg: IntegrationCheckSuiteConfig,
    notes: str,
) -> list[MetricRecord]:
    metrics = [
        MetricRecord(
            id=f"{prefix}.solve_s",
            gate="speed",
            direction="lower_better",
            value=result.solve_s,
            unit="s",
            notes=notes,
        ),
        MetricRecord(
            id=f"{prefix}.rel_err_l2",
            gate="accuracy",
            direction="lower_better",
            value=result.rel_err_l2,
            max_allowed=cfg.rel_err_max_allowed,
            unit="percent",
            notes="candidate x_tf vs live scipy_ultra truth",
        ),
        MetricRecord(
            id=f"{prefix}.truth.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=result.truth_x_tf,
            atol=cfg.truth_x_tf_atol,
            rtol=cfg.truth_x_tf_rtol,
            unit="state",
            notes="live scipy_ultra truth x_tf vs stored golden",
        ),
        MetricRecord(
            id=f"{prefix}.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=result.x_tf,
            atol=cfg.checkpoint_atol,
            rtol=cfg.checkpoint_rtol,
            unit="state",
            notes="candidate final state vs stored golden",
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
                atol=cfg.checkpoint_atol,
                rtol=cfg.checkpoint_rtol,
                unit="state",
            )
        )
    return metrics


def _checkpoint_key(t: float) -> str:
    label = f"{t:g}".replace(".", "p").replace("-", "m")
    return f"t{label}"
