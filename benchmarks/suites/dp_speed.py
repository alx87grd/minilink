"""Regression-gate value-iteration suite: backward-sweep wall times per backend.

Catches regressions in the :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
backward step — a lost table cache, a de-vectorized NumPy backup, or a JAX path
that silently stops being jitted — and checks that the fast backend still agrees
with the NumPy reference on the cost-to-go and the greedy policy.

Agreement is measured relative to ``J``: the two backends match to machine
precision on most nodes, and their absolute gap is carried by the nodes whose
value has saturated toward the out-of-bound penalty, so an absolute ceiling
would only track how much of the grid is infeasible.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from benchmarks.baseline import MetricRecord
from benchmarks.dynamic_programming import benchmark_backend

# Pendulum swing-up on a grid small enough to gate, large enough to be vectorized-bound.
X_GRID_SHAPE = (101, 101)
U_GRID_SHAPE = (11,)
N_STEPS = 40
DT = 0.05

# Tiny workload for the CI smoke run: dispatch-bound, so it gates accuracy, not speedup.
TINY_X_GRID_SHAPE = (21, 21)
TINY_U_GRID_SHAPE = (5,)
TINY_N_STEPS = 5

# Ceilings for the fast backend against the NumPy reference.
MAX_REL_DJ = 1.0e-2
MAX_POLICY_DISAGREEMENT = 0.02


@dataclass(frozen=True)
class DpSpeedSuiteConfig:
    """Tunable workload for ``run_dp_speed_suite``."""

    n_runs: int = 2
    tiny: bool = False
    include_jax: bool = True


def run_dp_speed_suite(config: DpSpeedSuiteConfig | None = None) -> list[MetricRecord]:
    """Run value-iteration speed and cross-backend gates; return flat metric records."""
    cfg = config or DpSpeedSuiteConfig()

    x_grid_shape = TINY_X_GRID_SHAPE if cfg.tiny else X_GRID_SHAPE
    u_grid_shape = TINY_U_GRID_SHAPE if cfg.tiny else U_GRID_SHAPE
    n_steps = TINY_N_STEPS if cfg.tiny else N_STEPS

    # Two runs minimum: the first JAX solve pays compilation, the best one must not
    runs = max(2, cfg.n_runs)

    def timed(backend):
        return benchmark_backend(
            backend,
            x_grid_shape,
            u_grid_shape,
            n_steps=n_steps,
            dt=DT,
            runs=runs,
        )

    # The vectorized NumPy lookup table is the reference, for speed and for value
    reference = timed("numpy")
    note = f"pendulum swing-up, {reference.grid}, {n_steps} sweeps"

    metrics = [
        MetricRecord(
            id="dp.pendulum.grid.build_s",
            gate="speed",
            direction="lower_better",
            value=float(reference.build_s),
            unit="s",
            notes=f"{note}; shared grid and transition-table build",
        ),
        _solve_metric(reference, note),
    ]

    if not (cfg.include_jax and _jax_available()):
        return metrics

    jax_row = timed("jax")
    metrics.append(_solve_metric(jax_row, note))
    metrics.extend(_agreement_metrics(jax_row, reference))

    # A tiny grid is dispatch-bound, so its speedup says nothing about the engine
    if not cfg.tiny:
        metrics.append(_speedup_metric(jax_row, reference))

    return metrics


def _solve_metric(row, note: str) -> MetricRecord:
    """Steady-state backward-sweep wall time of one backend."""
    return MetricRecord(
        id=f"dp.pendulum.{row.backend}.solve_s",
        gate="speed",
        direction="lower_better",
        value=float(row.best_solve_s),
        unit="s",
        notes=f"{note}; steady-state backward sweeps",
    )


def _speedup_metric(row, reference) -> MetricRecord:
    """How much faster the backend's backward sweeps are than the NumPy reference."""
    speedup = reference.best_solve_s / row.best_solve_s

    return MetricRecord(
        id=f"dp.pendulum.{row.backend}.speedup_vs_numpy",
        gate="speed",
        direction="higher_better",
        value=float(speedup),
        unit="ratio",
        notes="steady-state solve speedup; collapses if the path stops being jitted",
    )


def _agreement_metrics(row, reference) -> list[MetricRecord]:
    """Cost-to-go and policy agreement of one backend against the NumPy reference."""
    J = reference.result.J
    scale = np.maximum(np.abs(J), 1.0)

    # Relative gap: scale-free, so it does not track how much of the grid is infeasible
    max_rel_dJ = np.max(np.abs(row.result.J - J) / scale)

    # Share of nodes whose greedy action differs
    disagreement = np.mean(row.result.pi != reference.result.pi)

    return [
        MetricRecord(
            id=f"dp.pendulum.{row.backend}.max_rel_dJ_vs_numpy",
            gate="accuracy",
            direction="lower_better",
            value=float(max_rel_dJ),
            max_allowed=MAX_REL_DJ,
            unit="ratio",
            notes="max |dJ| / max(|J|, 1) vs the numpy backend",
        ),
        MetricRecord(
            id=f"dp.pendulum.{row.backend}.policy_disagreement",
            gate="accuracy",
            direction="lower_better",
            value=float(disagreement),
            max_allowed=MAX_POLICY_DISAGREEMENT,
            unit="fraction",
            notes="share of nodes whose greedy action differs from the numpy backend",
        ),
    ]


def _jax_available() -> bool:
    """JAX is an optional extra; the NumPy gates run without it."""
    try:
        import jax  # noqa: F401
    except ModuleNotFoundError:
        return False

    return True
