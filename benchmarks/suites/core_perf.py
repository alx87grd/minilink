"""Core performance and accuracy regression suite."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from benchmarks.baseline import MetricRecord
from benchmarks.f_evaluators import (
    FEvaluatorBenchmarkVariant,
    benchmark_f_evaluators,
)
from benchmarks.simulation import (
    STANDARD_SIMULATION_CASES,
    TRUTH_SIMULATION_VARIANT,
    SimulationBenchmarkVariant,
    benchmark_simulation_backend,
)
from benchmarks.systems.basic import JaxPendulum, NumpyPendulum
from benchmarks.systems.network import make_dense_network
from minilink.blocks.routing import Gain
from minilink.blocks.sources import Step
from minilink.core.diagram import DiagramSystem


@dataclass(frozen=True)
class CorePerfSuiteConfig:
    """Tunable workload sizes for ``run_core_perf_suite``."""

    pendulum_n_calls: int = 2000
    diagram_n_calls: int = 500
    sim_n_runs: int = 2
    diagram_dense_nodes: int = 50
    diagram_dense_connections: int = 5
    static_n_steps: int = 200


DIAGRAM_DENSE_X = None  # set per build
DIAGRAM_DENSE_U = np.array([])


def run_core_perf_suite(
    config: CorePerfSuiteConfig | None = None,
) -> list[MetricRecord]:
    """Run the core-perf regression suite and return flat metric records."""
    cfg = config or CorePerfSuiteConfig()
    metrics: list[MetricRecord] = []
    metrics.extend(_f_evaluator_metrics(cfg))
    metrics.extend(_diagram_dx_metrics(cfg))
    metrics.extend(_simulation_metrics(cfg))
    metrics.extend(_static_facade_metrics(cfg))
    return metrics


def _pendulum_for_f_benchmark():
    try:
        import jax.numpy as jnp  # noqa: F401
    except ImportError:
        return NumpyPendulum(damping=0.5)
    return JaxPendulum(damping=0.5)


def _f_evaluator_metrics(cfg: CorePerfSuiteConfig) -> list[MetricRecord]:
    metrics: list[MetricRecord] = []
    pendulum = _pendulum_for_f_benchmark()
    pendulum.x0[0] = 1.0
    x_np = np.array([1.0, 0.0])
    u_np = np.array([0.0])
    result = benchmark_f_evaluators(
        pendulum,
        x_np,
        u_np,
        n_calls=cfg.pendulum_n_calls,
    )
    for row in result.rows:
        prefix = f"pendulum_f.{row.variant.name}"
        if row.message:
            continue
        if row.loop_s is not None:
            metrics.append(
                MetricRecord(
                    id=f"{prefix}.loop_s",
                    gate="speed",
                    direction="lower_better",
                    value=float(row.loop_s),
                    unit="s",
                )
            )
        if row.compile_s is not None:
            metrics.append(
                MetricRecord(
                    id=f"{prefix}.compile_s",
                    gate="speed",
                    direction="lower_better",
                    value=float(row.compile_s),
                    unit="s",
                )
            )
        if row.speedup_vs_native is not None and row.variant.backend is not None:
            metrics.append(
                MetricRecord(
                    id=f"{prefix}.speedup_vs_native",
                    gate="speed",
                    direction="higher_better",
                    value=float(row.speedup_vs_native),
                    unit="ratio",
                )
            )

    diagram = make_dense_network(
        num_nodes=cfg.diagram_dense_nodes,
        connections_per_node=cfg.diagram_dense_connections,
    )
    global DIAGRAM_DENSE_X
    DIAGRAM_DENSE_X = np.ones(diagram.n)
    result = benchmark_f_evaluators(
        diagram,
        DIAGRAM_DENSE_X,
        DIAGRAM_DENSE_U,
        n_calls=cfg.diagram_n_calls,
        variants=(
            FEvaluatorBenchmarkVariant("native", None),
            FEvaluatorBenchmarkVariant("numpy", "numpy"),
        ),
    )
    for row in result.rows:
        if row.variant.name != "numpy" or row.message:
            continue
        if row.speedup_vs_native is not None:
            metrics.append(
                MetricRecord(
                    id="diagram_dense_f.numpy.speedup_vs_native",
                    gate="speed",
                    direction="higher_better",
                    value=float(row.speedup_vs_native),
                    unit="ratio",
                )
            )
    return metrics


def _diagram_dx_metrics(cfg: CorePerfSuiteConfig) -> list[MetricRecord]:
    diagram = make_dense_network(
        num_nodes=cfg.diagram_dense_nodes,
        connections_per_node=cfg.diagram_dense_connections,
    )
    x = np.ones(diagram.n)
    u = np.array([])
    dx_ref = np.asarray(diagram.f(x, u), dtype=float)
    evaluator = diagram.compile(backend="numpy", verbose=False)
    dx_compiled = np.asarray(evaluator.f(x, u, 0.0), dtype=float)
    residual = float(np.max(np.abs(dx_ref - dx_compiled)))
    return [
        MetricRecord(
            id="diagram_dense_f.numpy.dx_residual",
            gate="accuracy",
            direction="lower_better",
            value=residual,
            max_allowed=1e-9,
            unit="abs",
            notes="max |diagram.f - evaluator.f|",
        ),
        MetricRecord(
            id="diagram_dense_f.numpy.dx_ref",
            gate="accuracy",
            direction="vector_match",
            value=[float(v) for v in dx_ref.reshape(-1)],
            atol=1e-9,
            rtol=1e-9,
            unit="state",
            notes="reference dx from native diagram.f",
        ),
    ]


def _simulation_metrics(cfg: CorePerfSuiteConfig) -> list[MetricRecord]:
    metrics: list[MetricRecord] = []
    cases = {case.id: case for case in STANDARD_SIMULATION_CASES}

    pendulum_case = cases["pendulum_long"]
    pendulum = pendulum_case.build()
    try:
        pendulum_result = benchmark_simulation_backend(
            pendulum,
            candidate=SimulationBenchmarkVariant("rk4_fixedsteps", "jax"),
            truth=TRUTH_SIMULATION_VARIANT,
            t0=pendulum_case.t0,
            tf=pendulum_case.tf,
            dt=pendulum_case.dt,
            n_runs=cfg.sim_n_runs,
        )
    except Exception:
        pendulum_result = None

    if pendulum_result is not None:
        metrics.extend(
            _sim_result_metrics(
                prefix="sim.pendulum_long",
                candidate_tag="rk4_jax",
                result=pendulum_result,
            )
        )

    diagram_case = cases["diagram_dense"]
    diagram = diagram_case.build()
    diagram_result = benchmark_simulation_backend(
        diagram,
        candidate=SimulationBenchmarkVariant("rk4_fixedsteps", "numpy"),
        truth=TRUTH_SIMULATION_VARIANT,
        t0=diagram_case.t0,
        tf=diagram_case.tf,
        dt=diagram_case.dt,
        n_runs=cfg.sim_n_runs,
    )
    metrics.extend(
        _sim_result_metrics(
            prefix="sim.diagram_dense",
            candidate_tag="rk4_numpy",
            result=diagram_result,
        )
    )
    return metrics


def _sim_result_metrics(
    *, prefix: str, candidate_tag: str, result
) -> list[MetricRecord]:
    truth_x = [float(v) for v in result.truth_x_final.reshape(-1)]
    return [
        MetricRecord(
            id=f"{prefix}.{candidate_tag}.speedup_vs_truth",
            gate="speed",
            direction="higher_better",
            value=float(result.speedup_vs_truth),
            unit="ratio",
        ),
        MetricRecord(
            id=f"{prefix}.{candidate_tag}.rel_err_l2",
            gate="accuracy",
            direction="lower_better",
            value=float(result.rel_err_l2),
            max_allowed=1.0,
            unit="percent",
            notes="candidate x_tf vs live scipy_ultra truth",
        ),
        MetricRecord(
            id=f"{prefix}.truth.x_tf",
            gate="accuracy",
            direction="vector_match",
            value=truth_x,
            atol=0.05,
            rtol=0.05,
            unit="state",
            notes="live scipy_ultra truth x_tf vs stored golden",
        ),
    ]


def _static_facade_metrics(cfg: CorePerfSuiteConfig) -> list[MetricRecord]:
    metrics: list[MetricRecord] = []

    gain = Gain(K=2.0, dim=1)
    t0 = time.perf_counter()
    gain.compute_trajectory(
        t0=0, tf=1, n_steps=cfg.static_n_steps, show=False, verbose=False
    )
    metrics.append(
        MetricRecord(
            id="static.gain.compute_trajectory_s",
            gate="speed",
            direction="lower_better",
            value=float(time.perf_counter() - t0),
            unit="s",
            notes="metadata only",
        )
    )

    step = Step(
        initial_value=np.array([0.0]),
        final_value=np.array([1.0]),
        step_time=0.5,
    )
    diagram = DiagramSystem()
    diagram.add_subsystem(step, "src")
    diagram.add_subsystem(Gain(K=1.0, dim=1), "gain")
    diagram.connect("src", "y", "gain", "u")
    t0 = time.perf_counter()
    diagram.compute_trajectory(
        t0=0, tf=1, n_steps=cfg.static_n_steps, show=False, verbose=False
    )
    metrics.append(
        MetricRecord(
            id="static.diagram.compute_trajectory_s",
            gate="speed",
            direction="lower_better",
            value=float(time.perf_counter() - t0),
            unit="s",
            notes="metadata only",
        )
    )
    return metrics
