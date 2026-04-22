"""Benchmark helpers: ``f`` evaluator timings and ``Simulator`` speed matrices.

Modules:

- ``f_speed`` — native vs NumPy vs JAX ``f`` loop timing and optional table print.
- ``simulation_speed`` — timing helpers, :func:`benchmark_sim_backend`, and
  :func:`benchmark_sim_speed_matrix`.

Example runners (flat scripts, not pytest) live under ``tests/benchmark/``.
"""

from minilink.benchmark.f_speed import (
    FSpeedTimings,
    benchmark_f_speeds,
    print_f_speed_table,
)
from minilink.benchmark.simulation_speed import (
    DEFAULT_BACKENDS,
    DEFAULT_SOLVERS,
    BenchmarkResult,
    SimMatrixRow,
    SimulatorMatrixResult,
    TimingStats,
    benchmark_sim_backend,
    benchmark_sim_speed_matrix,
    print_sim_matrix_table,
    relative_l2_error,
    run_timed,
    summarize_durations,
)

__all__ = [
    "BenchmarkResult",
    "benchmark_sim_backend",
    "DEFAULT_BACKENDS",
    "DEFAULT_SOLVERS",
    "SimMatrixRow",
    "SimulatorMatrixResult",
    "TimingStats",
    "benchmark_sim_speed_matrix",
    "print_sim_matrix_table",
    "relative_l2_error",
    "run_timed",
    "summarize_durations",
    "FSpeedTimings",
    "benchmark_f_speeds",
    "print_f_speed_table",
]
