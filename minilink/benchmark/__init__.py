"""Benchmark helpers: ``f`` evaluator timings and ``Simulator`` speed matrices.

Modules:

- ``f_speed`` — native vs NumPy vs JAX ``f`` loop timing and optional table print.
- ``simulation_speed`` — timing helpers, :func:`benchmark_sim_backend`,
  :func:`benchmark_sim_speed_matrix` (sweep a ``pairs`` list).
- ``standard_sim_cases`` — three fixed scenarios and :func:`run_standard_sim_suite`.

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
    DEFAULT_SWEEP_PAIRS,
    TRUTH_BACKEND,
    TRUTH_SOLVER,
    BenchmarkResult,
    MatrixResult,
    MatrixRow,
    TimingStats,
    benchmark_sim_backend,
    benchmark_sim_speed_matrix,
    print_sim_matrix_table,
    relative_l2_error,
    run_timed,
    summarize_durations,
)
from minilink.benchmark.standard_sim_cases import (
    STANDARD_SIM_CASES,
    StandardCase,
    print_standard_sim_suite,
    run_standard_sim_suite,
)

__all__ = [
    "BenchmarkResult",
    "MatrixResult",
    "MatrixRow",
    "STANDARD_SIM_CASES",
    "StandardCase",
    "TRUTH_BACKEND",
    "TRUTH_SOLVER",
    "benchmark_sim_backend",
    "benchmark_sim_speed_matrix",
    "DEFAULT_BACKENDS",
    "DEFAULT_SOLVERS",
    "DEFAULT_SWEEP_PAIRS",
    "print_sim_matrix_table",
    "print_standard_sim_suite",
    "relative_l2_error",
    "run_timed",
    "run_standard_sim_suite",
    "summarize_durations",
    "TimingStats",
    "FSpeedTimings",
    "benchmark_f_speeds",
    "print_f_speed_table",
]
