"""Manual optimizer-backend benchmark runner.

Run from the repository root::

    python tests/benchmark/benchmark_optimizer_backends.py
    python tests/benchmark/benchmark_optimizer_backends.py --maxiter 500 --runs 3
"""

from __future__ import annotations

from minilink.optimization.benchmark import (
    STANDARD_OPTIMIZATION_CASES,
    benchmark_optimizer_backends,
    default_optimizer_variants,
    ipopt_optimizer_available,
    print_optimizer_benchmark,
)

if not ipopt_optimizer_available():
    print("note: cyipopt is not installed; running SciPy variants only.")

variants = default_optimizer_variants(
    maxiter=200,
    ftol=1e-2,
    tol=1e-2,
)
result = benchmark_optimizer_backends(
    STANDARD_OPTIMIZATION_CASES,
    variants,
    n_runs=1,
)
print_optimizer_benchmark(result)
