"""Manual optimizer-backend benchmark runner.

Run from the repository root::

    python benchmarks/run_optimizer_backends.py
    python benchmarks/run_optimizer_backends.py --maxiter 500 --runs 3
"""

from __future__ import annotations

from benchmarks.common import ipopt_available
from benchmarks.optimization import (
    STANDARD_OPTIMIZATION_CASES,
    benchmark_optimizer_backends,
    default_optimizer_variants,
    print_optimizer_benchmark,
)

if not ipopt_available():
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
