"""Manual optimizer-backend benchmark runner.

Run from the repository root::

    python tests/benchmark/benchmark_optimizer_backends.py
    python tests/benchmark/benchmark_optimizer_backends.py --maxiter 500 --runs 3
"""

from __future__ import annotations

import argparse

from minilink.optimization.benchmark import (
    STANDARD_OPTIMIZATION_CASES,
    benchmark_optimizer_backends,
    default_optimizer_variants,
    ipopt_optimizer_available,
    print_optimizer_benchmark,
)

parser = argparse.ArgumentParser()
parser.add_argument("--maxiter", type=int, default=200)
parser.add_argument("--ftol", type=float, default=1e-8)
parser.add_argument("--tol", type=float, default=1e-8)
parser.add_argument("--runs", type=int, default=1)
args = parser.parse_args()

if not ipopt_optimizer_available():
    print("note: cyipopt is not installed; running SciPy variants only.")

variants = default_optimizer_variants(
    maxiter=args.maxiter,
    ftol=args.ftol,
    tol=args.tol,
)
result = benchmark_optimizer_backends(
    STANDARD_OPTIMIZATION_CASES,
    variants,
    n_runs=args.runs,
)
print_optimizer_benchmark(result)
