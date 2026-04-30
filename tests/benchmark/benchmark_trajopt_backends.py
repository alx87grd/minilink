"""Manual trajectory-optimization benchmark runner.

Run from the repository root::

    python tests/benchmark/benchmark_trajopt_backends.py --case cartpole --starts both
"""

from __future__ import annotations

import argparse

from minilink.planning.trajectory_optimization.benchmark import (
    TrajectoryOptimizationBenchmarkConfig,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_variants,
    print_trajectory_optimization_benchmark,
)

parser = argparse.ArgumentParser()
parser.add_argument("--case", choices=["cartpole"], default="cartpole")
parser.add_argument("--tf", type=float, default=5.0)
parser.add_argument("--steps", type=int, default=20)
parser.add_argument("--maxiter", type=int, default=200)
parser.add_argument("--ftol", type=float, default=1e-2)
parser.add_argument("--runs", type=int, default=1)
parser.add_argument("--starts", choices=["cold", "warm", "both"], default="both")
parser.add_argument("--warm-guess-path", default="")
args = parser.parse_args()

starts = {
    "cold": ("cold",),
    "warm": ("warm",),
    "both": ("cold", "warm"),
}[args.starts]

config = TrajectoryOptimizationBenchmarkConfig(
    case=args.case,
    tf=args.tf,
    n_steps=args.steps,
    maxiter=args.maxiter,
    ftol=args.ftol,
    n_runs=args.runs,
    warm_guess_path=args.warm_guess_path or None,
)
variants = default_trajectory_optimization_variants(starts=starts, ftol=args.ftol)
result = benchmark_trajectory_optimization(config, variants)
print_trajectory_optimization_benchmark(result)
