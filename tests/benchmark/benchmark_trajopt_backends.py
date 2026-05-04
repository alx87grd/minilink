"""Manual trajectory-optimization benchmark runner.

Edit the settings below, then run from the repository root::

    python tests/benchmark/benchmark_trajopt_backends.py

In IPython/Jupyter, ``%run`` this file after changing the variables, or assign
the variables in a cell and paste/run the body (imports + ``starts`` through
``print_...``).
"""

from __future__ import annotations

from minilink.planning.trajectory_optimization.benchmark import (
    TrajectoryOptimizationBenchmarkConfig,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_variants,
    print_trajectory_optimization_benchmark,
)

case = "cartpole"
tf = 5.0
steps = 20
maxiter = 200
ftol = 1e-2
runs = 1
starts_choice = "both"  # "cold" | "warm" | "both"
warm_guess_path = ""

starts = {
    "cold": ("cold",),
    "warm": ("warm",),
    "both": ("cold", "warm"),
}[starts_choice]

config = TrajectoryOptimizationBenchmarkConfig(
    case=case,
    tf=tf,
    n_steps=steps,
    maxiter=maxiter,
    ftol=ftol,
    n_runs=runs,
    warm_guess_path=warm_guess_path or None,
)
variants = default_trajectory_optimization_variants(starts=starts, ftol=ftol)
result = benchmark_trajectory_optimization(config, variants)
print_trajectory_optimization_benchmark(result)
