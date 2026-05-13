"""Manual trajectory-optimization benchmark runner.

Sweeps transcriptions and backends (NumPy and JAX when installed). Ipopt rows are
included when ``cyipopt`` is available unless ``include_ipopt`` is false.

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
    ipopt_trajopt_available,
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

# Appends Ipopt rows when cyipopt is installed (`pip install minilink[ipopt]`).
include_ipopt = True

# One block per distinct transcription / compile_backend / grid (stderr-style details).
print_mathematical_program_details = True

starts = {
    "cold": ("cold",),
    "warm": ("warm",),
    "both": ("cold", "warm"),
}[starts_choice]

if include_ipopt and not ipopt_trajopt_available():
    print("note: cyipopt is not installed; skipping Ipopt trajopt variants.")

config = TrajectoryOptimizationBenchmarkConfig(
    case=case,
    tf=tf,
    n_steps=steps,
    maxiter=maxiter,
    ftol=ftol,
    n_runs=runs,
    warm_guess_path=warm_guess_path or None,
    print_mathematical_program_details=print_mathematical_program_details,
)
variants = default_trajectory_optimization_variants(
    starts=starts,
    ftol=ftol,
    include_ipopt=include_ipopt,
)
result = benchmark_trajectory_optimization(config, variants)
print_trajectory_optimization_benchmark(result)
