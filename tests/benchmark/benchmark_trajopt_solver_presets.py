"""Manual trajectory-optimization solver-preset benchmark runner.

Edit the settings below, then run from the repository root::

    python tests/benchmark/benchmark_trajopt_solver_presets.py

This script intentionally keeps the sweep small. By default it benchmarks only
direct collocation and a cold start, comparing a NumPy/SLSQP baseline against
JAX analytic-derivative solver presets, including Ipopt when ``cyipopt`` is
installed.
"""

from __future__ import annotations

from minilink.planning.trajectory_optimization.benchmark import (
    TrajectoryOptimizationBenchmarkConfig,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_solver_variants,
    ipopt_trajopt_available,
    print_trajectory_optimization_benchmark,
)

# Problem size.
case = "cartpole"
tf = 5.0
steps = 100

# Solver budget.
maxiter = 200
ftol = 1e-2
runs = 1

# Keep this benchmark tractable by default. Use "both" only when you want to
# compare cold-start setup against warm-start solve behavior.
starts_choice = "cold"  # "cold" | "warm" | "both"

# Solver presets to include. Ipopt is added only when cyipopt is installed.
include_trust_constr = False
include_ipopt = True

# Optional path for saving/reusing the warm-start trajectory.
warm_guess_path = ""

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
)
variants = default_trajectory_optimization_solver_variants(
    starts=starts,
    ftol=ftol,
    include_trust_constr=include_trust_constr,
    include_ipopt=include_ipopt,
)
result = benchmark_trajectory_optimization(config, variants)
print_trajectory_optimization_benchmark(result)
