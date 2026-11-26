"""Trajectory optimization: direct collocation solver presets on cart-pole swing-up.

Run from the repository root::

    python benchmarks/manual/trajopt_solver_presets.py

Not part of the regression gate; for exploration on a workstation only.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from benchmarks.common import ipopt_available
from benchmarks.trajopt import (
    TrajectoryOptimizationBenchmarkConfig,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_solver_variants,
    print_trajectory_optimization_benchmark,
)

########################################################
### Tunables
########################################################

CASE = "cartpole"
TF = 5.0  # [s]
N_STEPS = 100
MAXITER = 200
FTOL = 1e-2
N_RUNS = 1

# "cold" | "warm" | "both"
STARTS_CHOICE = "cold"

INCLUDE_TRUST_CONSTR = False
INCLUDE_IPOPT = True
WARM_GUESS_PATH = ""

########################################################
### Solver sweep and table
########################################################

starts = {
    "cold": ("cold",),
    "warm": ("warm",),
    "both": ("cold", "warm"),
}[STARTS_CHOICE]

if INCLUDE_IPOPT and not ipopt_available():
    print("note: cyipopt is not installed; skipping Ipopt trajopt variants.")

config = TrajectoryOptimizationBenchmarkConfig(
    case=CASE,
    tf=TF,
    n_steps=N_STEPS,
    maxiter=MAXITER,
    ftol=FTOL,
    n_runs=N_RUNS,
    warm_guess_path=WARM_GUESS_PATH or None,
)
variants = default_trajectory_optimization_solver_variants(
    starts=starts,
    ftol=FTOL,
    include_trust_constr=INCLUDE_TRUST_CONSTR,
    include_ipopt=INCLUDE_IPOPT,
)
result = benchmark_trajectory_optimization(config, variants)
print_trajectory_optimization_benchmark(result)
