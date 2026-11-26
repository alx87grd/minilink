"""Simulator speed and accuracy: solver × backend matrix vs scipy_ultra truth.

Run from the repository root::

    python benchmarks/manual/simulator_speed_matrix.py

Not part of the regression gate; for exploration on a workstation only.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import jax

from benchmarks.simulation import (
    DEFAULT_SIMULATION_VARIANTS,
    benchmark_simulation_matrix,
    print_simulation_matrix_benchmark,
)
from benchmarks.systems.basic import make_pendulum
from benchmarks.systems.engine import make_physics_many_spheres
from benchmarks.systems.network import make_dense_network

########################################################
### Tunables
########################################################

USE_X64 = False
N_RUNS = 5
ACCURACY_THRESHOLD_PCT = 1.0

########################################################
### Pendulum
########################################################

jax.config.update("jax_enable_x64", USE_X64)

sys = make_pendulum()
result = benchmark_simulation_matrix(
    sys,
    case_name="Pendulum",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=100.0,
    dt=0.01,
    n_runs=N_RUNS,
    accuracy_threshold_pct=ACCURACY_THRESHOLD_PCT,
)
print_simulation_matrix_benchmark(result)

########################################################
### Dense network diagram
########################################################

sys = make_dense_network(num_nodes=50, connections_per_node=5)
result = benchmark_simulation_matrix(
    sys,
    case_name="Dense network (50 nodes, 5 conn/node)",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=5.0,
    dt=0.1,
    n_runs=N_RUNS,
    accuracy_threshold_pct=ACCURACY_THRESHOLD_PCT,
)
print_simulation_matrix_benchmark(result)

########################################################
### Many coupled spheres
########################################################

sys = make_physics_many_spheres(nx=6, ny=4)
result = benchmark_simulation_matrix(
    sys,
    case_name="PhysicsManySpheres (24 bodies)",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=1.0,
    dt=0.01,
    n_runs=N_RUNS,
    accuracy_threshold_pct=ACCURACY_THRESHOLD_PCT,
)
print_simulation_matrix_benchmark(result)
