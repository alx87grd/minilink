"""Manual simulator matrix: one system, full default solver×backend sweep vs truth.

Run from repo root (package on ``PYTHONPATH``)::

    python benchmarks/run_simulator_speed_matrix.py

Truth for error and speed is ``scipy_ultra`` + ``numpy``.
"""

from __future__ import annotations

import jax

from benchmarks.simulation import (
    DEFAULT_SIMULATION_VARIANTS,
    benchmark_simulation_matrix,
    print_simulation_matrix_benchmark,
)
from benchmarks.systems.basic import make_pendulum
from benchmarks.systems.engine import make_physics_many_spheres
from benchmarks.systems.network import make_dense_network

USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

sys = make_pendulum()
result = benchmark_simulation_matrix(
    sys,
    case_name="Pendulum",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=100.0,
    dt=0.01,
    n_runs=5,
)
print_simulation_matrix_benchmark(result)

sys = make_dense_network(num_nodes=50, connections_per_node=5)
result = benchmark_simulation_matrix(
    sys,
    case_name="Dense network (50 nodes, 5 conn/node)",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=5.0,
    dt=0.1,
    n_runs=5,
)
print_simulation_matrix_benchmark(result)

sys = make_physics_many_spheres(nx=6, ny=4)
result = benchmark_simulation_matrix(
    sys,
    case_name="PhysicsManySpheres (24 bodies)",
    variants=DEFAULT_SIMULATION_VARIANTS,
    t0=0.0,
    tf=1.0,
    dt=0.01,
    n_runs=5,
)
print_simulation_matrix_benchmark(result)
