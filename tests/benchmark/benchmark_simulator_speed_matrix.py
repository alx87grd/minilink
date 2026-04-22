"""Manual simulator benchmark across multiple systems/solver/backend options.

Run with:
    python tests/benchmark/benchmark_simulator_speed_matrix.py
"""

from __future__ import annotations

import jax

from minilink.benchmark import benchmark_sim_speed_matrix
from minilink.blocks.testing import (
    make_dense_network,
    make_pendulum,
    make_physics_many_spheres,
)

# False -> float32 default, True -> float64 default (if JAX supports x64).
USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

_precision = "float64" if USE_X64 else "float32"

benchmark_sim_speed_matrix(
    make_pendulum,
    case_name="Pendulum",
    t0=0.0,
    tf=10.0,
    dt=0.01,
    n_runs=3,
    baseline_solver="scipy_ultra",
    baseline_backend="numpy",
    precision_note=_precision,
)

# benchmark_sim_speed_matrix(
#     lambda: make_dense_network(num_nodes=50, connections_per_node=5),
#     case_name="Dense Network (50 nodes, 5 conn/node)",
#     t0=0.0,
#     tf=5.0,
#     dt=0.1,
#     n_runs=3,
#     baseline_solver="scipy_ultra",
#     baseline_backend="numpy",
#     precision_note=_precision,
# )

# benchmark_sim_speed_matrix(
#     lambda: make_physics_many_spheres(nx=6, ny=4),
#     case_name="PhysicsManySpheres (24 bodies)",
#     t0=0.0,
#     tf=1.0,
#     dt=0.01,
#     n_runs=2,
#     baseline_solver="scipy_ultra",
#     baseline_backend="numpy",
#     precision_note=_precision,
# )
