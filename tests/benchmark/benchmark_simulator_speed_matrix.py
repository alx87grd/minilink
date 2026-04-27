"""Manual simulator matrix: one system, full default solver×backend sweep vs truth.

Run from repo root (package on ``PYTHONPATH``)::

    python tests/benchmark/benchmark_simulator_speed_matrix.py

Truth for error and speed is ``scipy_ultra`` + ``numpy`` (see integration timing / evaluator timing modules in ``minilink.simulation`` / ``minilink.compile``).
"""

from __future__ import annotations

import jax

from minilink.simulation.integration_timing import (
    DEFAULT_SWEEP_PAIRS,
    benchmark_sim_speed_matrix,
)
from minilink.simulation.scenarios.basic import make_pendulum
from minilink.simulation.scenarios.engine import make_physics_many_spheres
from minilink.simulation.scenarios.network import make_dense_network

USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

sys = make_pendulum()
benchmark_sim_speed_matrix(
    sys,
    case_name="Pendulum",
    pairs=DEFAULT_SWEEP_PAIRS,
    t0=0.0,
    tf=100.0,
    dt=0.01,
    n_runs=5,
)

sys = make_dense_network(num_nodes=50, connections_per_node=5)
benchmark_sim_speed_matrix(
    sys,
    case_name="Dense network (50 nodes, 5 conn/node)",
    pairs=DEFAULT_SWEEP_PAIRS,
    t0=0.0,
    tf=5.0,
    dt=0.1,
    n_runs=5,
)

sys = make_physics_many_spheres(nx=6, ny=4)
benchmark_sim_speed_matrix(
    sys,
    case_name="PhysicsManySpheres (24 bodies)",
    pairs=DEFAULT_SWEEP_PAIRS,
    t0=0.0,
    tf=1.0,
    dt=0.01,
    n_runs=5,
)
