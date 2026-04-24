"""Ad-hoc simulator matrix: set ``PAIRS`` (solver, backend), ``system``, and time grid at the top.

Run from repo root::

    python tests/benchmark/benchmark_simulator_speed_manual.py

Uses the same truth pair and table style as :file:`benchmark_simulator_speed_matrix.py`.
"""

from minilink.benchmark import benchmark_sim_speed_matrix
from minilink.blocks.testing import (
    make_dense_network,
    make_pendulum,
    make_physics_many_spheres,
)

# (solver, compile_backend) — only these pairs are run (edit freely).
PAIRS = (
    ("euler", "numpy"),
    ("euler", "jax"),
    ("rk4_fixedsteps", "numpy"),
    ("rk4_fixedsteps", "jax"),
    ("scipy", "numpy"),
    ("scipy", "jax"),
    ("scipy_stiff", "numpy"),
    ("scipy_stiff", "jax"),
    ("scipy_lsoda", "numpy"),
    ("scipy_lsoda", "jax"),
    ("scipy_max", "numpy"),
    ("scipy_max", "jax"),
    ("scipy_ultra", "numpy"),
    ("scipy_ultra", "jax"),
)

# Set up JAX for double precision.
import jax

USE_X64 = True
jax.config.update("jax_enable_x64", USE_X64)

# Pick one:
# system = make_physics_many_spheres(nx=12, ny=12)
# case_name = "ManySpheres 12x12"
# t0, tf, dt = 0.0, 2.0, 0.005

system = make_pendulum()
case_name = "Pendulum"
t0, tf, dt = 0.0, 100.0, 0.01

# system = make_dense_network(num_nodes=50, connections_per_node=5)
# case_name = "Dense network"
# t0, tf, dt = 0.0, 5.0, 0.1

benchmark_sim_speed_matrix(
    system,
    case_name=case_name,
    pairs=PAIRS,
    t0=t0,
    tf=tf,
    dt=dt,
    n_runs=1,
)
