"""Ad-hoc simulator matrix: set variants, ``system``, and time grid at the top.

Run from repo root::

    python tests/benchmark/benchmark_simulator_speed_manual.py

Uses the same truth pair and table style as :file:`benchmark_simulator_speed_matrix.py`.
"""

import jax

from minilink.simulation.benchmark import (
    SimulationBenchmarkVariant,
    benchmark_simulation_matrix,
    print_simulation_matrix_benchmark,
)
from minilink.simulation.scenarios.basic import make_pendulum

# Only these variants are run (edit freely).
VARIANTS = (
    SimulationBenchmarkVariant("euler", "numpy"),
    SimulationBenchmarkVariant("euler", "jax"),
    SimulationBenchmarkVariant("rk4_fixedsteps", "numpy"),
    SimulationBenchmarkVariant("rk4_fixedsteps", "jax"),
    SimulationBenchmarkVariant("scipy", "numpy"),
    SimulationBenchmarkVariant("scipy", "jax"),
    SimulationBenchmarkVariant("scipy_stiff", "numpy"),
    SimulationBenchmarkVariant("scipy_stiff", "jax"),
    SimulationBenchmarkVariant("scipy_lsoda", "numpy"),
    SimulationBenchmarkVariant("scipy_lsoda", "jax"),
    SimulationBenchmarkVariant("scipy_max", "numpy"),
    SimulationBenchmarkVariant("scipy_max", "jax"),
    SimulationBenchmarkVariant("scipy_ultra", "numpy"),
    SimulationBenchmarkVariant("scipy_ultra", "jax"),
)

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

result = benchmark_simulation_matrix(
    system,
    case_name=case_name,
    variants=VARIANTS,
    t0=t0,
    tf=tf,
    dt=dt,
    n_runs=1,
)
print_simulation_matrix_benchmark(result)
