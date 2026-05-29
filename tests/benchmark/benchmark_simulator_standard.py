"""Run the three standard simulator cases for one candidate (solver, backend).

Cases: long pendulum, short many-spheres, dense network diagram. Each uses a
fresh system. Truth is ``scipy_ultra`` + ``numpy``.

    python tests/benchmark/benchmark_simulator_standard.py
"""

from __future__ import annotations

import jax

from minilink.simulation.benchmark import (
    SimulationBenchmarkVariant,
    benchmark_standard_simulation_suite,
    print_standard_simulation_benchmark,
)

USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

CANDIDATE = SimulationBenchmarkVariant("rk4_fixedsteps", "jax")
N_RUNS = 1

if __name__ == "__main__":
    results = benchmark_standard_simulation_suite(
        CANDIDATE,
        n_runs=N_RUNS,
    )
    print_standard_simulation_benchmark(results)
