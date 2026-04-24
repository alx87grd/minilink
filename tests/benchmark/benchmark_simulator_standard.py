"""Run the three standard simulator cases for one candidate (solver, backend).

Cases: long pendulum, short many-spheres, dense network diagram. Each uses a
fresh system. Truth is ``scipy_ultra`` + ``numpy`` (``minilink.benchmark``).

    python tests/benchmark/benchmark_simulator_standard.py
"""

from __future__ import annotations

import jax

from minilink.benchmark import print_standard_sim_suite, run_standard_sim_suite

USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

CANDIDATE_SOLVER = "rk4_fixedsteps"
CANDIDATE_BACKEND = "jax"
N_RUNS = 1

if __name__ == "__main__":
    results = run_standard_sim_suite(
        CANDIDATE_SOLVER,
        CANDIDATE_BACKEND,
        n_runs=N_RUNS,
    )
    print_standard_sim_suite(results)
