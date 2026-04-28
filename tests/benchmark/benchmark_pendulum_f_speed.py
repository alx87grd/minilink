"""Flat script: JaxPendulum — time ``f`` native / numpy evaluator / jax evaluator.

Run:
    python tests/benchmark/benchmark_pendulum_f_speed.py
"""

import numpy as np

from minilink.compile.benchmark import benchmark_f_evaluators, print_f_benchmark
from minilink.simulation.scenarios.basic import JaxPendulum

sys = JaxPendulum(damping=0.5)
sys.x0[0] = 1.0

x_np = np.array([1.0, 0.0])
u_np = np.array([0.0])
n_calls = 1000

result = benchmark_f_evaluators(sys, x_np, u_np, t=0.0, n_calls=n_calls)
print_f_benchmark(result)
