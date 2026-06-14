"""Flat script: JaxPendulum — time ``f`` native / numpy evaluator / jax evaluator.

Run:
    python benchmarks/run_pendulum_f_speed.py
"""

import numpy as np

from benchmarks.f_evaluators import benchmark_f_evaluators, print_f_benchmark
from benchmarks.systems.basic import JaxPendulum

sys = JaxPendulum(damping=0.5)
sys.x0[0] = 1.0

x_np = np.array([1.0, 0.0])
u_np = np.array([0.0])
n_calls = 1000

result = benchmark_f_evaluators(sys, x_np, u_np, t=0.0, n_calls=n_calls)
print_f_benchmark(result)
