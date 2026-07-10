"""Flat script: leaf ``step`` — native / NumPy evaluator / JAX evaluator.

Run:
    python benchmarks/run_step_speed.py
"""

import numpy as np

from benchmarks.step_evaluators import benchmark_step_evaluators, print_step_benchmark
from benchmarks.systems.step import LogisticMap

sys = LogisticMap()
x_np = np.array([0.4])
u_np = np.array([])
n_calls = 100_000

result = benchmark_step_evaluators(sys, x_np, u_np, k=0, n_calls=n_calls)
print_step_benchmark(result)
