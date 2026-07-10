"""Flat script: step diagram ``step`` speed (same table as leaf benchmark).

Run:
    python benchmarks/run_step_diagram_speed.py
"""

import numpy as np

from benchmarks.step_evaluators import benchmark_step_evaluators, print_step_benchmark
from benchmarks.systems.step import make_step_chain

diagram = make_step_chain(depth=50)
x_np = np.ones(diagram.n)
u_np = np.array([1.0])
n_calls = 10_000

result = benchmark_step_evaluators(diagram, x_np, u_np, k=0, n_calls=n_calls)
print_step_benchmark(result)
