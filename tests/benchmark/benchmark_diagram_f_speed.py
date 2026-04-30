"""Flat script: diagram ``f`` speed (same table as pendulum benchmark).

Run:
    python tests/benchmark/benchmark_diagram_f_speed.py
"""

import numpy as np

from minilink.compile.benchmark import benchmark_f_evaluators, print_f_benchmark
from minilink.simulation.scenarios.network import make_dense_network

diag = make_dense_network(num_nodes=50, connections_per_node=5)
x_np = np.ones(diag.n)
u_np = np.array([])

# diag = build_deep_network(depth=50)
# x_np = np.ones(diag.n)
# u_np = np.array([])

result = benchmark_f_evaluators(diag, x_np, u_np, t=0.0, n_calls=1000)
print_f_benchmark(result)
