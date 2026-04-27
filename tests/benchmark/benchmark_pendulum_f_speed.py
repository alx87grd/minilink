"""Flat script: JaxPendulum — time ``f`` native / numpy evaluator / jax evaluator.

Run:
    python tests/benchmark/benchmark_pendulum_f_speed.py
"""

import numpy as np

from minilink.simulation.scenarios.basic import JaxPendulum
from minilink.compile.evaluator_timing import benchmark_f_speeds

sys = JaxPendulum(damping=0.5)
sys.x0[0] = 1.0

x_np = np.array([1.0, 0.0])
u_np = np.array([0.0])
n_calls = 1000

benchmark_f_speeds(sys, x_np, u_np, t=0.0, n_calls=n_calls)
