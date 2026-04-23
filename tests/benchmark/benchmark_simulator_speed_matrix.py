"""Manual simulator benchmark across multiple systems/solver/backend options.

Truth for ``rel_err_l2`` and the speed column uses ``scipy_ultra`` + ``numpy`` below.
Table rows: green if error vs truth is below ``accuracy_threshold_pct``, red otherwise;
the fastest *mean* time among green rows is bold bright green.

The **anchor** row in the header is ``rk4_fixedsteps`` + ``jax`` (same grid as the matrix);
it is *not* reused as truth, so you get ``anchor_err_vs_truth`` and ``anchor_mean_t`` for
comparison. For long grids (e.g. ``tf=100``, ``dt=0.01``), adaptive SciPy ``solve_ivp``
with dense ``t_eval`` rarely beats RK4+JAX on wall time under the same error cap; see
``tests/benchmark/tune_scipy_vs_rk4.py`` for a 10-round reproducible search.

Run with:
    python tests/benchmark/benchmark_simulator_speed_matrix.py
"""

from __future__ import annotations

import jax

from minilink.benchmark import benchmark_sim_speed_matrix
from minilink.blocks.testing import (
    make_dense_network,
    make_pendulum,
    make_physics_many_spheres,
)

# False -> float32 default, True -> float64 default (if JAX supports x64).
USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)

_precision = "float64" if USE_X64 else "float32"

benchmark_sim_speed_matrix(
    make_pendulum,
    case_name="Pendulum",
    t0=0.0,
    tf=100.0,
    dt=0.01,
    n_runs=1,
    baseline_solver="scipy_ultra",
    baseline_backend="numpy",
    accuracy_threshold_pct=1.0,
    anchor_solver="rk4_fixedsteps",
    anchor_backend="jax",
    precision_note=_precision,
    progress=True,
)

# benchmark_sim_speed_matrix(
#     lambda: make_dense_network(num_nodes=50, connections_per_node=5),
#     case_name="Dense Network (50 nodes, 5 conn/node)",
#     t0=0.0,
#     tf=5.0,
#     dt=0.1,
#     n_runs=3,
#     baseline_solver="scipy_ultra",
#     baseline_backend="numpy",
#     accuracy_threshold_pct=1.0,
#     anchor_solver="rk4_fixedsteps",
#     anchor_backend="jax",
#     precision_note=_precision,
# )

# benchmark_sim_speed_matrix(
#     lambda: make_physics_many_spheres(nx=6, ny=4),
#     case_name="PhysicsManySpheres (24 bodies)",
#     t0=0.0,
#     tf=1.0,
#     dt=0.01,
#     n_runs=2,
#     baseline_solver="scipy_ultra",
#     baseline_backend="numpy",
#     accuracy_threshold_pct=1.0,
#     anchor_solver="rk4_fixedsteps",
#     anchor_backend="jax",
#     precision_note=_precision,
# )
