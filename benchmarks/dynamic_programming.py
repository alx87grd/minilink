"""How much faster is the lookup-table / JAX value iteration than the loop?

Times the three :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
backends on a pendulum swing-up:

- ``loop``  — per-node Python double loop (pyro's reference engine).
- ``numpy`` — vectorized over a precomputed successor/cost lookup table.
- ``jax``   — the same backup jitted as one ``lax.while_loop`` on device.

Reports grid-build time, cold solve (JAX includes one-time compilation), and
steady-state solve (best of several runs), plus a cross-backend value check.
"""

import time
from dataclasses import dataclass

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

UPRIGHT = np.array([-np.pi, 0.0])


@dataclass
class DPBenchmarkRow:
    """One timed value-iteration run."""

    backend: str
    grid: str
    build_s: float
    cold_solve_s: float
    best_solve_s: float
    iterations: int
    result: object  # DynamicProgrammingResult, for the cross-backend value check


def pendulum_problem():
    """Pendulum swing-up problem (pyro SinglePendulum defaults)."""
    sys = Pendulum()
    sys.state.lower_bound = np.array([-2.0 * np.pi, -2.0 * np.pi])
    sys.state.upper_bound = np.array([2.0 * np.pi, 2.0 * np.pi])
    sys.inputs["u"].lower_bound = np.array([-5.0])
    sys.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(sys, xbar=UPRIGHT, R=np.eye(1))
    return PlanningProblem(sys, x_goal=UPRIGHT, cost=cost)


def benchmark_backend(
    backend: str,
    x_grid_shape,
    u_grid_shape,
    *,
    n_steps: int = 300,
    dt: float = 0.05,
    alpha: float = 1.0,
    runs: int = 3,
) -> DPBenchmarkRow:
    """Time grid build and a fixed number of backward sweeps for one backend.

    A fixed sweep count makes the per-iteration throughput directly comparable
    across backends and bounds the runtime. One planner is reused across runs so
    the JAX runner compiles once (``cold`` includes compilation, ``best`` not).
    """
    problem = pendulum_problem()

    t0 = time.perf_counter()
    grid = StateSpaceGrid(
        problem,
        x_grid_shape=x_grid_shape,
        u_grid_shape=u_grid_shape,
        dt=dt,
        precompute=(backend != "loop"),
    )
    build_s = time.perf_counter() - t0

    options = DynamicProgrammingOptions(backend=backend, alpha=alpha)
    planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)

    cold_solve_s = None
    best_solve_s = np.inf
    result = None
    for _ in range(runs):
        t0 = time.perf_counter()
        result = planner.solve_steps(n_steps)
        elapsed = time.perf_counter() - t0
        if cold_solve_s is None:
            cold_solve_s = elapsed
        best_solve_s = min(best_solve_s, elapsed)

    return DPBenchmarkRow(
        backend=backend,
        grid=f"{tuple(x_grid_shape)}x{tuple(u_grid_shape)}",
        build_s=build_s,
        cold_solve_s=cold_solve_s,
        best_solve_s=best_solve_s,
        iterations=result.iterations,
        result=result,
    )


def print_dp_benchmark(rows, *, reference: str = "loop") -> None:
    """Print a timing table with solve speedups relative to ``reference``."""
    ref = next((r for r in rows if r.backend == reference), rows[0])

    print(
        f"\n{'backend':8} {'grid':18} {'build':>8} {'cold':>9} {'best':>9} "
        f"{'iters':>6} {'solve x':>9}"
    )
    print("-" * 73)
    for r in rows:
        # Speedup compares the steady-state solve (the backward-step engine);
        # the grid/table build is a separate, shared one-time cost.
        speedup = (
            ref.best_solve_s / r.best_solve_s if r.best_solve_s > 0 else float("nan")
        )
        print(
            f"{r.backend:8} {r.grid:18} {r.build_s:8.2f} {r.cold_solve_s:9.3f} "
            f"{r.best_solve_s:9.3f} {r.iterations:6d} {speedup:8.1f}x"
        )
    print("-" * 73)
    print(
        "build = grid/table (shared, one-time NumPy precompute); "
        "cold = first solve (jax incl. compile); best = steady-state solve [s]"
    )


def check_agreement(rows, *, atol: float = 1e-3) -> None:
    """Report the largest cost-to-go gap between backends on feasible nodes."""
    base = rows[0]
    feasible = base.result.J < 1e5
    print(f"\ncross-backend value check (vs {base.backend}, feasible nodes):")
    for r in rows[1:]:
        gap = float(np.max(np.abs(r.result.J[feasible] - base.result.J[feasible])))
        status = "ok" if gap < atol else "DIFF"
        print(f"  {r.backend:8} max|dJ| = {gap:.2e}  [{status}]")
