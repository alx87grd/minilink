"""Three standard ``Simulator`` benchmark cases: long pendulum, short many-spheres, dense diagram."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Sequence

from minilink.benchmark.simulation_speed import (
    TRUTH_BACKEND,
    TRUTH_SOLVER,
    BenchmarkResult,
    benchmark_sim_backend,
)

# Re-export for runners.
__all__ = [
    "StandardCase",
    "STANDARD_SIM_CASES",
    "print_standard_sim_suite",
    "run_standard_sim_suite",
]


@dataclass(frozen=True)
class StandardCase:
    """One named scenario: build a system, time grid, and a short label for tables."""

    id: str
    label: str
    t0: float
    tf: float
    dt: float
    build: Callable[[], Any]


def _pendulum_long() -> Any:
    from minilink.blocks.testing import make_pendulum

    return make_pendulum()


def _spheres_short() -> Any:
    from minilink.blocks.testing import make_physics_many_spheres

    return make_physics_many_spheres(nx=6, ny=4)


def _diagram_dense() -> Any:
    from minilink.blocks.testing import make_dense_network

    return make_dense_network(num_nodes=50, connections_per_node=5)


# Long horizon / SciPy+mesh; many-body short run; large compiled network.
STANDARD_SIM_CASES: tuple[StandardCase, ...] = (
    StandardCase(
        "pendulum_long",
        "Pendulum (long)",
        0.0,
        100.0,
        0.01,
        _pendulum_long,
    ),
    StandardCase(
        "spheres_short",
        "ManySpheres (short)",
        0.0,
        1.0,
        0.01,
        _spheres_short,
    ),
    StandardCase(
        "diagram_dense",
        "Dense network",
        0.0,
        5.0,
        0.1,
        _diagram_dense,
    ),
)


def run_standard_sim_suite(
    candidate_solver: str,
    candidate_backend: str,
    *,
    truth_solver: str = TRUTH_SOLVER,
    truth_backend: str = TRUTH_BACKEND,
    n_runs: int = 1,
    cases: Sequence[StandardCase] = STANDARD_SIM_CASES,
) -> list[tuple[str, BenchmarkResult]]:
    """Run :func:`benchmark_sim_backend` on each standard case (fresh system per case)."""
    out: list[tuple[str, BenchmarkResult]] = []
    for case in cases:
        system = case.build()
        r = benchmark_sim_backend(
            system,
            candidate_solver=candidate_solver,
            candidate_backend=candidate_backend,
            truth_solver=truth_solver,
            truth_backend=truth_backend,
            t0=case.t0,
            tf=case.tf,
            dt=case.dt,
            n_runs=n_runs,
        )
        out.append((case.label, r))
    return out


def print_standard_sim_suite(
    results: list[tuple[str, BenchmarkResult]],
) -> None:
    """Print a compact table: case, mean_s, speedup, rel_err %."""
    print("\n=== Standard simulator suite ===")
    print(
        f"truth=({TRUTH_SOLVER}, {TRUTH_BACKEND})  "
        "speedup = truth_mean_time / candidate_mean_time  err = L2% vs truth final x"
    )
    print(
        f"{'case':<22} {'mean_s':>10} {'speedup':>9} {'err_%':>10}  "
        f"{'cand':<18}"
    )
    print("-" * 80)
    for label, r in results:
        cand = f"{r.candidate_solver}+{r.candidate_backend}"
        print(
            f"{label:<22} {r.mean_time:>10.5f} {r.speedup_vs_truth:>9.2f} "
            f"{r.rel_err_l2:>10.4f}  {cand:<18}"
        )
