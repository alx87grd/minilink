"""Simulation benchmark utilities: timing helpers, baseline ``Simulator`` compare, solver×backend matrix."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, NamedTuple, Sequence, TypeVar

_T = TypeVar("_T")

import numpy as np

from minilink.simulation import Simulator

# ---------------------------------------------------------------------------
# Timing helpers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TimingStats:
    mean: float
    std: float
    min: float
    max: float


def run_timed(func: Callable[[], _T], n_runs: int = 3) -> tuple[list[float], list[_T]]:
    """Run a callable multiple times and return raw durations and outputs.

    Returns
    -------
    durations : list of float
        Wall times in seconds.
    outputs : list
        Return value from ``func`` for each run.
    """
    durations = []
    outputs = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        out = func()
        durations.append(time.perf_counter() - t0)
        outputs.append(out)
    return durations, outputs


def summarize_durations(durations) -> TimingStats:
    arr = np.asarray(durations, dtype=float)
    return TimingStats(
        mean=float(np.mean(arr)),
        std=float(np.std(arr)),
        min=float(np.min(arr)),
        max=float(np.max(arr)),
    )


def relative_l2_error(candidate, reference) -> float:
    """Return percentage relative L2 error of candidate against reference."""
    candidate_arr = np.asarray(candidate, dtype=float)
    reference_arr = np.asarray(reference, dtype=float)
    diff_norm = np.linalg.norm(candidate_arr - reference_arr)
    ref_norm = np.linalg.norm(reference_arr)
    if ref_norm > 0.0:
        return 100.0 * diff_norm / ref_norm
    if diff_norm == 0.0:
        return 0.0
    return float("inf")


# ---------------------------------------------------------------------------
# Single candidate vs baseline Simulator run
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BenchmarkResult:
    candidate_solver: str
    candidate_backend: str
    baseline_solver: str
    baseline_backend: str
    mean_time: float
    std_time: float
    baseline_mean_time: float
    speedup_vs_baseline: float
    rel_err_l2: float
    nfev: int
    njev: int
    n_t: int


def benchmark_sim_backend(
    system_factory: Callable[[], Any],
    *,
    candidate_solver: str,
    candidate_backend: str,
    baseline_solver: str = "scipy_max",
    baseline_backend: str = "numpy",
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
) -> BenchmarkResult:
    """Benchmark one ``(solver, compile_backend)`` pair against a baseline configuration."""

    def _run(solver, backend):
        sim = Simulator(
            system_factory(),
            t0=t0,
            tf=tf,
            dt=dt,
            solver=solver,
            verbose=False,
            compile_backend=backend,
        )
        traj = sim.solve()
        x_end = np.asarray(traj.x[:, -1], dtype=float)
        return x_end, sim.last_debug

    candidate_durations, candidate_outputs = run_timed(
        lambda: _run(candidate_solver, candidate_backend), n_runs=n_runs
    )
    baseline_durations, baseline_outputs = run_timed(
        lambda: _run(baseline_solver, baseline_backend), n_runs=n_runs
    )

    candidate_stats = summarize_durations(candidate_durations)
    baseline_stats = summarize_durations(baseline_durations)

    candidate_x_end = np.mean(
        np.stack([out[0] for out in candidate_outputs], axis=0), axis=0
    )
    baseline_x_end = np.mean(
        np.stack([out[0] for out in baseline_outputs], axis=0), axis=0
    )
    rel_err = relative_l2_error(candidate_x_end, baseline_x_end)

    debug = candidate_outputs[-1][1]
    speedup = (
        baseline_stats.mean / candidate_stats.mean
        if candidate_stats.mean > 0
        else float("inf")
    )
    return BenchmarkResult(
        candidate_solver=candidate_solver,
        candidate_backend=candidate_backend,
        baseline_solver=baseline_solver,
        baseline_backend=baseline_backend,
        mean_time=candidate_stats.mean,
        std_time=candidate_stats.std,
        baseline_mean_time=baseline_stats.mean,
        speedup_vs_baseline=speedup,
        rel_err_l2=rel_err,
        nfev=int(debug["nfev"]),
        njev=int(debug["njev"]),
        n_t=int(debug["n_t"]),
    )


# ---------------------------------------------------------------------------
# Solver × compile_backend matrix
# ---------------------------------------------------------------------------


DEFAULT_SOLVERS: tuple[str, ...] = (
    "euler",
    "rk4_fixedsteps",
    "scipy",
    "scipy_stiff",
    "scipy_max",
    "scipy_ultra",
)
DEFAULT_BACKENDS: tuple[str, ...] = ("numpy", "jax")


class SimMatrixRow(NamedTuple):
    """One (solver, backend) cell after comparison to the baseline."""

    solver: str
    backend: str
    mean_time: float
    std_time: float
    nfev: int
    njev: int
    n_t: int
    rel_err_l2: float
    speedup_vs_baseline: float


class SimulatorMatrixResult(NamedTuple):
    """Full matrix for one ``system_factory`` and time grid."""

    case_name: str
    t0: float
    tf: float
    dt: float
    n_runs: int
    baseline_solver: str
    baseline_backend: str
    precision_note: str
    rows: tuple[SimMatrixRow, ...]


def _backend_label(backend: str) -> str:
    if backend != "jax":
        return backend
    try:
        import jax

        return f"jax({jax.default_backend()})"
    except ImportError:
        return "jax"


def _to_row(r: BenchmarkResult) -> SimMatrixRow:
    return SimMatrixRow(
        solver=r.candidate_solver,
        backend=r.candidate_backend,
        mean_time=r.mean_time,
        std_time=r.std_time,
        nfev=r.nfev,
        njev=r.njev,
        n_t=r.n_t,
        rel_err_l2=r.rel_err_l2,
        speedup_vs_baseline=r.speedup_vs_baseline,
    )


def print_sim_matrix_table(result: SimulatorMatrixResult) -> None:
    """Print the standard simulator matrix table (same layout as the benchmark script)."""
    print(f"\n=== {result.case_name} ===")
    print(
        f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} "
        f"baseline=({result.baseline_solver}, {result.baseline_backend}) "
        f"precision={result.precision_note}"
    )
    print(
        "solver_mode      backend          mean [s]    std [s]   nfev   njev    n_t"
        "     rel_err_l2 speed_vs_base"
    )
    print(
        "--------------- --------------- ---------- ---------- ------ ------ ------"
        " -------------- -------------"
    )
    for row in result.rows:
        bl = _backend_label(row.backend)
        print(
            f"{row.solver:<15} {bl:<15} {row.mean_time:>10.6f}"
            f" {row.std_time:>10.6f} {row.nfev:>6d} {row.njev:>6d}"
            f" {row.n_t:>6d} {row.rel_err_l2:>14.4f}%"
            f" {row.speedup_vs_baseline:>13.2f}x"
        )


def benchmark_sim_speed_matrix(
    system_factory: Callable[[], Any],
    *,
    case_name: str,
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
    baseline_solver: str = "scipy_ultra",
    baseline_backend: str = "numpy",
    solvers: Sequence[str] | None = None,
    backends: Sequence[str] | None = None,
    precision_note: str = "float32",
    print_table: bool = True,
) -> SimulatorMatrixResult:
    """Benchmark every ``(solver, backend)`` pair against a fixed baseline Simulator run.

    Parameters
    ----------
    system_factory
        Zero-argument callable returning a fresh system (same contract as
        :func:`benchmark_sim_backend`).
    case_name
        Printed title for the matrix block.
    solvers, backends
        Explicit lists to sweep. Defaults: :data:`DEFAULT_SOLVERS`,
        :data:`DEFAULT_BACKENDS`.
    precision_note
        Short label printed in the table header (e.g. ``\"float32\"`` / ``\"float64\"``).
    print_table
        If ``True``, print the matrix via :func:`print_sim_matrix_table`.

    Returns
    -------
    SimulatorMatrixResult
        One row per ``(solver, backend)`` in nested iteration order.
    """
    solvers_t = tuple(solvers) if solvers is not None else DEFAULT_SOLVERS
    backends_t = tuple(backends) if backends is not None else DEFAULT_BACKENDS

    rows: list[SimMatrixRow] = []
    for solver in solvers_t:
        for backend in backends_t:
            r = benchmark_sim_backend(
                system_factory,
                candidate_solver=solver,
                candidate_backend=backend,
                baseline_solver=baseline_solver,
                baseline_backend=baseline_backend,
                t0=t0,
                tf=tf,
                dt=dt,
                n_runs=n_runs,
            )
            rows.append(_to_row(r))

    out = SimulatorMatrixResult(
        case_name=case_name,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        baseline_solver=baseline_solver,
        baseline_backend=baseline_backend,
        precision_note=precision_note,
        rows=tuple(rows),
    )
    if print_table:
        print_sim_matrix_table(out)
    return out
