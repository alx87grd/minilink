"""Simulation benchmark utilities: timing, one pair vs truth, matrix over ``pairs``."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence, TypeVar

import numpy as np

from minilink.simulation import Simulator

_T = TypeVar("_T")

# ---------------------------------------------------------------------------
# Defaults (truth for error and speed ratio in matrix / single-pair benchmarks)
# ---------------------------------------------------------------------------

TRUTH_SOLVER = "scipy_ultra"
TRUTH_BACKEND = "numpy"

DEFAULT_SOLVERS: tuple[str, ...] = (
    "euler",
    "rk4_fixedsteps",
    "scipy",
    "scipy_stiff",
    "scipy_lsoda",
    "scipy_max",
    "scipy_ultra",
)
DEFAULT_BACKENDS: tuple[str, ...] = ("numpy", "jax")

# Full solver×backend grid for scripts that do not need a custom list.
DEFAULT_SWEEP_PAIRS: tuple[tuple[str, str], ...] = tuple(
    (s, b) for s in DEFAULT_SOLVERS for b in DEFAULT_BACKENDS
)

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
    """Run a callable multiple times; return wall times (s) and return values per run."""
    durations: list[float] = []
    outputs: list[_T] = []
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
# One candidate (solver, backend) vs truth
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BenchmarkResult:
    candidate_solver: str
    candidate_backend: str
    truth_solver: str
    truth_backend: str
    mean_time: float
    std_time: float
    truth_mean_time: float
    speedup_vs_truth: float
    rel_err_l2: float
    nfev: int
    njev: int
    n_t: int


def benchmark_sim_backend(
    system: Any,
    *,
    candidate_solver: str,
    candidate_backend: str,
    truth_solver: str = TRUTH_SOLVER,
    truth_backend: str = TRUTH_BACKEND,
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
) -> BenchmarkResult:
    """Benchmark one ``(solver, compile_backend)`` against a truth configuration on ``system``."""

    def _run(sol: str, back: str):
        sim = Simulator(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            solver=sol,
            verbose=False,
            compile_backend=back,
        )
        traj = sim.solve()
        x_end = np.asarray(traj.x[:, -1], dtype=float)
        return x_end, sim.last_debug

    cand_dur, cand_out = run_timed(
        lambda: _run(candidate_solver, candidate_backend), n_runs=n_runs
    )
    tru_dur, tru_out = run_timed(
        lambda: _run(truth_solver, truth_backend), n_runs=n_runs
    )

    cand_stats = summarize_durations(cand_dur)
    tru_stats = summarize_durations(tru_dur)

    cand_x = np.mean(np.stack([o[0] for o in cand_out], axis=0), axis=0)
    tru_x = np.mean(np.stack([o[0] for o in tru_out], axis=0), axis=0)
    rel_err = relative_l2_error(cand_x, tru_x)

    debug = cand_out[-1][1]
    speedup = (
        tru_stats.mean / cand_stats.mean
        if cand_stats.mean > 0
        else float("inf")
    )
    return BenchmarkResult(
        candidate_solver=candidate_solver,
        candidate_backend=candidate_backend,
        truth_solver=truth_solver,
        truth_backend=truth_backend,
        mean_time=cand_stats.mean,
        std_time=cand_stats.std,
        truth_mean_time=tru_stats.mean,
        speedup_vs_truth=speedup,
        rel_err_l2=rel_err,
        nfev=int(debug["nfev"]),
        njev=int(debug["njev"]),
        n_t=int(debug["n_t"]),
    )


# ---------------------------------------------------------------------------
# Matrix: list of (solver, backend) vs one truth
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MatrixRow:
    """One cell: time, error vs truth, accuracy gate, speed vs truth time."""

    solver: str
    backend: str
    mean_time: float
    std_time: float
    nfev: int
    njev: int
    n_t: int
    rel_err_l2: float
    accuracy_ok: bool
    speed_vs_truth: float


@dataclass(frozen=True)
class MatrixResult:
    """All rows for one ``system`` and time grid."""

    case_name: str
    t0: float
    tf: float
    dt: float
    n_runs: int
    truth_solver: str
    truth_backend: str
    truth_mean_time: float
    accuracy_threshold_pct: float
    rows: tuple[MatrixRow, ...]


def _stdout_color() -> bool:
    return bool(sys.stdout.isatty())


def _ansi(s: str, *codes: int) -> str:
    if not _stdout_color() or not codes:
        return s
    return f"\033[{';'.join(str(c) for c in codes)}m{s}\033[0m"


def _fastest_accurate_row_indices(
    rows: tuple[MatrixRow, ...], threshold: float
) -> set[int]:
    acc = [i for i, r in enumerate(rows) if r.rel_err_l2 < threshold]
    if not acc:
        return set()
    best = max(rows[i].speed_vs_truth for i in acc)
    return {i for i in acc if rows[i].speed_vs_truth >= best - 1e-15}


def print_sim_matrix_table(result: MatrixResult) -> None:
    """Print the matrix: truth time, per-cell err vs truth, color by accuracy threshold."""
    th = result.accuracy_threshold_pct
    print(f"\n=== {result.case_name} ===")
    print(
        f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} "
        f"truth=({result.truth_solver}, {result.truth_backend}) "
        f"truth_mean_t={result.truth_mean_time:.6f}s "
        f"green if rel_err<{th:g}% (accuracy)  speed=truth_t/cell_t"
    )
    print(
        "solver_mode      backend         mean [s]    std [s]   nfev   njev    n_t"
        "     rel_err_l2   ok spd_vs_T"
    )
    print(
        "--------------- --------------- ---------- ---------- ------ ------ ------"
        " -------------- --- -----------"
    )
    fast_idx = _fastest_accurate_row_indices(result.rows, th)
    for i, row in enumerate(result.rows):
        bl = row.backend
        ok = "Y" if row.accuracy_ok else "N"
        line = (
            f"{row.solver:<15} {bl:<15} {row.mean_time:>10.6f}"
            f" {row.std_time:>10.6f} {row.nfev:>6d} {row.njev:>6d}"
            f" {row.n_t:>6d} {row.rel_err_l2:>14.4f}%"
            f" {ok:>3} {row.speed_vs_truth:>11.2f}x"
        )
        if i in fast_idx:
            line = _ansi(line, 1, 92)
        elif row.accuracy_ok:
            line = _ansi(line, 32)
        else:
            line = _ansi(line, 31)
        print(line)


def benchmark_sim_speed_matrix(
    system: Any,
    *,
    case_name: str,
    pairs: Sequence[tuple[str, str]],
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
    truth_solver: str = TRUTH_SOLVER,
    truth_backend: str = TRUTH_BACKEND,
    accuracy_threshold_pct: float = 1.0,
    print_table: bool = True,
    progress: bool = False,
) -> MatrixResult:
    """Benchmark each ``(solver, compile_backend)`` in ``pairs`` against one truth run on ``system``.

    The truth pair sets the reference final state and ``truth_mean_time`` for
    ``speed_vs_truth = truth_mean_time / cell_mean_time`` on each row.

    A row is ``ok`` when ``rel_err_l2 < accuracy_threshold_pct`` (default 1%).

    Parameters
    ----------
    system
        A built system (same object used for every timed solve in this run).
    case_name
        Title for the printed table.
    pairs
        Ordered list of ``(solver, compile_backend)`` to benchmark.
    truth_solver, truth_backend
        Reference for error and for the speed column denominator.
    print_table
        If ``True``, print via :func:`print_sim_matrix_table`.
    progress
        If ``True``, print flushed progress lines.
    """
    pair_list = list(pairs)
    n_matrix = len(pair_list)

    def _p(msg: str) -> None:
        if progress:
            print(msg, flush=True)

    def _run(sol: str, back: str) -> tuple[np.ndarray, dict[str, Any]]:
        sim = Simulator(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            solver=sol,
            verbose=False,
            compile_backend=back,
        )
        traj = sim.solve()
        x_end = np.asarray(traj.x[:, -1], dtype=float)
        return x_end, sim.last_debug

    def _run_timed_pair(
        sol: str, back: str
    ) -> tuple[TimingStats, np.ndarray, dict[str, Any]]:
        durations, outputs = run_timed(lambda: _run(sol, back), n_runs=n_runs)
        x_stack = np.stack([o[0] for o in outputs], axis=0)
        x_mean = np.mean(x_stack, axis=0)
        stats = summarize_durations(durations)
        dbg = outputs[-1][1]
        return stats, x_mean, dbg

    _p(
        f"[benchmark] {case_name}: truth run "
        f"({truth_solver}, {truth_backend}) × {n_runs} …"
    )
    t_truth = time.perf_counter()
    truth_stats, truth_x, truth_dbg = _run_timed_pair(truth_solver, truth_backend)
    _p(f"[benchmark] {case_name}: truth finished in {time.perf_counter() - t_truth:.3f}s")

    truth_time = float(truth_stats.mean)

    rows: list[MatrixRow] = []
    for i_cell, (solver, backend) in enumerate(pair_list, start=1):
        if solver == truth_solver and backend == truth_backend:
            _p(
                f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                f"{solver} + {backend} (reuse truth)"
            )
            t_cell = time.perf_counter()
            stats, x_mean, dbg = truth_stats, truth_x, truth_dbg
        else:
            _p(
                f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                f"{solver} + {backend} × {n_runs} …"
            )
            t_cell = time.perf_counter()
            stats, x_mean, dbg = _run_timed_pair(solver, backend)

        rel = float(relative_l2_error(x_mean, truth_x))
        accuracy_ok = rel < accuracy_threshold_pct
        speed_vs = truth_time / stats.mean if stats.mean > 0 else float("inf")
        rows.append(
            MatrixRow(
                solver=solver,
                backend=backend,
                mean_time=stats.mean,
                std_time=stats.std,
                nfev=int(dbg["nfev"]),
                njev=int(dbg["njev"]),
                n_t=int(dbg["n_t"]),
                rel_err_l2=rel,
                accuracy_ok=accuracy_ok,
                speed_vs_truth=speed_vs,
            )
        )
        ok = "Y" if accuracy_ok else "N"
        _p(
            f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
            f"{solver} + {backend} done in {time.perf_counter() - t_cell:.3f}s "
            f"(mean/cell {stats.mean:.4f}s, err {rel:.4f}%, ok={ok})"
        )

    out = MatrixResult(
        case_name=case_name,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        truth_solver=truth_solver,
        truth_backend=truth_backend,
        truth_mean_time=truth_time,
        accuracy_threshold_pct=accuracy_threshold_pct,
        rows=tuple(rows),
    )
    if print_table:
        _p(f"[benchmark] {case_name}: printing results table …")
        print_sim_matrix_table(out)
    _p(f"[benchmark] {case_name}: finished ({n_matrix} matrix cells).")
    return out
