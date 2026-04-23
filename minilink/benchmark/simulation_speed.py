"""Simulation benchmark utilities: timing helpers, baseline ``Simulator`` compare, solver×backend matrix."""

from __future__ import annotations

import sys
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
    "scipy_lsoda",
    "scipy_max",
    "scipy_ultra",
)
DEFAULT_BACKENDS: tuple[str, ...] = ("numpy", "jax")


class SimMatrixRow(NamedTuple):
    """One (solver, backend) cell: error vs truth, ``<threshold``% gate, speed vs truth."""

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


class SimulatorMatrixResult(NamedTuple):
    """Full matrix for one ``system_factory`` and time grid."""

    case_name: str
    t0: float
    tf: float
    dt: float
    n_runs: int
    baseline_solver: str
    baseline_backend: str
    anchor_solver: str
    anchor_backend: str
    anchor_mean_time: float
    anchor_rel_err_l2: float
    precision_note: str
    rows: tuple[SimMatrixRow, ...]
    truth_mean_time: float
    accuracy_threshold_pct: float


def _backend_label(backend: str) -> str:
    if backend != "jax":
        return backend
    try:
        import jax

        return f"jax({jax.default_backend()})"
    except ImportError:
        return "jax"


def _stdout_color() -> bool:
    return bool(sys.stdout.isatty())


def _ansi(s: str, *codes: int) -> str:
    if not _stdout_color() or not codes:
        return s
    return f"\033[{';'.join(str(c) for c in codes)}m{s}\033[0m"


def _fastest_accurate_row_indices(
    rows: tuple[SimMatrixRow, ...], threshold: float
) -> set[int]:
    """Row indices with ``rel_err_l2 < threshold`` and maximum ``speed_vs_truth`` (ties included)."""
    acc = [i for i, r in enumerate(rows) if r.rel_err_l2 < threshold]
    if not acc:
        return set()
    best = max(rows[i].speed_vs_truth for i in acc)
    return {i for i in acc if rows[i].speed_vs_truth >= best - 1e-15}


def print_sim_matrix_table(result: SimulatorMatrixResult) -> None:
    """Print the simulator matrix with truth reference, 1% accuracy colors, and speed vs truth."""
    th = result.accuracy_threshold_pct
    print(f"\n=== {result.case_name} ===")
    same_anchor = (result.anchor_solver, result.anchor_backend) == (
        result.baseline_solver,
        result.baseline_backend,
    )
    anchor_bit = (
        f" anchor=({result.anchor_solver}, {result.anchor_backend})"
        f" anchor_err_vs_truth={result.anchor_rel_err_l2:.6f}%"
        f" anchor_mean_t={result.anchor_mean_time:.6f}s"
    )
    if same_anchor:
        anchor_bit = " anchor=truth (deduped)"
    print(
        f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} "
        f"truth=({result.baseline_solver}, {result.baseline_backend}) "
        f"truth_mean_t={result.truth_mean_time:.6f}s "
        f"green if rel_err<{th:g}% (accuracy) speed=truth_t/cell_t "
        f"precision={result.precision_note}"
        f"{anchor_bit}"
    )
    print(
        "solver_mode      backend          mean [s]    std [s]   nfev   njev    n_t"
        "     rel_err_l2   ok spd_vs_T"
    )
    print(
        "--------------- --------------- ---------- ---------- ------ ------ ------"
        " -------------- --- -----------"
    )
    fast_idx = _fastest_accurate_row_indices(result.rows, th)
    for i, row in enumerate(result.rows):
        bl = _backend_label(row.backend)
        ok = "Y" if row.accuracy_ok else "N"
        line = (
            f"{row.solver:<15} {bl:<15} {row.mean_time:>10.6f}"
            f" {row.std_time:>10.6f} {row.nfev:>6d} {row.njev:>6d}"
            f" {row.n_t:>6d} {row.rel_err_l2:>14.4f}%"
            f" {ok:>3} {row.speed_vs_truth:>11.2f}x"
        )
        if i in fast_idx:
            line = _ansi(line, 1, 92)  # bold + bright green: fastest under threshold
        elif row.accuracy_ok:
            line = _ansi(line, 32)  # green
        else:
            line = _ansi(line, 31)  # red
        print(line)


def benchmark_sim_speed_matrix(
    system_factory: Callable[[], Any],
    *,
    case_name: str,
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
    baseline_solver: str = "scipy_max",
    baseline_backend: str = "numpy",
    accuracy_threshold_pct: float = 1.0,
    anchor_solver: str = "rk4_fixedsteps",
    anchor_backend: str = "jax",
    solvers: Sequence[str] | None = None,
    backends: Sequence[str] | None = None,
    precision_note: str = "float32",
    print_table: bool = True,
    progress: bool = False,
) -> SimulatorMatrixResult:
    """Benchmark every ``(solver, backend)`` pair vs a truth run.

    **Truth** ``(baseline_solver, baseline_backend)`` defines the reference final state
    (for ``rel_err_l2``) and the speed column ``truth_mean_time / cell_mean_time``.

    A cell has ``ok=Y`` in the table when ``rel_err_l2 < accuracy_threshold_pct``
    (default 1%).

    The **anchor** pair (defaults ``rk4_fixedsteps`` + ``jax``) is optional context in
    the table header. If it equals the baseline, the anchor run is skipped (reuses truth).

    Parameters
    ----------
    system_factory
        Zero-argument callable returning a fresh system (same contract as
        :func:`benchmark_sim_backend`).
    case_name
        Printed title for the matrix block.
    baseline_solver, baseline_backend
        Truth reference for error and for speed ratio denominator.
    accuracy_threshold_pct
        ``ok`` is true in each row when ``rel_err_l2`` is strictly below this value.
    anchor_solver, anchor_backend
        Optional second reference; reported in the header when different from the baseline.
    solvers, backends
        Explicit lists to sweep. Defaults: :data:`DEFAULT_SOLVERS`,
        :data:`DEFAULT_BACKENDS`.
    precision_note
        Short label printed in the table header (e.g. ``\"float32\"`` / ``\"float64\"``).
    print_table
        If ``True``, print the matrix via :func:`print_sim_matrix_table`.
    progress
        If ``True``, print flushed progress lines (truth run, anchor run, each matrix cell).

    Returns
    -------
    SimulatorMatrixResult
        One row per ``(solver, backend)`` in nested iteration order.
    """
    solvers_t = tuple(solvers) if solvers is not None else DEFAULT_SOLVERS
    backends_t = tuple(backends) if backends is not None else DEFAULT_BACKENDS
    n_matrix = len(solvers_t) * len(backends_t)

    def _p(msg: str) -> None:
        if progress:
            print(msg, flush=True)

    def _run(solver: str, backend: str) -> tuple[np.ndarray, dict[str, Any]]:
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

    def _run_timed_pair(solver: str, backend: str) -> tuple[TimingStats, np.ndarray, dict[str, Any]]:
        durations, outputs = run_timed(
            lambda: _run(solver, backend), n_runs=n_runs
        )
        x_stack = np.stack([out[0] for out in outputs], axis=0)
        x_mean = np.mean(x_stack, axis=0)
        stats = summarize_durations(durations)
        debug = outputs[-1][1]
        return stats, x_mean, debug

    _p(
        f"[benchmark] {case_name}: truth run "
        f"({baseline_solver}, {baseline_backend}) × {n_runs} …"
    )
    t_truth = time.perf_counter()
    truth_stats, truth_x, _truth_dbg = _run_timed_pair(
        baseline_solver, baseline_backend
    )
    _p(f"[benchmark] {case_name}: truth finished in {time.perf_counter() - t_truth:.3f}s")

    truth_time = float(truth_stats.mean)
    same_as_truth = (anchor_solver, anchor_backend) == (
        baseline_solver,
        baseline_backend,
    )
    if same_as_truth:
        anchor_stats, anchor_x, anchor_dbg = truth_stats, truth_x, _truth_dbg
        _p(
            f"[benchmark] {case_name}: anchor=truth — skipping duplicate run "
            f"({anchor_solver}, {anchor_backend})"
        )
    else:
        _p(
            f"[benchmark] {case_name}: anchor run "
            f"({anchor_solver}, {anchor_backend}) × {n_runs} …"
        )
        t_anchor0 = time.perf_counter()
        anchor_stats, anchor_x, anchor_dbg = _run_timed_pair(anchor_solver, anchor_backend)
        _p(
            f"[benchmark] {case_name}: anchor finished in "
            f"{time.perf_counter() - t_anchor0:.3f}s"
        )

    anchor_time = float(anchor_stats.mean)
    anchor_rel = 0.0 if same_as_truth else float(relative_l2_error(anchor_x, truth_x))

    rows: list[SimMatrixRow] = []
    i_cell = 0
    for solver in solvers_t:
        for backend in backends_t:
            i_cell += 1
            bl = _backend_label(backend)
            if solver == baseline_solver and backend == baseline_backend:
                _p(
                    f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                    f"{solver} + {bl} (reuse truth)"
                )
                t_cell = time.perf_counter()
                stats, x_mean, dbg = truth_stats, truth_x, _truth_dbg
            elif (
                (solver, backend) == (anchor_solver, anchor_backend)
                and (anchor_solver, anchor_backend) != (baseline_solver, baseline_backend)
            ):
                _p(
                    f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                    f"{solver} + {bl} (reuse anchor)"
                )
                t_cell = time.perf_counter()
                stats, x_mean, dbg = anchor_stats, anchor_x, anchor_dbg
            else:
                _p(
                    f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                    f"{solver} + {bl} × {n_runs} …"
                )
                t_cell = time.perf_counter()
                stats, x_mean, dbg = _run_timed_pair(solver, backend)

            rel = float(relative_l2_error(x_mean, truth_x))
            accuracy_ok = rel < accuracy_threshold_pct
            speed_vs = truth_time / stats.mean if stats.mean > 0 else float("inf")

            rows.append(
                SimMatrixRow(
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
            dt_cell = time.perf_counter() - t_cell
            ok = "Y" if accuracy_ok else "N"
            _p(
                f"[benchmark] {case_name}: [{i_cell}/{n_matrix}] "
                f"{solver} + {bl} done in {dt_cell:.3f}s "
                f"(mean/cell {stats.mean:.4f}s, err {rel:.4f}%, ok={ok})"
            )

    out = SimulatorMatrixResult(
        case_name=case_name,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        baseline_solver=baseline_solver,
        baseline_backend=baseline_backend,
        anchor_solver=anchor_solver,
        anchor_backend=anchor_backend,
        anchor_mean_time=anchor_time,
        anchor_rel_err_l2=anchor_rel,
        precision_note=precision_note,
        rows=tuple(rows),
        truth_mean_time=truth_time,
        accuracy_threshold_pct=accuracy_threshold_pct,
    )
    if print_table:
        _p(f"[benchmark] {case_name}: printing results table …")
        print_sim_matrix_table(out)
    _p(f"[benchmark] {case_name}: finished ({n_matrix} matrix cells).")
    return out
