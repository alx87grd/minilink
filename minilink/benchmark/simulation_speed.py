"""Simulation benchmark utilities: single-pair, matrix, and standard-case suites."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence, TypeVar

import numpy as np

from minilink.simulation.simulator import Simulator

_T = TypeVar("_T")


def integration_method_for_solver_mode(solver_mode: str) -> str:
    """Table-friendly label: ``euler``, ``RK4``, or ``solve_ivp``'s ``method`` string."""
    scipy_methods = {
        "scipy": "RK45",
        "scipy_stiff": "Radau",
        "scipy_lsoda": "LSODA",
        "scipy_max": "DOP853",
        "scipy_ultra": "DOP853",
    }
    if solver_mode == "euler":
        return "euler"
    if solver_mode == "rk4_fixedsteps":
        return "RK4"
    if solver_mode in scipy_methods:
        return scipy_methods[solver_mode]
    raise ValueError(f"Unknown solver mode {solver_mode!r}")


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


def format_benchmark_backend_label(compile_backend: str) -> str:
    """Human-readable backend label for benchmark tables and result structs.

    For ``compile_backend="jax"``, returns ``jax(cpu)``, ``jax(gpu)``, etc., from the
    active JAX default device. Any other value is returned unchanged.
    """
    if compile_backend != "jax":
        return compile_backend
    try:
        import jax
    except ImportError:
        return "jax"
    try:
        plat = str(jax.default_backend()).lower()
    except Exception:
        return "jax"
    return f"jax({plat})"


def _compile_backend_key_for_compare(backend_label: str) -> str:
    """Map display labels such as ``jax(gpu)`` back to the compile API key ``jax``."""
    if (
        len(backend_label) >= 6
        and backend_label.startswith("jax(")
        and backend_label.endswith(")")
    ):
        return "jax"
    return backend_label


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
# Timed simulator (compile = ``_build_evaluator`` / ``sys.compile``; solve = ``.solve``)
# ---------------------------------------------------------------------------


class _TimedSimulator(Simulator):
    """Records wall time for :meth:`_build_evaluator` and :meth:`solve` (seconds)."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self._last_compile_s = 0.0
        self._last_solve_s = 0.0
        super().__init__(*args, **kwargs)

    def _build_evaluator(self, sys, compile_backend):
        t0 = time.perf_counter()
        ev = super()._build_evaluator(sys, compile_backend)
        self._last_compile_s = time.perf_counter() - t0
        return ev

    def solve(self):
        t0 = time.perf_counter()
        out = super().solve()
        self._last_solve_s = time.perf_counter() - t0
        return out


class _CacheCompileTimedSimulator(_TimedSimulator):
    """Benchmark-only: reuse ``sys.compile`` result per ``(id(sys), compile_backend)``.

    Does not change :class:`~minilink.simulation.Simulator`; used only for matrix sweeps
    when ``compile_once=True`` to avoid recompiling the same backend for every cell.
    """

    def __init__(self, *args: Any, _eval_cache: dict[tuple[int, str], Any], **kwargs: Any):
        self._eval_cache = _eval_cache
        super().__init__(*args, **kwargs)

    def _build_evaluator(self, sys, compile_backend):
        key = (id(sys), compile_backend)
        if key in self._eval_cache:
            self._last_compile_s = 0.0
            return self._eval_cache[key]
        t0 = time.perf_counter()
        ev = super()._build_evaluator(sys, compile_backend)
        self._last_compile_s = time.perf_counter() - t0
        self._eval_cache[key] = ev
        return ev


def _new_timed_sim(
    system: Any,
    t0: float,
    tf: float,
    dt: float,
    solver: str,
    compile_backend: str,
    *,
    evaluator_cache: dict[tuple[int, str], Any] | None = None,
) -> _TimedSimulator:
    kwargs = dict(
        t0=t0,
        tf=tf,
        dt=dt,
        solver=solver,
        verbose=False,
        compile_backend=compile_backend,
    )
    if evaluator_cache is not None:
        return _CacheCompileTimedSimulator(
            system,
            _eval_cache=evaluator_cache,
            **kwargs,
        )
    return _TimedSimulator(system, **kwargs)


@dataclass(frozen=True)
class _PairTimingResult:
    stats: TimingStats
    mean_compile_s: float
    mean_solve_s: float
    x_mean: np.ndarray
    debug: dict[str, Any]


def _mean_state_and_debug(
    outputs: list[tuple[np.ndarray, dict[str, Any]]],
) -> tuple[np.ndarray, dict[str, Any]]:
    x_stack = np.stack([x for x, _ in outputs], axis=0)
    return np.mean(x_stack, axis=0), outputs[-1][1]


def _resolve_backend_compile_time(
    backend: str,
    compile_s: float,
    first_compile_s_by_backend: dict[str, float] | None,
) -> float:
    if first_compile_s_by_backend is None:
        return float(compile_s)
    if compile_s > 0.0:
        first_compile_s_by_backend.setdefault(backend, float(compile_s))
    return float(first_compile_s_by_backend.get(backend, compile_s))


def _run_compile_once_pair(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    sol: str,
    back: str,
    n_runs: int,
    evaluator_cache: dict[tuple[int, str], Any] | None,
    first_compile_s_by_backend: dict[str, float] | None,
) -> _PairTimingResult:
    sim = _new_timed_sim(
        system,
        t0,
        tf,
        dt,
        sol,
        back,
        evaluator_cache=evaluator_cache,
    )
    mean_comp = _resolve_backend_compile_time(
        back,
        sim._last_compile_s,
        first_compile_s_by_backend,
    )

    def _solve_once() -> tuple[np.ndarray, dict[str, Any]]:
        traj = sim.solve()
        return np.asarray(traj.x[:, -1], dtype=float), sim.last_debug

    durs, outs = run_timed(_solve_once, n_runs=n_runs)
    stats = summarize_durations(durs)
    x_mean, dbg = _mean_state_and_debug(outs)
    return _PairTimingResult(
        stats=stats,
        mean_compile_s=mean_comp,
        mean_solve_s=stats.mean,
        x_mean=x_mean,
        debug=dbg,
    )


def _run_compile_each_pair(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    sol: str,
    back: str,
    n_runs: int,
) -> _PairTimingResult:
    def _run_once() -> tuple[np.ndarray, dict[str, Any], float, float]:
        sim = _new_timed_sim(system, t0, tf, dt, sol, back)
        traj = sim.solve()
        x_end = np.asarray(traj.x[:, -1], dtype=float)
        return x_end, sim.last_debug, sim._last_compile_s, sim._last_solve_s

    durs, outs = run_timed(_run_once, n_runs=n_runs)
    stats = summarize_durations(durs)
    x_mean = np.mean(np.stack([o[0] for o in outs], axis=0), axis=0)
    dbg = outs[-1][1]
    compile_times = [o[2] for o in outs]
    solve_times = [o[3] for o in outs]
    return _PairTimingResult(
        stats=stats,
        mean_compile_s=float(np.mean(compile_times)),
        mean_solve_s=float(np.mean(solve_times)),
        x_mean=x_mean,
        debug=dbg,
    )


def _run_timed_one_pair(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    sol: str,
    back: str,
    n_runs: int,
    compile_once: bool,
    evaluator_cache: dict[tuple[int, str], Any] | None = None,
    first_compile_s_by_backend: dict[str, float] | None = None,
) -> _PairTimingResult:
    """Return timing stats, compile/solve means, mean final state, and last debug."""
    if compile_once:
        return _run_compile_once_pair(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            sol=sol,
            back=back,
            n_runs=n_runs,
            evaluator_cache=evaluator_cache,
            first_compile_s_by_backend=first_compile_s_by_backend,
        )
    return _run_compile_each_pair(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        sol=sol,
        back=back,
        n_runs=n_runs,
    )


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
    mean_compile_time: float
    mean_solve_time: float
    truth_mean_time: float
    truth_mean_compile_time: float
    truth_mean_solve_time: float
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
    compile_once: bool = True,
) -> BenchmarkResult:
    """Benchmark one ``(solver, compile_backend)`` against a truth configuration on ``system``.

    Parameters
    ----------
    compile_once
        If ``True`` (default), compile once and time only :meth:`~minilink.simulation.Simulator.solve` for
        ``n_runs``. Then ``mean_time`` and ``speedup_vs_truth`` use **solve-only** means; compile
        appears in ``mean_compile_time`` (once) and ``mean_solve_time`` matches ``mean_time``.

        If ``False``, each run builds a new :class:`~minilink.simulation.Simulator` (recompiles each
        time). ``mean_time`` is the mean wall time of a full (build + solve) iteration;
        ``mean_compile_time`` / ``mean_solve_time`` are split the same way (``mean_time`` can be
        slightly larger than their sum due to other ``__init__`` work).
    """

    cand = _run_timed_one_pair(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        sol=candidate_solver,
        back=candidate_backend,
        n_runs=n_runs,
        compile_once=compile_once,
    )
    truth = _run_timed_one_pair(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        sol=truth_solver,
        back=truth_backend,
        n_runs=n_runs,
        compile_once=compile_once,
    )

    rel_err = relative_l2_error(cand.x_mean, truth.x_mean)
    speedup = (
        truth.stats.mean / cand.stats.mean
        if cand.stats.mean > 0
        else float("inf")
    )
    return BenchmarkResult(
        candidate_solver=candidate_solver,
        candidate_backend=format_benchmark_backend_label(candidate_backend),
        truth_solver=truth_solver,
        truth_backend=format_benchmark_backend_label(truth_backend),
        mean_time=cand.stats.mean,
        std_time=cand.stats.std,
        mean_compile_time=cand.mean_compile_s,
        mean_solve_time=cand.mean_solve_s,
        truth_mean_time=truth.stats.mean,
        truth_mean_compile_time=truth.mean_compile_s,
        truth_mean_solve_time=truth.mean_solve_s,
        speedup_vs_truth=speedup,
        rel_err_l2=rel_err,
        nfev=int(cand.debug["nfev"]),
        njev=int(cand.debug["njev"]),
        n_t=int(cand.debug["n_t"]),
    )


# ---------------------------------------------------------------------------
# Standard cases: three fixed ``Simulator`` scenarios
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StandardCase:
    """One named scenario: build a system, time grid, and a short table label."""

    id: str
    label: str
    t0: float
    tf: float
    dt: float
    build: Callable[[], Any]


def _pendulum_long() -> Any:
    from minilink.blocks.testing.basic import make_pendulum

    return make_pendulum()


def _spheres_short() -> Any:
    from minilink.blocks.testing.engine import make_physics_many_spheres

    return make_physics_many_spheres(nx=6, ny=4)


def _diagram_dense() -> Any:
    from minilink.blocks.testing.network import make_dense_network

    return make_dense_network(num_nodes=50, connections_per_node=5)


STANDARD_SIM_CASES: tuple[StandardCase, ...] = (
    StandardCase("pendulum_long", "Pendulum (long)", 0.0, 100.0, 0.01, _pendulum_long),
    StandardCase("spheres_short", "ManySpheres (short)", 0.0, 1.0, 0.01, _spheres_short),
    StandardCase("diagram_dense", "Dense network", 0.0, 5.0, 0.1, _diagram_dense),
)


def run_standard_sim_suite(
    candidate_solver: str,
    candidate_backend: str,
    *,
    truth_solver: str = TRUTH_SOLVER,
    truth_backend: str = TRUTH_BACKEND,
    n_runs: int = 1,
    compile_once: bool = True,
    cases: Sequence[StandardCase] = STANDARD_SIM_CASES,
) -> list[tuple[str, BenchmarkResult]]:
    """Run :func:`benchmark_sim_backend` on each standard case (fresh system per case)."""
    results: list[tuple[str, BenchmarkResult]] = []
    for case in cases:
        results.append(
            (
                case.label,
                benchmark_sim_backend(
                    case.build(),
                    candidate_solver=candidate_solver,
                    candidate_backend=candidate_backend,
                    truth_solver=truth_solver,
                    truth_backend=truth_backend,
                    t0=case.t0,
                    tf=case.tf,
                    dt=case.dt,
                    n_runs=n_runs,
                    compile_once=compile_once,
                ),
            )
        )
    return results


def print_standard_sim_suite(results: list[tuple[str, BenchmarkResult]]) -> None:
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
    for label, result in results:
        candidate = f"{result.candidate_solver}+{result.candidate_backend}"
        print(
            f"{label:<22} {result.mean_time:>10.5f} {result.speedup_vs_truth:>9.2f} "
            f"{result.rel_err_l2:>10.4f}  {candidate:<18}"
        )


# ---------------------------------------------------------------------------
# Matrix: list of (solver, backend) vs one truth
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MatrixRow:
    """One cell: time, error vs truth, accuracy gate, speed vs truth time."""

    solver: str
    method: str
    backend: str
    mean_time: float
    std_time: float
    mean_compile_time: float
    mean_solve_time: float
    total_s: float
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
    compile_once: bool
    truth_solver: str
    truth_backend: str
    truth_mean_time: float
    truth_mean_compile_time: float
    truth_mean_solve_time: float
    accuracy_threshold_pct: float
    rows: tuple[MatrixRow, ...]


def _stdout_color() -> bool:
    """Use ANSI colors when output is a TTY, when ``FORCE_COLOR`` is set, or in IPython/Jupyter.

    Respects https://no-color.org/ via ``NO_COLOR``. Notebooks often have a non-tty
    ``sys.stdout`` but still render escape codes in the cell output.
    """
    if os.environ.get("NO_COLOR", "").strip():
        return False
    if os.environ.get("FORCE_COLOR", "").strip() or os.environ.get(
        "CLICOLOR_FORCE", ""
    ).strip():
        return True
    if sys.stdout.isatty():
        return True
    try:
        from IPython import get_ipython
    except ImportError:
        return False
    try:
        if get_ipython() is not None:
            return True
    except Exception:
        pass
    return False


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


# --- Matrix: fixed column widths; titles are checked against sample data width.
# ``_MZ_BAK`` must fit ``jax(<platform>)`` (see :func:`format_benchmark_backend_label`).
_MZ_SOL, _MZ_MET, _MZ_BAK, _MZ_F = 15, 8, 10, 10


def _row_tail(
    nfev: int,
    njev: int,
    n_t: int,
    rel_err_l2: float,
    ok: str,
    speed_vs_truth: float,
) -> str:
    return (
        f" {nfev:>6d} {njev:>6d} {n_t:>6d} {rel_err_l2:>14.4f}%"
        f" {ok:>3} {speed_vs_truth:>11.2f}x"
    )


def _row_tail_titles() -> str:
    """Header for columns shared with :func:`_row_tail` (same string length as data)."""
    return (
        f" {'nfev':>6} {'njev':>6} {'n_t':>6} "
        f"{'L2% err':>15}"
        f" {'ok':>3} {'spd_vs_T':>11}x"
    )


def _matrix_titles(compile_once: bool) -> str:
    t = _row_tail_titles()
    if compile_once:
        h = (
            f"{'solver':<{_MZ_SOL}} {'metho':<{_MZ_MET}} {'back':<{_MZ_BAK}} "
            f"{'cmp [s]':>{_MZ_F}} {'solve [s]':>{_MZ_F}} {'std [s]':>{_MZ_F}} "
            f"{'total [s]':>{_MZ_F}}"
        )
        p_sol, p_met, p_b = "0" * _MZ_SOL, "0" * _MZ_MET, "0" * _MZ_BAK
        sample = (
            f"{p_sol:<{_MZ_SOL}} {p_met:<{_MZ_MET}} {p_b:<{_MZ_BAK}} "
            f"{0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f}"
        ) + _row_tail(0, 0, 0, 0.0, "Y", 0.0)
    else:
        h = (
            f"{'solver':<{_MZ_SOL}} {'metho':<{_MZ_MET}} {'back':<{_MZ_BAK}} "
            f"{'total [s]':>{_MZ_F}} {'std [s]':>{_MZ_F}} "
            f"{'cmp [s]':>{_MZ_F}} {'slv [s]':>{_MZ_F}}"
        )
        p_sol, p_met, p_b = "0" * _MZ_SOL, "0" * _MZ_MET, "0" * _MZ_BAK
        sample = (
            f"{p_sol:<{_MZ_SOL}} {p_met:<{_MZ_MET}} {p_b:<{_MZ_BAK}} "
            f"{0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f} {0.0:>{_MZ_F}.6f}"
        ) + _row_tail(0, 0, 0, 0.0, "Y", 0.0)
    line = h + t
    if len(line) != len(sample):
        raise RuntimeError(
            f"matrix title width {len(line)} != data width {len(sample)} (compile_once={compile_once})"
        )
    return line


def _format_matrix_row_text(row: MatrixRow, *, compile_once: bool) -> str:
    """Table columns: ``total [s]`` is wall time (``compile + n_runs * mean solve`` when
    ``compile_once``, else mean time per full iteration). ``slv`` is solve-only slice.
    """
    bl = row.backend
    ok = "Y" if row.accuracy_ok else "N"
    tail = _row_tail(
        row.nfev, row.njev, row.n_t, row.rel_err_l2, ok, row.speed_vs_truth
    )
    head3 = f"{row.solver:<{_MZ_SOL}} {row.method:<{_MZ_MET}} {bl:<{_MZ_BAK}} "
    if compile_once:
        return (
            f"{head3}{row.mean_compile_time:>{_MZ_F}.6f} "
            f"{row.mean_solve_time:>{_MZ_F}.6f} {row.std_time:>{_MZ_F}.6f} "
            f"{row.total_s:>{_MZ_F}.6f}" + tail
        )
    return (
        f"{head3}{row.total_s:>{_MZ_F}.6f} {row.std_time:>{_MZ_F}.6f} "
        f"{row.mean_compile_time:>{_MZ_F}.6f} {row.mean_solve_time:>{_MZ_F}.6f}" + tail
    )


def _print_matrix_row_line(
    row: MatrixRow,
    *,
    compile_once: bool,
    truth_solver: str,
    truth_backend: str,
    is_winner: bool = False,
    flush: bool = False,
) -> None:
    """Color: winner (fastest among accurate) > baseline (truth pair) > ok > fail."""
    line = _format_matrix_row_text(row, compile_once=compile_once)
    is_baseline = (
        row.solver == truth_solver
        and _compile_backend_key_for_compare(row.backend) == truth_backend
    )
    if is_winner:
        line = _ansi(line, 1, 92)
    elif is_baseline:
        line = _ansi(line, 1, 36)
    elif row.accuracy_ok:
        line = _ansi(line, 32)
    else:
        line = _ansi(line, 31)
    print(line, flush=flush)


def print_sim_matrix_header(result: MatrixResult) -> None:
    """Print title, truth summary, and column headings (no data rows)."""
    th = result.accuracy_threshold_pct
    co = "compile_once" if result.compile_once else "compile_each_run"
    print(f"\n=== {result.case_name} ===")
    if result.compile_once:
        truth_total = (
            result.truth_mean_compile_time
            + result.n_runs * result.truth_mean_solve_time
        )
        print(
            f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} mode={co} "
            f"truth=({result.truth_solver}, "
            f"{format_benchmark_backend_label(result.truth_backend)}) "
            f"truth_total={truth_total:.6f}s "
            f"truth_solve={result.truth_mean_time:.6f}s "
            f"truth_cmp={result.truth_mean_compile_time:.6f}s "
            f"green if rel_err<{th:g}%  speed=truth_solve/cell_solve"
        )
        titles = _matrix_titles(True)
        print(titles)
        print("-" * len(titles))
    else:
        print(
            f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} mode={co} "
            f"truth=({result.truth_solver}, "
            f"{format_benchmark_backend_label(result.truth_backend)}) "
            f"truth_total={result.truth_mean_time:.6f}s "
            f"truth_cmp={result.truth_mean_compile_time:.6f}s "
            f"truth_slv={result.truth_mean_solve_time:.6f}s "
            f"green if rel_err<{th:g}%  speed=truth_t/cell_t (full-iter time)"
        )
        titles = _matrix_titles(False)
        print(titles)
        print("-" * len(titles))


def _print_repeated_winner_rows(
    rows: tuple[MatrixRow, ...],
    *,
    compile_once: bool,
    truth_solver: str,
    truth_backend: str,
    th: float,
) -> None:
    """After a streamed table, reprint the fastest accurate row(s) in bold green only."""
    fast_idx = _fastest_accurate_row_indices(rows, th)
    if not fast_idx:
        return
    print(flush=True)
    label = "Winner row" if len(fast_idx) == 1 else "Tied winner rows"
    print(_ansi(f"── {label} (repeated) ──", 90), flush=True)
    for i in sorted(fast_idx):
        _print_matrix_row_line(
            rows[i],
            compile_once=compile_once,
            truth_solver=truth_solver,
            truth_backend=truth_backend,
            is_winner=True,
            flush=True,
        )


def _print_fastest_accurate_footer(rows: tuple[MatrixRow, ...], th: float) -> None:
    """One-line summary of tied fastest accurate rows (for streamed tables)."""
    fast_idx = _fastest_accurate_row_indices(rows, th)
    if not fast_idx:
        return
    best_spd = max(rows[i].speed_vs_truth for i in fast_idx)
    labels = [f"{rows[i].solver}+{rows[i].backend}" for i in sorted(fast_idx)]
    joined = ", ".join(labels)
    msg = (
        f"Fastest among accurate (rel_err < {th:g}%): {joined}  speed_vs_T={best_spd:.2f}x"
    )
    print(_ansi(msg, 1, 92), flush=True)


def print_sim_matrix_table(result: MatrixResult) -> None:
    """Print the full matrix: header plus all rows; winner=bold green, baseline=cyan."""
    co = result.compile_once
    print_sim_matrix_header(result)
    th = result.accuracy_threshold_pct
    fast_idx = _fastest_accurate_row_indices(result.rows, th)
    for i, row in enumerate(result.rows):
        _print_matrix_row_line(
            row,
            compile_once=co,
            truth_solver=result.truth_solver,
            truth_backend=result.truth_backend,
            is_winner=(i in fast_idx),
        )


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
    compile_once: bool = True,
) -> MatrixResult:
    """Benchmark each ``(solver, compile_backend)`` in ``pairs`` against one truth run on ``system``.

    Prints a live table: header after the truth run, one row per cell as it finishes
    (baseline cyan, accuracy green/red), then the fastest accurate cell is **reprinted
    in bold green** on one extra line, followed by a short text summary. If any accurate
    row exists, :func:`print_sim_matrix_table` on the result reproduces the same layout
    in one pass.

    The truth pair sets the reference final state. ``speed_vs_truth`` is
    ``truth_mean_time / cell_mean_time`` where both times use the same convention:
    **solve-only** means when ``compile_once=True`` (default); full (build + solve) per
    run when ``compile_once=False`` (see :func:`benchmark_sim_backend`).

    A row is ``ok`` when ``rel_err_l2 < accuracy_threshold_pct`` (default 1%).

    Parameters
    ----------
    system
        A built system (same object used for every timed solve in this run).
    case_name
        Title for the printed table.
    pairs
        Ordered list of ``(solver, compile_backend)`` to benchmark. If the truth pair
        appears in this list, it is run and printed **first** so the baseline row is
        at the top of the table.
    truth_solver, truth_backend
        Reference for error and for the speed column denominator.
    compile_once
        If ``True`` (default), one :func:`~minilink.core.framework.System.compile` per
        **backend** (cached across the matrix) and ``n_runs`` timed solves per cell; table
        columns: ``cmp / solve / std`` (``cmp`` is the one-time cost for that backend in
        this run). If ``False``, full rebuild every timed iteration; table columns:
        ``total / std / cmp / slv``.
    """
    pair_list = list(pairs)
    _tk = (truth_solver, truth_backend)
    if _tk in pair_list:
        pair_list = [_tk] + [p for p in pair_list if p != _tk]

    # Matrix run only: reuse compiled evaluator for the same (system, compile_backend).
    _eval_cache: dict[tuple[int, str], Any] = {}
    _first_cmp_by_backend: dict[str, float] = {}

    def _run_timed_pair(
        sol: str, back: str
    ) -> _PairTimingResult:
        return _run_timed_one_pair(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            sol=sol,
            back=back,
            n_runs=n_runs,
            compile_once=compile_once,
            evaluator_cache=_eval_cache if compile_once else None,
            first_compile_s_by_backend=_first_cmp_by_backend
            if compile_once
            else None,
        )

    truth = _run_timed_pair(truth_solver, truth_backend)
    truth_time = float(truth.stats.mean)

    print_sim_matrix_header(
        MatrixResult(
            case_name=case_name,
            t0=t0,
            tf=tf,
            dt=dt,
            n_runs=n_runs,
            compile_once=compile_once,
            truth_solver=truth_solver,
            truth_backend=truth_backend,
            truth_mean_time=truth_time,
            truth_mean_compile_time=truth.mean_compile_s,
            truth_mean_solve_time=truth.mean_solve_s,
            accuracy_threshold_pct=accuracy_threshold_pct,
            rows=(),
        )
    )

    rows: list[MatrixRow] = []
    for solver, backend in pair_list:
        if solver == truth_solver and backend == truth_backend:
            timed = truth
        else:
            timed = _run_timed_pair(solver, backend)

        rel = float(relative_l2_error(timed.x_mean, truth.x_mean))
        accuracy_ok = rel < accuracy_threshold_pct
        speed_vs = (
            truth_time / timed.stats.mean if timed.stats.mean > 0 else float("inf")
        )
        if compile_once:
            total_s = timed.mean_compile_s + n_runs * timed.mean_solve_s
        else:
            total_s = float(timed.stats.mean)
        row = MatrixRow(
            solver=solver,
            method=integration_method_for_solver_mode(solver),
            backend=format_benchmark_backend_label(backend),
            mean_time=timed.stats.mean,
            std_time=timed.stats.std,
            mean_compile_time=timed.mean_compile_s,
            mean_solve_time=timed.mean_solve_s,
            total_s=total_s,
            nfev=int(timed.debug["nfev"]),
            njev=int(timed.debug["njev"]),
            n_t=int(timed.debug["n_t"]),
            rel_err_l2=rel,
            accuracy_ok=accuracy_ok,
            speed_vs_truth=speed_vs,
        )
        rows.append(row)
        _print_matrix_row_line(
            row,
            compile_once=compile_once,
            truth_solver=truth_solver,
            truth_backend=truth_backend,
            is_winner=False,
            flush=True,
        )

    out = MatrixResult(
        case_name=case_name,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        compile_once=compile_once,
        truth_solver=truth_solver,
        truth_backend=truth_backend,
        truth_mean_time=truth_time,
        truth_mean_compile_time=truth.mean_compile_s,
        truth_mean_solve_time=truth.mean_solve_s,
        accuracy_threshold_pct=accuracy_threshold_pct,
        rows=tuple(rows),
    )
    _print_repeated_winner_rows(
        out.rows,
        compile_once=compile_once,
        truth_solver=truth_solver,
        truth_backend=truth_backend,
        th=accuracy_threshold_pct,
    )
    _print_fastest_accurate_footer(out.rows, accuracy_threshold_pct)
    return out