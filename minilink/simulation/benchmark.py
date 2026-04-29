"""Benchmarks for simulator solver and compile-backend combinations."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Sequence, TypeVar

import numpy as np

from minilink.compile.jax_utils import format_benchmark_backend_label
from minilink.simulation.simulator import Simulator

_T = TypeVar("_T")


@dataclass(frozen=True)
class SimulationBenchmarkVariant:
    """One simulator solver and compile-backend option."""

    solver: str
    compile_backend: str


@dataclass(frozen=True)
class SimulationBenchmarkCase:
    """One named simulation scenario."""

    id: str
    label: str
    t0: float
    tf: float
    dt: float
    build: Callable[[], Any]


@dataclass(frozen=True)
class SimulationBackendBenchmarkResult:
    """One candidate variant compared to one truth variant."""

    candidate: SimulationBenchmarkVariant
    truth: SimulationBenchmarkVariant
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


@dataclass(frozen=True)
class SimulationBenchmarkRow:
    """One row in a solver/backend sweep."""

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
class SimulationBenchmarkResult:
    """All rows for one system and time grid."""

    case_name: str
    t0: float
    tf: float
    dt: float
    n_runs: int
    compile_once: bool
    truth: SimulationBenchmarkVariant
    truth_mean_time: float
    truth_mean_compile_time: float
    truth_mean_solve_time: float
    accuracy_threshold_pct: float
    rows: tuple[SimulationBenchmarkRow, ...]


TRUTH_SIMULATION_VARIANT = SimulationBenchmarkVariant("scipy_ultra", "numpy")

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

DEFAULT_SIMULATION_VARIANTS: tuple[SimulationBenchmarkVariant, ...] = tuple(
    SimulationBenchmarkVariant(solver, backend)
    for solver in DEFAULT_SOLVERS
    for backend in DEFAULT_BACKENDS
)


def integration_method_for_solver_mode(solver_mode: str) -> str:
    """Table label for the numerical integration method."""
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


def benchmark_simulation_backend(
    system: Any,
    *,
    candidate: SimulationBenchmarkVariant,
    truth: SimulationBenchmarkVariant = TRUTH_SIMULATION_VARIANT,
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
    compile_once: bool = True,
) -> SimulationBackendBenchmarkResult:
    """Benchmark one simulator variant against one truth variant."""
    candidate_run = _run_simulation_variant(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        variant=candidate,
        n_runs=n_runs,
        compile_once=compile_once,
    )
    truth_run = _run_simulation_variant(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        variant=truth,
        n_runs=n_runs,
        compile_once=compile_once,
    )
    speedup = _speedup(truth_run.stats.mean, candidate_run.stats.mean)
    return SimulationBackendBenchmarkResult(
        candidate=candidate,
        truth=truth,
        mean_time=candidate_run.stats.mean,
        std_time=candidate_run.stats.std,
        mean_compile_time=candidate_run.mean_compile_s,
        mean_solve_time=candidate_run.mean_solve_s,
        truth_mean_time=truth_run.stats.mean,
        truth_mean_compile_time=truth_run.mean_compile_s,
        truth_mean_solve_time=truth_run.mean_solve_s,
        speedup_vs_truth=speedup,
        rel_err_l2=_relative_l2_error(candidate_run.x_mean, truth_run.x_mean),
        nfev=int(candidate_run.debug["nfev"]),
        njev=int(candidate_run.debug["njev"]),
        n_t=int(candidate_run.debug["n_t"]),
    )


def benchmark_simulation_matrix(
    system: Any,
    *,
    case_name: str,
    variants: Sequence[SimulationBenchmarkVariant],
    t0: float = 0.0,
    tf: float = 10.0,
    dt: float = 0.01,
    n_runs: int = 3,
    truth: SimulationBenchmarkVariant = TRUTH_SIMULATION_VARIANT,
    accuracy_threshold_pct: float = 1.0,
    compile_once: bool = True,
) -> SimulationBenchmarkResult:
    """Benchmark a list of simulator variants against one truth variant."""
    ordered_variants = _truth_first(tuple(variants), truth)
    evaluator_cache: dict[tuple[int, str], Any] = {}
    first_compile_s_by_backend: dict[str, float] = {}

    def run(variant: SimulationBenchmarkVariant) -> _SimulationRun:
        return _run_simulation_variant(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            variant=variant,
            n_runs=n_runs,
            compile_once=compile_once,
            evaluator_cache=evaluator_cache if compile_once else None,
            first_compile_s_by_backend=first_compile_s_by_backend
            if compile_once
            else None,
        )

    truth_run = run(truth)
    rows = tuple(
        _simulation_row(
            variant,
            truth_run,
            truth_run if variant == truth else run(variant),
            n_runs=n_runs,
            compile_once=compile_once,
            accuracy_threshold_pct=accuracy_threshold_pct,
        )
        for variant in ordered_variants
    )
    return SimulationBenchmarkResult(
        case_name=case_name,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        compile_once=compile_once,
        truth=truth,
        truth_mean_time=truth_run.stats.mean,
        truth_mean_compile_time=truth_run.mean_compile_s,
        truth_mean_solve_time=truth_run.mean_solve_s,
        accuracy_threshold_pct=accuracy_threshold_pct,
        rows=rows,
    )


def benchmark_standard_simulation_suite(
    candidate: SimulationBenchmarkVariant,
    *,
    truth: SimulationBenchmarkVariant = TRUTH_SIMULATION_VARIANT,
    n_runs: int = 1,
    compile_once: bool = True,
    cases: Sequence[SimulationBenchmarkCase] | None = None,
) -> list[tuple[str, SimulationBackendBenchmarkResult]]:
    """Run one simulator benchmark variant on each standard case."""
    if cases is None:
        cases = STANDARD_SIMULATION_CASES
    return [
        (
            case.label,
            benchmark_simulation_backend(
                case.build(),
                candidate=candidate,
                truth=truth,
                t0=case.t0,
                tf=case.tf,
                dt=case.dt,
                n_runs=n_runs,
                compile_once=compile_once,
            ),
        )
        for case in cases
    ]


def print_standard_simulation_benchmark(
    results: list[tuple[str, SimulationBackendBenchmarkResult]],
) -> None:
    """Print a compact standard-case benchmark table."""
    truth = results[0][1].truth if results else TRUTH_SIMULATION_VARIANT
    print("\n=== Standard simulator suite ===")
    print(
        f"truth=({truth.solver}, {truth.compile_backend})  "
        "speedup = truth_mean_time / candidate_mean_time  err = L2% vs truth final x"
    )
    print(f"{'case':<22} {'mean_s':>10} {'speedup':>9} {'err_%':>10}  {'cand':<18}")
    print("-" * 80)
    for label, result in results:
        backend = format_benchmark_backend_label(result.candidate.compile_backend)
        candidate = f"{result.candidate.solver}+{backend}"
        print(
            f"{label:<22} {result.mean_time:>10.5f} "
            f"{result.speedup_vs_truth:>9.2f} {result.rel_err_l2:>10.4f}  "
            f"{candidate:<18}"
        )


def print_simulation_matrix_benchmark(result: SimulationBenchmarkResult) -> None:
    """Print a full matrix benchmark table."""
    print_simulation_matrix_header(result)
    fast_idx = _fastest_accurate_row_indices(
        result.rows,
        result.accuracy_threshold_pct,
    )
    for i, row in enumerate(result.rows):
        _print_matrix_row_line(
            row,
            compile_once=result.compile_once,
            truth=result.truth,
            is_winner=(i in fast_idx),
        )
    _print_repeated_winner_rows(result, fast_idx)
    _print_fastest_accurate_footer(result.rows, result.accuracy_threshold_pct)


def print_simulation_matrix_header(result: SimulationBenchmarkResult) -> None:
    """Print title, truth summary, and column headings."""
    th = result.accuracy_threshold_pct
    mode = "compile_once" if result.compile_once else "compile_each_run"
    print(f"\n=== {result.case_name} ===")
    if result.compile_once:
        truth_total = (
            result.truth_mean_compile_time
            + result.n_runs * result.truth_mean_solve_time
        )
        print(
            f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} "
            f"mode={mode} truth=({result.truth.solver}, "
            f"{format_benchmark_backend_label(result.truth.compile_backend)}) "
            f"truth_total={truth_total:.6f}s "
            f"truth_solve={result.truth_mean_time:.6f}s "
            f"truth_cmp={result.truth_mean_compile_time:.6f}s "
            f"green if rel_err<{th:g}%  speed=truth_solve/cell_solve"
        )
    else:
        print(
            f"t0={result.t0} tf={result.tf} dt={result.dt} runs={result.n_runs} "
            f"mode={mode} truth=({result.truth.solver}, "
            f"{format_benchmark_backend_label(result.truth.compile_backend)}) "
            f"truth_total={result.truth_mean_time:.6f}s "
            f"truth_cmp={result.truth_mean_compile_time:.6f}s "
            f"truth_slv={result.truth_mean_solve_time:.6f}s "
            f"green if rel_err<{th:g}%  speed=truth_t/cell_t"
        )
    titles = _matrix_titles(result.compile_once)
    print(titles)
    print("-" * len(titles))


# ---------------------------------------------------------------------------
# Standard Cases
# ---------------------------------------------------------------------------


def _pendulum_long() -> Any:
    from minilink.simulation.scenarios.basic import make_pendulum

    return make_pendulum()


def _spheres_short() -> Any:
    from minilink.simulation.scenarios.engine import make_physics_many_spheres

    return make_physics_many_spheres(nx=6, ny=4)


def _diagram_dense() -> Any:
    from minilink.simulation.scenarios.network import make_dense_network

    return make_dense_network(num_nodes=50, connections_per_node=5)


STANDARD_SIMULATION_CASES: tuple[SimulationBenchmarkCase, ...] = (
    SimulationBenchmarkCase(
        "pendulum_long",
        "Pendulum (long)",
        0.0,
        100.0,
        0.01,
        _pendulum_long,
    ),
    SimulationBenchmarkCase(
        "spheres_short",
        "ManySpheres (short)",
        0.0,
        1.0,
        0.01,
        _spheres_short,
    ),
    SimulationBenchmarkCase(
        "diagram_dense",
        "Dense network",
        0.0,
        5.0,
        0.1,
        _diagram_dense,
    ),
)


# ---------------------------------------------------------------------------
# Simulation Runs
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _RuntimeStats:
    mean: float
    std: float
    min: float
    max: float


@dataclass(frozen=True)
class _SimulationRun:
    stats: _RuntimeStats
    mean_compile_s: float
    mean_solve_s: float
    x_mean: np.ndarray
    debug: dict[str, Any]


class _TimedSimulator(Simulator):
    """Simulator that records compile and solve wall time."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self._last_compile_s = 0.0
        self._last_solve_s = 0.0
        super().__init__(*args, **kwargs)

    def _build_evaluator(self, sys, compile_backend):
        t0 = time.perf_counter()
        evaluator = super()._build_evaluator(sys, compile_backend)
        self._last_compile_s = time.perf_counter() - t0
        return evaluator

    def solve(self):
        t0 = time.perf_counter()
        trajectory = super().solve()
        self._last_solve_s = time.perf_counter() - t0
        return trajectory


class _CachedCompileTimedSimulator(_TimedSimulator):
    """Timed simulator with a benchmark-local evaluator cache."""

    def __init__(self, *args: Any, _eval_cache: dict[tuple[int, str], Any], **kwargs: Any):
        self._eval_cache = _eval_cache
        super().__init__(*args, **kwargs)

    def _build_evaluator(self, sys, compile_backend):
        key = (id(sys), compile_backend)
        if key not in self._eval_cache:
            self._eval_cache[key] = super()._build_evaluator(sys, compile_backend)
            return self._eval_cache[key]
        self._last_compile_s = 0.0
        return self._eval_cache[key]


def _run_simulation_variant(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    variant: SimulationBenchmarkVariant,
    n_runs: int,
    compile_once: bool,
    evaluator_cache: dict[tuple[int, str], Any] | None = None,
    first_compile_s_by_backend: dict[str, float] | None = None,
) -> _SimulationRun:
    if compile_once:
        return _run_compile_once(
            system,
            t0=t0,
            tf=tf,
            dt=dt,
            variant=variant,
            n_runs=n_runs,
            evaluator_cache=evaluator_cache,
            first_compile_s_by_backend=first_compile_s_by_backend,
        )
    return _run_compile_each(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        variant=variant,
        n_runs=n_runs,
    )


def _run_compile_once(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    variant: SimulationBenchmarkVariant,
    n_runs: int,
    evaluator_cache: dict[tuple[int, str], Any] | None,
    first_compile_s_by_backend: dict[str, float] | None,
) -> _SimulationRun:
    sim = _new_timed_sim(
        system,
        t0,
        tf,
        dt,
        variant,
        evaluator_cache=evaluator_cache,
    )

    def solve_once() -> tuple[np.ndarray, dict[str, Any]]:
        traj = sim.solve()
        return traj.x[:, -1].copy(), sim.last_debug

    durations, outputs = _run_timed(solve_once, n_runs=n_runs)
    stats = _runtime_stats(durations)
    x_mean, debug = _mean_state_and_debug(outputs)
    return _SimulationRun(
        stats=stats,
        mean_compile_s=_compile_time(
            variant.compile_backend,
            sim._last_compile_s,
            first_compile_s_by_backend,
        ),
        mean_solve_s=stats.mean,
        x_mean=x_mean,
        debug=debug,
    )


def _run_compile_each(
    system: Any,
    *,
    t0: float,
    tf: float,
    dt: float,
    variant: SimulationBenchmarkVariant,
    n_runs: int,
) -> _SimulationRun:
    def run_once() -> tuple[np.ndarray, dict[str, Any], float, float]:
        sim = _new_timed_sim(system, t0, tf, dt, variant)
        traj = sim.solve()
        return traj.x[:, -1].copy(), sim.last_debug, sim._last_compile_s, sim._last_solve_s

    durations, outputs = _run_timed(run_once, n_runs=n_runs)
    stats = _runtime_stats(durations)
    return _SimulationRun(
        stats=stats,
        mean_compile_s=float(np.mean([out[2] for out in outputs])),
        mean_solve_s=float(np.mean([out[3] for out in outputs])),
        x_mean=np.mean(np.stack([out[0] for out in outputs], axis=0), axis=0),
        debug=outputs[-1][1],
    )


def _new_timed_sim(
    system: Any,
    t0: float,
    tf: float,
    dt: float,
    variant: SimulationBenchmarkVariant,
    *,
    evaluator_cache: dict[tuple[int, str], Any] | None = None,
) -> _TimedSimulator:
    kwargs = dict(
        t0=t0,
        tf=tf,
        dt=dt,
        solver=variant.solver,
        verbose=False,
        compile_backend=variant.compile_backend,
    )
    if evaluator_cache is None:
        return _TimedSimulator(system, **kwargs)
    return _CachedCompileTimedSimulator(system, _eval_cache=evaluator_cache, **kwargs)


def _simulation_row(
    variant: SimulationBenchmarkVariant,
    truth_run: _SimulationRun,
    run: _SimulationRun,
    *,
    n_runs: int,
    compile_once: bool,
    accuracy_threshold_pct: float,
) -> SimulationBenchmarkRow:
    rel_err = _relative_l2_error(run.x_mean, truth_run.x_mean)
    total_s = run.mean_compile_s + n_runs * run.mean_solve_s
    if not compile_once:
        total_s = run.stats.mean
    return SimulationBenchmarkRow(
        solver=variant.solver,
        method=integration_method_for_solver_mode(variant.solver),
        backend=format_benchmark_backend_label(variant.compile_backend),
        mean_time=run.stats.mean,
        std_time=run.stats.std,
        mean_compile_time=run.mean_compile_s,
        mean_solve_time=run.mean_solve_s,
        total_s=total_s,
        nfev=int(run.debug["nfev"]),
        njev=int(run.debug["njev"]),
        n_t=int(run.debug["n_t"]),
        rel_err_l2=rel_err,
        accuracy_ok=rel_err < accuracy_threshold_pct,
        speed_vs_truth=_speedup(truth_run.stats.mean, run.stats.mean),
    )


def _run_timed(func: Callable[[], _T], n_runs: int) -> tuple[list[float], list[_T]]:
    durations: list[float] = []
    outputs: list[_T] = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        outputs.append(func())
        durations.append(time.perf_counter() - t0)
    return durations, outputs


def _runtime_stats(durations: Sequence[float]) -> _RuntimeStats:
    t = np.array(durations, dtype=float)
    return _RuntimeStats(
        mean=float(np.mean(t)),
        std=float(np.std(t)),
        min=float(np.min(t)),
        max=float(np.max(t)),
    )


def _mean_state_and_debug(
    outputs: list[tuple[np.ndarray, dict[str, Any]]],
) -> tuple[np.ndarray, dict[str, Any]]:
    x = np.stack([state for state, _ in outputs], axis=0)
    return np.mean(x, axis=0), outputs[-1][1]


def _compile_time(
    backend: str,
    compile_s: float,
    first_compile_s_by_backend: dict[str, float] | None,
) -> float:
    if first_compile_s_by_backend is None:
        return float(compile_s)
    if compile_s > 0.0:
        first_compile_s_by_backend.setdefault(backend, float(compile_s))
    return float(first_compile_s_by_backend.get(backend, compile_s))


def _relative_l2_error(candidate: np.ndarray, reference: np.ndarray) -> float:
    diff_norm = np.linalg.norm(candidate - reference)
    ref_norm = np.linalg.norm(reference)
    if ref_norm > 0.0:
        return float(100.0 * diff_norm / ref_norm)
    if diff_norm == 0.0:
        return 0.0
    return float("inf")


def _speedup(reference_s: float, candidate_s: float) -> float:
    if candidate_s > 0.0:
        return reference_s / candidate_s
    return float("inf")


def _truth_first(
    variants: tuple[SimulationBenchmarkVariant, ...],
    truth: SimulationBenchmarkVariant,
) -> tuple[SimulationBenchmarkVariant, ...]:
    if truth not in variants:
        return variants
    return (truth, *(variant for variant in variants if variant != truth))


# ---------------------------------------------------------------------------
# Table Formatting
# ---------------------------------------------------------------------------


def _compile_backend_key_for_compare(backend_label: str) -> str:
    if backend_label.startswith("jax(") and backend_label.endswith(")"):
        return "jax"
    return backend_label


def _stdout_color() -> bool:
    if os.environ.get("NO_COLOR", "").strip():
        return False
    if os.environ.get("FORCE_COLOR", "").strip():
        return True
    if os.environ.get("CLICOLOR_FORCE", "").strip():
        return True
    return sys.stdout.isatty()


def _ansi(text: str, *codes: int) -> str:
    if not _stdout_color() or not codes:
        return text
    return f"\033[{';'.join(str(code) for code in codes)}m{text}\033[0m"


def _fastest_accurate_row_indices(
    rows: tuple[SimulationBenchmarkRow, ...],
    threshold: float,
) -> set[int]:
    accurate = [i for i, row in enumerate(rows) if row.rel_err_l2 < threshold]
    if not accurate:
        return set()
    best = max(rows[i].speed_vs_truth for i in accurate)
    return {i for i in accurate if rows[i].speed_vs_truth >= best - 1e-15}


_MZ_SOL, _MZ_MET, _MZ_BAK, _MZ_F = 15, 8, 10, 10


def _matrix_titles(compile_once: bool) -> str:
    tail = (
        f" {'nfev':>6} {'njev':>6} {'n_t':>6} "
        f"{'L2% err':>15} {'ok':>3} {'spd_vs_T':>11}x"
    )
    if compile_once:
        head = (
            f"{'solver':<{_MZ_SOL}} {'metho':<{_MZ_MET}} {'back':<{_MZ_BAK}} "
            f"{'cmp [s]':>{_MZ_F}} {'solve [s]':>{_MZ_F}} "
            f"{'std [s]':>{_MZ_F}} {'total [s]':>{_MZ_F}}"
        )
    else:
        head = (
            f"{'solver':<{_MZ_SOL}} {'metho':<{_MZ_MET}} {'back':<{_MZ_BAK}} "
            f"{'total [s]':>{_MZ_F}} {'std [s]':>{_MZ_F}} "
            f"{'cmp [s]':>{_MZ_F}} {'slv [s]':>{_MZ_F}}"
        )
    return head + tail


def _format_matrix_row(row: SimulationBenchmarkRow, *, compile_once: bool) -> str:
    ok = "Y" if row.accuracy_ok else "N"
    tail = (
        f" {row.nfev:>6d} {row.njev:>6d} {row.n_t:>6d} "
        f"{row.rel_err_l2:>14.4f}% {ok:>3} {row.speed_vs_truth:>11.2f}x"
    )
    head = f"{row.solver:<{_MZ_SOL}} {row.method:<{_MZ_MET}} {row.backend:<{_MZ_BAK}} "
    if compile_once:
        return (
            f"{head}{row.mean_compile_time:>{_MZ_F}.6f} "
            f"{row.mean_solve_time:>{_MZ_F}.6f} "
            f"{row.std_time:>{_MZ_F}.6f} {row.total_s:>{_MZ_F}.6f}"
            + tail
        )
    return (
        f"{head}{row.total_s:>{_MZ_F}.6f} {row.std_time:>{_MZ_F}.6f} "
        f"{row.mean_compile_time:>{_MZ_F}.6f} "
        f"{row.mean_solve_time:>{_MZ_F}.6f}"
        + tail
    )


def _print_matrix_row_line(
    row: SimulationBenchmarkRow,
    *,
    compile_once: bool,
    truth: SimulationBenchmarkVariant,
    is_winner: bool = False,
    flush: bool = False,
) -> None:
    line = _format_matrix_row(row, compile_once=compile_once)
    is_baseline = (
        row.solver == truth.solver
        and _compile_backend_key_for_compare(row.backend) == truth.compile_backend
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


def _print_repeated_winner_rows(
    result: SimulationBenchmarkResult,
    fast_idx: set[int],
) -> None:
    if not fast_idx:
        return
    print(flush=True)
    label = "Winner row" if len(fast_idx) == 1 else "Tied winner rows"
    print(_ansi(f"── {label} (repeated) ──", 90), flush=True)
    for i in sorted(fast_idx):
        _print_matrix_row_line(
            result.rows[i],
            compile_once=result.compile_once,
            truth=result.truth,
            is_winner=True,
            flush=True,
        )


def _print_fastest_accurate_footer(
    rows: tuple[SimulationBenchmarkRow, ...],
    threshold: float,
) -> None:
    fast_idx = _fastest_accurate_row_indices(rows, threshold)
    if not fast_idx:
        return
    best = max(rows[i].speed_vs_truth for i in fast_idx)
    labels = ", ".join(f"{rows[i].solver}+{rows[i].backend}" for i in sorted(fast_idx))
    msg = f"Fastest among accurate (rel_err < {threshold:g}%): {labels}  speed_vs_T={best:.2f}x"
    print(_ansi(msg, 1, 92), flush=True)
