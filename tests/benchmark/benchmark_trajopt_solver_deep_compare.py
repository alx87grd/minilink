"""Deep trajectory-optimization solver comparison (SciPy vs Ipopt).

Run from the repository root::

    python3 tests/benchmark/benchmark_trajopt_solver_deep_compare.py

This script goes beyond ``benchmark_trajopt_solver_presets.py`` by:

* **Cold timing modes** (``--cold``): default ``in-process`` repeats timed solves
  without clearing JAX (fast; best for scaling sweeps). ``jax-reset`` clears
  :func:`jax.clear_caches` once per variant plus one untimed warmup solve before
  repeats (isolates JIT between solvers; can be slow). ``subprocess`` uses a
  fresh interpreter per timed run (strictest; very slow).
* **Steady state**: after one in-process warmup, several timed runs without
  clearing caches (typical repeated-solve workflow).
* **Scaling**: grid size ``n_steps`` sweep with paired solvers.
* **Tolerance**: ``ftol`` / Ipopt ``tol`` ladder on the same nominal grid.
* **Presets** (``--preset``): ``quick`` omits the expensive trust-region + Hessian
  SciPy variant and skips the NumPy finite-difference baseline so the script
  finishes quickly; ``default`` / ``full`` include both.

The harness reuses :func:`~minilink.planning.trajectory_optimization.benchmark.benchmark_trajectory_optimization`
for each measurement so program construction matches the library benchmark.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from minilink.planning.trajectory_optimization.benchmark import (
    TrajectoryOptimizationBenchmarkConfig,
    TrajectoryOptimizationBenchmarkRow,
    TrajectoryOptimizationBenchmarkVariant,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_solver_variants,
    ipopt_trajopt_available,
    jax_trajopt_available,
)


def _variant_by_name(
    name: str,
    *,
    ftol: float,
    include_trust_constr: bool,
    include_ipopt: bool,
) -> TrajectoryOptimizationBenchmarkVariant:
    variants = default_trajectory_optimization_solver_variants(
        starts=("cold",),
        ftol=ftol,
        include_trust_constr=include_trust_constr,
        include_ipopt=include_ipopt,
    )
    for variant in variants:
        if variant.name == name:
            return variant
    raise KeyError(f"Unknown variant {name!r}. Available: {[v.name for v in variants]}")


def _row_dict(row: TrajectoryOptimizationBenchmarkRow) -> dict[str, object]:
    v = row.variant
    return {
        "variant": v.name,
        "optimizer_method": v.optimizer_method,
        "compile_backend": v.compile_backend,
        "success": row.success,
        "total_s": row.total_s,
        "transcribe_s": row.transcribe_s,
        "solve_s": row.solve_s,
        "nit": row.nit,
        "nfev": row.nfev,
        "njev": row.njev,
        "cost": row.cost,
        "eq_inf": row.eq_inf,
        "bound_inf": row.bound_inf,
        "message": row.message,
    }


def _run_once_in_process(
    config: TrajectoryOptimizationBenchmarkConfig,
    variant: TrajectoryOptimizationBenchmarkVariant,
) -> TrajectoryOptimizationBenchmarkRow:
    result = benchmark_trajectory_optimization(config, (variant,))
    return result.rows[0]


def _config_to_jsonable(
    cfg: TrajectoryOptimizationBenchmarkConfig,
) -> dict[str, object]:
    """Serialize config for ``json.dumps`` (paths as strings)."""
    w = cfg.warm_guess_path
    return {
        "case": cfg.case,
        "tf": cfg.tf,
        "n_steps": cfg.n_steps,
        "maxiter": cfg.maxiter,
        "ftol": cfg.ftol,
        "n_runs": cfg.n_runs,
        "warm_guess_path": str(w) if w is not None else None,
    }


def _run_worker_payload(payload: dict[str, object]) -> dict[str, object]:
    """Execute one benchmark row for JSON worker mode."""
    config = TrajectoryOptimizationBenchmarkConfig(**payload["config"])
    variant = _variant_by_name(
        str(payload["variant_name"]),
        ftol=float(payload["ftol"]),
        include_trust_constr=bool(payload.get("include_trust_constr", False)),
        include_ipopt=bool(payload.get("include_ipopt", True)),
    )
    row = _run_once_in_process(config, variant)
    return {"ok": True, "row": _row_dict(row)}


def _try_clear_jax_caches() -> None:
    """Drop XLA/JAX compilation caches when JAX is installed."""
    try:
        import jax

        jax.clear_caches()
    except (AttributeError, ImportError):
        pass


def _run_subprocess_row(
    repo_root: Path,
    payload: dict[str, object],
) -> dict[str, object]:
    script = Path(__file__).resolve()
    proc = subprocess.run(
        [sys.executable, str(script), "--worker"],
        cwd=str(repo_root),
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        check=False,
        timeout=600,
    )
    if proc.returncode != 0:
        return {
            "ok": False,
            "error": proc.stderr.strip() or proc.stdout.strip() or "unknown",
        }
    line = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else ""
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return {"ok": False, "error": f"bad worker stdout: {line!r}"}


def _mean_std(values: Sequence[float]) -> tuple[float, float | None]:
    if not values:
        return float("nan"), None
    mean = float(statistics.mean(values))
    if len(values) < 2:
        return mean, None
    return mean, float(statistics.stdev(values))


def _print_subtable(title: str, headers: tuple[str, ...], rows: list[tuple]) -> None:
    print(flush=True)
    print(title, flush=True)
    print("-" * len(title))
    colw = [
        max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(headers)
    ]
    head = "  ".join(f"{h:>{colw[i]}}" for i, h in enumerate(headers))
    print(head, flush=True)
    print("-" * len(head), flush=True)
    for r in rows:
        print(
            "  ".join(f"{str(r[i]):>{colw[i]}}" for i in range(len(headers))),
            flush=True,
        )


def _cold_batch_subprocess(
    repo_root: Path,
    *,
    config: TrajectoryOptimizationBenchmarkConfig,
    variant_names: Sequence[str],
    ftol: float,
    include_trust_constr: bool,
    include_ipopt: bool,
    repeats: int,
) -> dict[str, list[dict[str, object]]]:
    """Run each variant ``repeats`` times in isolated subprocesses."""
    out: dict[str, list[dict[str, object]]] = {name: [] for name in variant_names}
    base_payload = {
        "config": _config_to_jsonable(config),
        "ftol": ftol,
        "include_trust_constr": include_trust_constr,
        "include_ipopt": include_ipopt,
    }
    for name in variant_names:
        for _ in range(repeats):
            payload = {**base_payload, "variant_name": name}
            decoded = _run_subprocess_row(repo_root, payload)
            if not decoded.get("ok"):
                print(
                    f"subprocess error for {name}: {decoded.get('error')}",
                    file=sys.stderr,
                    flush=True,
                )
                out[name].append({"success": False})
            else:
                out[name].append(decoded["row"])  # type: ignore[index]
    return out


def _cold_batch_in_process(
    *,
    config: TrajectoryOptimizationBenchmarkConfig,
    variant_names: Sequence[str],
    ftol: float,
    include_trust_constr: bool,
    include_ipopt: bool,
    repeats: int,
) -> dict[str, list[dict[str, object]]]:
    """Timed repeats with no JAX cache clearing (fast cross-check)."""
    out: dict[str, list[dict[str, object]]] = {name: [] for name in variant_names}
    for name in variant_names:
        variant = _variant_by_name(
            name,
            ftol=ftol,
            include_trust_constr=include_trust_constr,
            include_ipopt=include_ipopt,
        )
        for _ in range(repeats):
            row = _run_once_in_process(config, variant)
            out[name].append(_row_dict(row))
    return out


def _cold_batch_jax_reset(
    *,
    config: TrajectoryOptimizationBenchmarkConfig,
    variant_names: Sequence[str],
    ftol: float,
    include_trust_constr: bool,
    include_ipopt: bool,
    repeats: int,
) -> dict[str, list[dict[str, object]]]:
    """Per-variant: clear caches, one untimed compile warmup, then timed repeats.

    Clearing JAX caches before **every** timed run would re-pay XLA compilation on
    each repetition (minutes per cell on this benchmark). Instead we isolate
    solvers from each other (fresh JIT when switching ``variant_names``) and
    report post-compile solve times, comparable to the steady-state block.
    """
    out: dict[str, list[dict[str, object]]] = {name: [] for name in variant_names}
    for name in variant_names:
        variant = _variant_by_name(
            name,
            ftol=ftol,
            include_trust_constr=include_trust_constr,
            include_ipopt=include_ipopt,
        )
        _try_clear_jax_caches()
        _run_once_in_process(config, variant)
        for _ in range(repeats):
            row = _run_once_in_process(config, variant)
            out[name].append(_row_dict(row))
    return out


def _cold_batch(
    repo_root: Path | None,
    *,
    config: TrajectoryOptimizationBenchmarkConfig,
    variant_names: Sequence[str],
    ftol: float,
    include_trust_constr: bool,
    include_ipopt: bool,
    repeats: int,
    mode: str,
) -> dict[str, list[dict[str, object]]]:
    if mode == "subprocess":
        if repo_root is None:
            raise ValueError("repo_root required for subprocess cold mode")
        return _cold_batch_subprocess(
            repo_root,
            config=config,
            variant_names=variant_names,
            ftol=ftol,
            include_trust_constr=include_trust_constr,
            include_ipopt=include_ipopt,
            repeats=repeats,
        )
    if mode == "jax-reset":
        return _cold_batch_jax_reset(
            config=config,
            variant_names=variant_names,
            ftol=ftol,
            include_trust_constr=include_trust_constr,
            include_ipopt=include_ipopt,
            repeats=repeats,
        )
    if mode == "in-process":
        return _cold_batch_in_process(
            config=config,
            variant_names=variant_names,
            ftol=ftol,
            include_trust_constr=include_trust_constr,
            include_ipopt=include_ipopt,
            repeats=repeats,
        )
    raise ValueError(f"Unknown cold mode {mode!r}")


def _steady_batch(
    config: TrajectoryOptimizationBenchmarkConfig,
    variants: Sequence[TrajectoryOptimizationBenchmarkVariant],
    *,
    warmup: bool,
    repeats: int,
) -> dict[str, list[TrajectoryOptimizationBenchmarkRow]]:
    rows_out: dict[str, list[TrajectoryOptimizationBenchmarkRow]] = {
        v.name: [] for v in variants
    }
    for variant in variants:
        if warmup:
            _run_once_in_process(config, variant)
        for _ in range(repeats):
            rows_out[variant.name].append(_run_once_in_process(config, variant))
    return rows_out


def _aggregate_rows(
    name: str, rows: Sequence[TrajectoryOptimizationBenchmarkRow | dict]
) -> dict[str, object]:
    ok = [
        r
        for r in rows
        if (
            r.success
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else r.get("success", False)
        )
    ]
    if not ok:
        return {"variant": name, "n_ok": 0}

    def _solve_s(r):
        return (
            r.solve_s
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else float(r["solve_s"])
        )

    def _total_s(r):
        return (
            r.total_s
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else float(r["total_s"])
        )

    def _trans_s(r):
        return (
            r.transcribe_s
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else float(r["transcribe_s"])
        )

    def _nit(r):
        v = r.nit if isinstance(r, TrajectoryOptimizationBenchmarkRow) else r.get("nit")
        return int(v) if v is not None else None

    def _nfev(r):
        v = (
            r.nfev
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else r.get("nfev")
        )
        return int(v) if v is not None else None

    def _cost(r):
        return (
            r.cost
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else r.get("cost")
        )

    def _eq(r):
        return (
            r.eq_inf
            if isinstance(r, TrajectoryOptimizationBenchmarkRow)
            else float(r["eq_inf"])
        )

    solves = [_solve_s(r) for r in ok]
    totals = [_total_s(r) for r in ok]
    trans = [_trans_s(r) for r in ok]
    nits = [_nit(r) for r in ok if _nit(r) is not None]
    nfevs = [_nfev(r) for r in ok if _nfev(r) is not None]
    costs = [_cost(r) for r in ok if _cost(r) is not None]
    eqs = [_eq(r) for r in ok]

    sm, ss = _mean_std(solves)
    tm, ts = _mean_std(totals)
    trm, trs = _mean_std(trans)

    nit_mean = float(statistics.mean(nits)) if nits else None
    nfev_mean = float(statistics.mean(nfevs)) if nfevs else None
    cost_mean = float(statistics.mean(costs)) if costs else None
    eq_mean = float(statistics.mean(eqs)) if eqs else None

    solve_per_nit = sm / nit_mean if nit_mean else None
    solve_per_nfev = sm / nfev_mean if nfev_mean else None

    return {
        "variant": name,
        "n_ok": len(ok),
        "n_total": len(rows),
        "solve_mean": sm,
        "solve_std": ss,
        "total_mean": tm,
        "total_std": ts,
        "trans_mean": trm,
        "trans_std": trs,
        "nit_mean": nit_mean,
        "nfev_mean": nfev_mean,
        "cost_mean": cost_mean,
        "eq_inf_mean": eq_mean,
        "solve_per_nit": solve_per_nit,
        "solve_per_nfev": solve_per_nfev,
    }


def main() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    if len(sys.argv) >= 2 and sys.argv[1] == "--worker":
        payload = json.loads(sys.stdin.read())
        try:
            result = _run_worker_payload(payload)
            print(json.dumps(result), flush=True)
        except Exception as exc:
            print(json.dumps({"ok": False, "error": repr(exc)}), flush=True)
        return

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preset",
        choices=("quick", "default", "full"),
        default="default",
        help="quick: smaller grids and fewer repeats; full: largest sweep",
    )
    parser.add_argument(
        "--cold",
        choices=("in-process", "jax-reset", "subprocess"),
        default="in-process",
        help="in-process: timed repeats only (default, fast). jax-reset: clear JAX caches "
        "once per variant plus one untimed warmup before repeats. subprocess: fresh "
        "Python interpreter per timed run (slowest).",
    )
    args = parser.parse_args()

    preset = {
        "quick": {
            "cold_repeats": 2,
            "steady_repeats": 3,
            "scaling_repeats": 1,
            "tol_repeats": 1,
            "n_steps_grid": (50, 100, 150),
            "ftol_grid": (1e-2, 1e-4),
            "base_maxiter": 320,
            "tol_maxiter": 500,
        },
        "default": {
            "cold_repeats": 3,
            "steady_repeats": 5,
            "scaling_repeats": 2,
            "tol_repeats": 2,
            "n_steps_grid": (40, 80, 120, 160),
            "ftol_grid": (1e-2, 1e-4, 1e-6),
            "base_maxiter": 400,
            "tol_maxiter": 700,
        },
        "full": {
            "cold_repeats": 4,
            "steady_repeats": 6,
            "scaling_repeats": 3,
            "tol_repeats": 3,
            "n_steps_grid": (40, 60, 80, 100, 120, 160, 200),
            "ftol_grid": (1e-2, 1e-4, 1e-6),
            "base_maxiter": 500,
            "tol_maxiter": 900,
        },
    }[args.preset]

    if not jax_trajopt_available():
        print(
            "JAX extra not available; install with pip install -e '.[jax]'.",
            file=sys.stderr,
        )
        sys.exit(1)

    cold_repeats = int(preset["cold_repeats"])
    steady_repeats = int(preset["steady_repeats"])
    scaling_repeats = int(preset["scaling_repeats"])
    tol_repeats = int(preset["tol_repeats"])
    n_steps_grid = preset["n_steps_grid"]
    ftol_grid = preset["ftol_grid"]
    base_maxiter = int(preset["base_maxiter"])
    tol_maxiter = int(preset["tol_maxiter"])

    base_config = TrajectoryOptimizationBenchmarkConfig(
        case="cartpole",
        tf=5.0,
        n_steps=100,
        maxiter=base_maxiter,
        ftol=1e-2,
        n_runs=1,
        warm_guess_path=None,
    )

    include_ipopt = ipopt_trajopt_available()
    if not include_ipopt:
        print(
            "note: cyipopt not importable; Ipopt rows are omitted.",
            file=sys.stderr,
            flush=True,
        )

    jax_names = ["jax-collocation-slsqp"]
    if include_ipopt:
        jax_names.append("jax-collocation-ipopt")
    if args.preset != "quick":
        jax_names.append("jax-collocation-trust")

    cold_mode = args.cold
    cold_repo = repo_root if cold_mode == "subprocess" else None

    print("=" * 88, flush=True)
    print("trajopt solver deep comparison (cartpole direct collocation)", flush=True)
    print("=" * 88, flush=True)
    print(
        f"preset={args.preset}  cold_mode={cold_mode}  "
        f"base grid: tf={base_config.tf} n_steps={base_config.n_steps} "
        f"maxiter={base_config.maxiter} ftol={base_config.ftol}",
        flush=True,
    )
    print(
        f"cold: {cold_repeats} rep/variant; steady: 1 warmup + {steady_repeats} timed reps",
        flush=True,
    )

    if args.preset != "quick":
        print(
            "\n## 0) NumPy + finite-difference baseline (single in-process run)",
            flush=True,
        )
        numpy_variant = _variant_by_name(
            "numpy-collocation-slsqp",
            ftol=float(base_config.ftol),
            include_trust_constr=False,
            include_ipopt=False,
        )
        t0 = time.perf_counter()
        numpy_row = _run_once_in_process(base_config, numpy_variant)
        numpy_dt = time.perf_counter() - t0
        print(
            f"  {numpy_row.variant.name}: total={numpy_row.total_s:.3f}s "
            f"(transcribe={numpy_row.transcribe_s:.3f} solve={numpy_row.solve_s:.3f}) "
            f"nit={numpy_row.nit} nfev={numpy_row.nfev} cost={numpy_row.cost:.4g} "
            f"eq_inf={numpy_row.eq_inf:.2e}  (wall for section {numpy_dt:.3f}s)",
            flush=True,
        )
    else:
        print(
            "\n## 0) NumPy + finite-difference baseline — skipped for preset=quick "
            "(run ``--preset default`` for the full FD reference on the same grid).",
            flush=True,
        )

    cold_label = {
        "subprocess": "## 1) Cold timing — subprocess (fresh interpreter per timed run)",
        "jax-reset": "## 1) Cold timing — jax.clear_caches + untimed warmup, then timed repeats",
        "in-process": "## 1) Cold timing — in-process repeats (no JAX cache clearing)",
    }[cold_mode]
    print(f"\n{cold_label}", flush=True)
    cold = _cold_batch(
        cold_repo,
        config=base_config,
        variant_names=jax_names,
        ftol=float(base_config.ftol),
        include_trust_constr=True,
        include_ipopt=include_ipopt,
        repeats=cold_repeats,
        mode=cold_mode,
    )
    cold_agg = [_aggregate_rows(name, cold[name]) for name in jax_names]
    _print_subtable(
        "aggregate (successful runs only)",
        (
            "variant",
            "n_ok",
            "solve_s μ",
            "σ",
            "total_s μ",
            "nit μ",
            "nfev μ",
            "cost μ",
            "eq_inf μ",
            "s/nit",
            "s/nfev",
        ),
        [
            (
                a["variant"],
                a.get("n_ok", 0),
                f"{a.get('solve_mean', float('nan')):.3f}",
                f"{a.get('solve_std') or 0:.3f}"
                if a.get("solve_std") is not None
                else "n/a",
                f"{a.get('total_mean', float('nan')):.3f}",
                f"{a.get('nit_mean', 0):.1f}" if a.get("nit_mean") else "n/a",
                f"{a.get('nfev_mean', 0):.1f}" if a.get("nfev_mean") else "n/a",
                f"{a.get('cost_mean', 0):.4g}"
                if a.get("cost_mean") is not None
                else "n/a",
                f"{a.get('eq_inf_mean', 0):.2e}"
                if a.get("eq_inf_mean") is not None
                else "n/a",
                f"{a.get('solve_per_nit', 0):.4f}" if a.get("solve_per_nit") else "n/a",
                f"{a.get('solve_per_nfev', 0):.5f}"
                if a.get("solve_per_nfev")
                else "n/a",
            )
            for a in cold_agg
        ],
    )

    print(
        "\n## 2) Steady in-process timing (one warmup, then averaged repeats)",
        flush=True,
    )
    steady_variants = [
        _variant_by_name(
            n,
            ftol=float(base_config.ftol),
            include_trust_constr=True,
            include_ipopt=include_ipopt,
        )
        for n in jax_names
    ]
    t0 = time.perf_counter()
    steady = _steady_batch(
        base_config,
        steady_variants,
        warmup=True,
        repeats=steady_repeats,
    )
    steady_dt = time.perf_counter() - t0
    steady_agg = [_aggregate_rows(name, steady[name]) for name in jax_names]
    _print_subtable(
        f"steady aggregate (wall for whole block {steady_dt:.1f}s)",
        (
            "variant",
            "n_ok",
            "solve_s μ",
            "σ",
            "nit μ",
            "nfev μ",
            "cost μ",
            "eq_inf μ",
            "s/nit",
            "s/nfev",
        ),
        [
            (
                a["variant"],
                a.get("n_ok", 0),
                f"{a.get('solve_mean', float('nan')):.3f}",
                f"{a.get('solve_std') or 0:.3f}"
                if a.get("solve_std") is not None
                else "n/a",
                f"{a.get('nit_mean', 0):.1f}" if a.get("nit_mean") else "n/a",
                f"{a.get('nfev_mean', 0):.1f}" if a.get("nfev_mean") else "n/a",
                f"{a.get('cost_mean', 0):.4g}"
                if a.get("cost_mean") is not None
                else "n/a",
                f"{a.get('eq_inf_mean', 0):.2e}"
                if a.get("eq_inf_mean") is not None
                else "n/a",
                f"{a.get('solve_per_nit', 0):.4f}" if a.get("solve_per_nit") else "n/a",
                f"{a.get('solve_per_nfev', 0):.5f}"
                if a.get("solve_per_nfev")
                else "n/a",
            )
            for a in steady_agg
        ],
    )

    scale_caption = f"rows: mean of {scaling_repeats} cold ({cold_mode}) runs per cell"
    print(f"\n## 3) Scaling: solve_s mean vs n_steps ({cold_mode})", flush=True)
    scale_names = ["jax-collocation-slsqp"]
    if include_ipopt:
        scale_names.append("jax-collocation-ipopt")
    scale_rows: list[tuple] = []
    for ns in n_steps_grid:
        cfg = TrajectoryOptimizationBenchmarkConfig(
            case="cartpole",
            tf=5.0,
            n_steps=int(ns),
            maxiter=base_maxiter,
            ftol=float(base_config.ftol),
            n_runs=1,
            warm_guess_path=None,
        )
        batch = _cold_batch(
            cold_repo,
            config=cfg,
            variant_names=scale_names,
            ftol=float(base_config.ftol),
            include_trust_constr=False,
            include_ipopt=include_ipopt,
            repeats=scaling_repeats,
            mode=cold_mode,
        )
        for name in scale_names:
            agg = _aggregate_rows(name, batch[name])
            scale_rows.append(
                (
                    str(ns),
                    name,
                    f"{agg.get('solve_mean', float('nan')):.3f}",
                    f"{agg.get('nit_mean', 0):.0f}" if agg.get("nit_mean") else "n/a",
                    f"{agg.get('cost_mean', 0):.4g}" if agg.get("cost_mean") else "n/a",
                    f"{agg.get('eq_inf_mean', 0):.2e}"
                    if agg.get("eq_inf_mean")
                    else "n/a",
                )
            )
    _print_subtable(
        scale_caption,
        ("n_steps", "variant", "solve_s μ", "nit μ", "cost μ", "eq_inf μ"),
        scale_rows,
    )

    tol_caption = f"mean of {tol_repeats} cold ({cold_mode}) runs; Ipopt tol == SciPy ftol numerically"
    print(f"\n## 4) Tolerance ladder ({cold_mode})", flush=True)
    tol_rows: list[tuple] = []
    for ft in ftol_grid:
        cfg = TrajectoryOptimizationBenchmarkConfig(
            case="cartpole",
            tf=5.0,
            n_steps=100,
            maxiter=tol_maxiter,
            ftol=float(ft),
            n_runs=1,
            warm_guess_path=None,
        )
        names = ["jax-collocation-slsqp"]
        if include_ipopt:
            names.append("jax-collocation-ipopt")
        batch = _cold_batch(
            cold_repo,
            config=cfg,
            variant_names=names,
            ftol=float(ft),
            include_trust_constr=False,
            include_ipopt=include_ipopt,
            repeats=tol_repeats,
            mode=cold_mode,
        )
        for name in names:
            agg = _aggregate_rows(name, batch[name])
            tol_rows.append(
                (
                    f"{ft:g}",
                    name,
                    f"{agg.get('solve_mean', float('nan')):.3f}",
                    f"{agg.get('nit_mean', 0):.0f}" if agg.get("nit_mean") else "n/a",
                    f"{agg.get('cost_mean', 0):.4g}" if agg.get("cost_mean") else "n/a",
                    f"{agg.get('eq_inf_mean', 0):.2e}"
                    if agg.get("eq_inf_mean")
                    else "n/a",
                )
            )
    _print_subtable(
        tol_caption,
        ("ftol/tol", "variant", "solve_s μ", "nit μ", "cost μ", "eq_inf μ"),
        tol_rows,
    )

    print("\n## 5) Interpretation notes", flush=True)
    print(
        "- Section 0 (when enabled) shows why finite-difference NumPy is the wrong baseline "
        "for solver speed: almost all time is Jacobian estimation (huge nfev).\n"
        "- Default ``--cold in-process`` is fast and suits scaling sweeps; the first variant "
        "may pay a one-time JAX compile cost that later variants reuse.\n"
        "- ``--cold jax-reset`` clears caches once per solver plus one untimed warmup before "
        "repeats (better isolation between solvers; first warmup can be slow).\n"
        "- ``--cold subprocess`` fully isolates each timed run but is usually only needed for "
        "sanity checks.\n"
        "- ``cost`` and ``eq_inf`` differ across solvers and tolerances: wall time is not the same "
        "metric as identical NLP termination.\n"
        "- ``jax-collocation-trust`` uses a dense Hessian; cost per iteration is usually higher "
        "than SLSQP or Ipopt with ``use_hessian=False``.\n"
        "- Wall-time ranking is not monotonic in grid size: on this cartpole benchmark, Ipopt "
        "can spike (many iterations) near certain ``n_steps`` while SLSQP exits earlier—pair "
        "``solve_s`` with ``nit``/``nfev`` and ``cost``/``eq_inf`` before drawing conclusions.\n"
        "- Ipopt's typical advantage is on large sparse structures; this dense collocation "
        "program is a modest-sized stress test, not a definitive ranking of the solvers.",
        flush=True,
    )


if __name__ == "__main__":
    main()
