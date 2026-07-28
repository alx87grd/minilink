"""Run perf and integration regression checks against committed baselines.

Usage (from repo root)::

    python benchmarks/run_regression_check.py
    python benchmarks/run_regression_check.py --suite integration
    python benchmarks/run_regression_check.py --suite e4
    python benchmarks/run_regression_check.py --suite f_mpc
    python benchmarks/run_regression_check.py --suite solve_speed
    python benchmarks/run_regression_check.py --suite all
    python benchmarks/run_regression_check.py --suite all --tiny
    python benchmarks/run_regression_check.py --update
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
from collections.abc import Callable
from dataclasses import dataclass
from datetime import date
from pathlib import Path

from benchmarks.baseline import (
    BaselineFile,
    MetricRecord,
    compare_metrics,
    default_regression_factor,
    filter_speed_gate_failures,
    load_baseline,
    merge_recorded_into_baseline,
    print_comparison_report,
    save_baseline,
)
from benchmarks.host_profiles import (
    PROFILES_DIR,
    print_host_speed_context,
    save_host_profile,
)
from benchmarks.scenarios.e4_trajopt_parity import (
    E4TrajoptParityConfig,
    run_e4_trajopt_parity_suite,
)
from benchmarks.scenarios.f_mpc_parity import run_f_mpc_parity_suite
from benchmarks.suites.core_perf import CorePerfSuiteConfig, run_core_perf_suite
from benchmarks.suites.integration_check import (
    IntegrationCheckSuiteConfig,
    run_integration_check_suite,
)
from benchmarks.suites.solve_speed import SolveSpeedSuiteConfig, run_solve_speed_suite

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINES_DIR = REPO_ROOT / "benchmarks" / "baselines"
SUITE_ORDER = (
    "core_perf",
    "integration",
    "solve_speed",
    "e4",
    "f_mpc",
)
DEFAULT_BASELINE_PATHS = {
    "core_perf": BASELINES_DIR / "core_perf.json",
    "integration": BASELINES_DIR / "integration_check.json",
    "solve_speed": BASELINES_DIR / "solve_speed.json",
    "e4": BASELINES_DIR / "e4_trajopt_parity.json",
    "f_mpc": BASELINES_DIR / "f_mpc_parity.json",
}


@dataclass(frozen=True)
class SuiteSpec:
    """One regression suite: runner, defaults, and baseline path."""

    name: str
    description: str
    baseline_path: Path
    run: Callable[[bool], list[MetricRecord]]
    optional: bool = False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Benchmark regression check")
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="Override baseline JSON path (single suite only)",
    )
    parser.add_argument(
        "--suite",
        choices=(*SUITE_ORDER, "all"),
        default="core_perf",
        help="Benchmark suite to run",
    )
    parser.add_argument(
        "--factor",
        type=float,
        default=None,
        help="Speed regression factor override (default: each baseline JSON value)",
    )
    parser.add_argument(
        "--update",
        action="store_true",
        help="Refresh baseline values from the current run",
    )
    parser.add_argument(
        "--relax-accuracy",
        action="store_true",
        help="When updating, also copy accuracy tolerances from the current run",
    )
    parser.add_argument(
        "--host-hint",
        default=None,
        help="Host hint stored in baseline on --update",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write recorded metrics JSON",
    )
    parser.add_argument(
        "--tiny",
        action="store_true",
        help="Tiny workload for CI smoke (reduced NLP/trajopt iterations)",
    )
    parser.add_argument(
        "--skip-optional",
        action="store_true",
        help="Skip suites that need JAX (e4, f_mpc, integration trajopt)",
    )
    parser.add_argument(
        "--speed-gate-suffixes",
        default=None,
        help=(
            "Comma-separated metric id suffixes for enforced speed gates "
            "(e.g. solve_s,nlp_s,speedup). Other speed metrics are reported only."
        ),
    )
    parser.add_argument(
        "--no-host-context",
        action="store_true",
        help="Skip informational host speed context table after each suite",
    )
    parser.add_argument(
        "--record-host-profile",
        metavar="ID",
        default=None,
        help=(
            "After running, save speed metrics to "
            "benchmarks/host_profiles/ID.json (context only, not a gate)"
        ),
    )
    parser.add_argument(
        "--host-profile-label",
        default=None,
        help="Human label stored with --record-host-profile",
    )
    args = parser.parse_args(argv)

    speed_suffixes = _parse_suffixes(args.speed_gate_suffixes)

    suites = _suite_specs(args.tiny)
    if args.suite == "all":
        selected = tuple(suites[name] for name in SUITE_ORDER)
    else:
        selected = (suites[args.suite],)

    exit_code = 0
    for spec in selected:
        if args.skip_optional and spec.optional:
            print(f"Skipping optional suite: {spec.name}")
            continue

        baseline_path = args.baseline or spec.baseline_path
        if args.baseline is not None and len(selected) > 1:
            raise SystemExit("--baseline requires a single --suite")

        try:
            recorded = spec.run(args.tiny)
        except ModuleNotFoundError as exc:
            print(f"Skipping suite {spec.name}: {exc}", file=sys.stderr)
            continue

        if args.json_out is not None and len(selected) == 1:
            args.json_out.write_text(
                json.dumps([_metric_dict(m) for m in recorded], indent=2) + "\n",
                encoding="utf-8",
            )

        if args.update:
            baseline = _baseline_for_update(
                baseline_path,
                recorded,
                suite=spec.name,
                description=spec.description,
            )
            baseline = merge_recorded_into_baseline(
                baseline,
                recorded,
                relax_accuracy=args.relax_accuracy,
            )
            baseline.recorded_at = date.today().isoformat()
            if args.host_hint is not None:
                baseline.host_hint = args.host_hint
            elif not baseline.host_hint:
                baseline.host_hint = platform.platform()
            save_baseline(baseline_path, baseline)
            print(f"Updated baseline: {baseline_path}")
            continue

        if not baseline_path.is_file():
            print(f"Baseline not found: {baseline_path}", file=sys.stderr)
            print("Run with --update to create it.", file=sys.stderr)
            exit_code = max(exit_code, 2)
            continue

        baseline = load_baseline(baseline_path)
        if args.factor is not None:
            factor = args.factor
        else:
            factor = baseline.regression_factor
        result = compare_metrics(recorded, baseline, factor=factor)
        if speed_suffixes:
            result = filter_speed_gate_failures(result, suffixes=speed_suffixes)
        print()
        print(f"=== Suite: {spec.name} (factor={factor:g}) ===")
        print_comparison_report(result)
        if not args.no_host_context:
            print_host_speed_context(
                recorded=recorded,
                baseline_host_hint=baseline.host_hint,
            )
        if args.record_host_profile and len(selected) == 1:
            _record_host_profile_snapshot(
                args.record_host_profile,
                label=args.host_profile_label,
                recorded=recorded,
                workload="tiny" if args.tiny else "default",
            )
        if result.failed:
            exit_code = 1

    return exit_code


def _suite_specs(tiny: bool) -> dict[str, SuiteSpec]:
    return {
        "core_perf": SuiteSpec(
            name="core_perf",
            description="Fast regression: compile speed ratios + integration accuracy ceilings.",
            baseline_path=DEFAULT_BASELINE_PATHS["core_perf"],
            run=lambda _tiny=tiny: run_core_perf_suite(
                CorePerfSuiteConfig(
                    pendulum_n_calls=2 if _tiny else 2000,
                    diagram_n_calls=2 if _tiny else 500,
                    sim_n_runs=1 if _tiny else 2,
                    static_n_steps=5 if _tiny else 200,
                )
            ),
        ),
        "integration": SuiteSpec(
            name="integration_check",
            description=(
                "End-to-end regression: trajectory checkpoint goldens and solve-time gates."
            ),
            baseline_path=DEFAULT_BASELINE_PATHS["integration"],
            optional=True,
            run=lambda _tiny=tiny: run_integration_check_suite(
                IntegrationCheckSuiteConfig(
                    sim_n_runs=1 if _tiny else 2,
                    trajopt_n_runs=1,
                    tiny=_tiny,
                )
            ),
        ),
        "solve_speed": SuiteSpec(
            name="solve_speed",
            description=(
                "NLP and trajopt solve wall-time gates (Optimizer + NumPy pendulum trajopt)."
            ),
            baseline_path=DEFAULT_BASELINE_PATHS["solve_speed"],
            run=lambda _tiny=tiny: run_solve_speed_suite(
                SolveSpeedSuiteConfig(
                    n_runs=1 if _tiny else 2,
                    tiny=_tiny,
                )
            ),
        ),
        "e4": SuiteSpec(
            name="e4_trajopt_parity",
            description=(
                "E4 trajopt parity: TOP rebuild + MPC parametric (JAX + SciPy SLSQP)."
            ),
            baseline_path=DEFAULT_BASELINE_PATHS["e4"],
            optional=True,
            run=lambda _tiny=tiny: run_e4_trajopt_parity_suite(
                E4TrajoptParityConfig(n_runs=1, tiny=_tiny)
            ),
        ),
        "f_mpc": SuiteSpec(
            name="f_mpc_parity",
            description="F MPC parity: hybrid ZOH, hand-loop, dual-rate.",
            baseline_path=DEFAULT_BASELINE_PATHS["f_mpc"],
            optional=True,
            run=lambda _tiny=tiny: run_f_mpc_parity_suite(),
        ),
    }


def _baseline_for_update(
    path: Path,
    recorded: list[MetricRecord],
    *,
    suite: str,
    description: str,
) -> BaselineFile:
    if path.is_file():
        return load_baseline(path)
    default_factor = default_regression_factor()
    if suite == "f_mpc_parity":
        default_factor = 2.0
    return BaselineFile(
        schema_version=1,
        suite=suite,
        description=description,
        regression_factor=default_factor,
        recorded_at="",
        host_hint="",
        metrics=tuple(recorded),
    )


def _metric_dict(metric: MetricRecord) -> dict:
    payload = {
        "id": metric.id,
        "gate": metric.gate,
        "direction": metric.direction,
        "value": metric.value,
        "unit": metric.unit,
    }
    if metric.max_allowed is not None:
        payload["max_allowed"] = metric.max_allowed
    if metric.atol is not None:
        payload["atol"] = metric.atol
    if metric.rtol is not None:
        payload["rtol"] = metric.rtol
    if metric.notes:
        payload["notes"] = metric.notes
    return payload


def _parse_suffixes(raw: str | None) -> tuple[str, ...]:
    if raw is None or not raw.strip():
        return ()
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def _record_host_profile_snapshot(
    profile_id: str,
    *,
    label: str | None,
    recorded: list[MetricRecord],
    workload: str,
) -> None:
    path = PROFILES_DIR / f"{profile_id}.json"
    save_host_profile(
        path,
        profile_id=profile_id,
        label=label or profile_id,
        recorded=recorded,
        workload=workload,
        notes="Recorded from run_regression_check.py --record-host-profile",
    )
    print(f"Recorded host profile: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
