"""Run perf and integration regression checks against committed baselines.

Usage (from repo root)::

    python benchmarks/run_regression_check.py
    python benchmarks/run_regression_check.py --suite integration
    python benchmarks/run_regression_check.py --suite all
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
    load_baseline,
    merge_recorded_into_baseline,
    print_comparison_report,
    save_baseline,
)
from benchmarks.suites.core_perf import CorePerfSuiteConfig, run_core_perf_suite
from benchmarks.suites.integration_check import (
    IntegrationCheckSuiteConfig,
    run_integration_check_suite,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINES_DIR = REPO_ROOT / "benchmarks" / "baselines"
DEFAULT_BASELINE_PATHS = {
    "core_perf": BASELINES_DIR / "core_perf.json",
    "integration": BASELINES_DIR / "integration_check.json",
}


@dataclass(frozen=True)
class SuiteSpec:
    """One regression suite: runner, defaults, and baseline path."""

    name: str
    description: str
    baseline_path: Path
    run: Callable[[bool], list[MetricRecord]]


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
        choices=("core_perf", "integration", "all"),
        default="core_perf",
        help="Benchmark suite to run",
    )
    parser.add_argument(
        "--factor",
        type=float,
        default=None,
        help="Speed regression factor override",
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
        help="Tiny workload for smoke tests",
    )
    args = parser.parse_args(argv)

    suites = _suite_specs(args.tiny)
    if args.suite == "all":
        selected = suites
    else:
        selected = (suites[args.suite],)

    exit_code = 0
    for spec in selected:
        baseline_path = args.baseline or spec.baseline_path
        if args.baseline is not None and len(selected) > 1:
            raise SystemExit("--baseline requires a single --suite")

        recorded = spec.run(args.tiny)
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
        factor = default_regression_factor() if args.factor is None else args.factor
        result = compare_metrics(recorded, baseline, factor=factor)
        print()
        print(f"=== Suite: {spec.name} ===")
        print_comparison_report(result)
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
            run=lambda _tiny=tiny: run_integration_check_suite(
                IntegrationCheckSuiteConfig(
                    sim_n_runs=1 if _tiny else 2,
                    trajopt_n_runs=1,
                )
            ),
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
    return BaselineFile(
        schema_version=1,
        suite=suite,
        description=description,
        regression_factor=default_regression_factor(),
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


if __name__ == "__main__":
    raise SystemExit(main())
