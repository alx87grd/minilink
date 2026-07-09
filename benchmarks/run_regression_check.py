"""Run core perf regression checks against committed baselines.

Usage (from repo root)::

    python benchmarks/run_regression_check.py
    python benchmarks/run_regression_check.py --update
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
from datetime import date
from pathlib import Path

from benchmarks.baseline import (
    BaselineFile,
    compare_metrics,
    default_regression_factor,
    load_baseline,
    merge_recorded_into_baseline,
    print_comparison_report,
    save_baseline,
)
from benchmarks.suites.core_perf import CorePerfSuiteConfig, run_core_perf_suite

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE_PATH = REPO_ROOT / "benchmarks" / "baselines" / "core_perf.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Benchmark regression check")
    parser.add_argument(
        "--baseline",
        type=Path,
        default=DEFAULT_BASELINE_PATH,
        help="Path to baseline JSON",
    )
    parser.add_argument(
        "--suite",
        choices=("core_perf",),
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

    if args.suite != "core_perf":
        raise SystemExit(f"Unknown suite {args.suite!r}")

    config = CorePerfSuiteConfig(
        pendulum_n_calls=2 if args.tiny else 2000,
        diagram_n_calls=2 if args.tiny else 500,
        sim_n_runs=1 if args.tiny else 2,
        static_n_steps=5 if args.tiny else 200,
    )

    recorded = run_core_perf_suite(config)
    if args.json_out is not None:
        args.json_out.write_text(
            json.dumps([_metric_dict(m) for m in recorded], indent=2) + "\n",
            encoding="utf-8",
        )

    if args.update:
        baseline = _baseline_for_update(args.baseline, recorded)
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
        save_baseline(args.baseline, baseline)
        print(f"Updated baseline: {args.baseline}")
        return 0

    if not args.baseline.is_file():
        print(f"Baseline not found: {args.baseline}", file=sys.stderr)
        print("Run with --update to create it.", file=sys.stderr)
        return 2

    baseline = load_baseline(args.baseline)
    factor = default_regression_factor() if args.factor is None else args.factor
    result = compare_metrics(recorded, baseline, factor=factor)
    print_comparison_report(result)
    return 1 if result.failed else 0


def _baseline_for_update(path: Path, recorded: list) -> BaselineFile:
    if path.is_file():
        return load_baseline(path)
    return BaselineFile(
        schema_version=1,
        suite="core_perf",
        description="Fast regression: speed ratios + integration accuracy ceilings.",
        regression_factor=default_regression_factor(),
        recorded_at="",
        host_hint="",
        metrics=tuple(recorded),
    )


def _metric_dict(metric) -> dict:
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
