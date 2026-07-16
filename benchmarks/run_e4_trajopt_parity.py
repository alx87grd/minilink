"""E4 trajopt regression parity harness (TOP rebuild + MPC parametric).

Usage (from repo root)::

    python benchmarks/run_e4_trajopt_parity.py --capture   # write baselines/e4_trajopt_parity.json
    python benchmarks/run_e4_trajopt_parity.py             # compare vs baseline, exit 1 on fail
"""

from __future__ import annotations

import argparse
import platform
import sys
from datetime import date
from pathlib import Path

from benchmarks.baseline import (
    BaselineFile,
    compare_metrics,
    load_baseline,
    print_comparison_report,
    save_baseline,
)
from benchmarks.scenarios.e4_trajopt_parity import run_e4_trajopt_parity_suite

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = REPO_ROOT / "benchmarks" / "baselines" / "e4_trajopt_parity.json"
SUITE_NAME = "e4_trajopt_parity"
SUITE_DESCRIPTION = (
    "E4 trajopt parity: cartpole/pendulum TOP rebuild + bicycle TOP parametric."
)
DEFAULT_FACTOR = 4.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="E4 trajopt regression parity check")
    parser.add_argument(
        "--capture",
        action="store_true",
        help="Write baselines/e4_trajopt_parity.json from the current run",
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=BASELINE_PATH,
        help="Baseline JSON path",
    )
    parser.add_argument(
        "--factor",
        type=float,
        default=None,
        help="Speed regression factor override (default 4.0)",
    )
    parser.add_argument(
        "--host-hint",
        default=None,
        help="Host hint stored in baseline on --capture",
    )
    args = parser.parse_args(argv)

    recorded = run_e4_trajopt_parity_suite()

    if args.capture:
        baseline = BaselineFile(
            schema_version=1,
            suite=SUITE_NAME,
            description=SUITE_DESCRIPTION,
            regression_factor=(
                DEFAULT_FACTOR if args.factor is None else float(args.factor)
            ),
            recorded_at=date.today().isoformat(),
            host_hint=args.host_hint or platform.platform(),
            metrics=tuple(recorded),
        )
        save_baseline(args.baseline, baseline)
        print(f"Captured baseline: {args.baseline}")
        print(f"  metrics: {len(recorded)}")
        return 0

    if not args.baseline.is_file():
        print(f"Baseline not found: {args.baseline}", file=sys.stderr)
        print("Run with --capture to create it.", file=sys.stderr)
        return 2

    baseline = load_baseline(args.baseline)
    factor = baseline.regression_factor if args.factor is None else float(args.factor)
    result = compare_metrics(recorded, baseline, factor=factor)
    print()
    print(f"=== Suite: {SUITE_NAME} ===")
    print_comparison_report(result)
    return 1 if result.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
