"""Run all example scripts as subprocess smokes (L6 nightly / local sweep).

Usage (from repo root)::

    python tests/smoke/run_all_demos.py --help
    python tests/smoke/run_all_demos.py --timeout 60 --continue-on-error
    python tests/smoke/run_all_demos.py --flagship-only
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SMOKE_DIR = Path(__file__).resolve().parent
SCRIPTS_ROOT = REPO_ROOT / "examples" / "scripts"
FLAGSHIP_MANIFEST = SMOKE_DIR / "flagship_manifest.json"


@dataclass(frozen=True)
class DemoRunRow:
    path: str
    status: str
    message: str = ""


def _discover_scripts() -> list[Path]:
    paths: list[Path] = []
    for path in sorted(SCRIPTS_ROOT.rglob("*.py")):
        if path.is_relative_to(SMOKE_DIR):
            continue
        paths.append(path)
    return paths


def _flagship_paths() -> list[Path]:
    entries = json.loads(FLAGSHIP_MANIFEST.read_text(encoding="utf-8"))
    return [REPO_ROOT / entry["path"] for entry in entries]


def _run_script(path: Path, *, timeout: float) -> DemoRunRow:
    rel = path.relative_to(REPO_ROOT)
    env = {**os.environ, "PYTHONPATH": str(REPO_ROOT), "MPLBACKEND": "Agg"}
    try:
        proc = subprocess.run(
            [sys.executable, str(path)],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return DemoRunRow(str(rel), "fail", f"timeout after {timeout:.0f}s")
    except OSError as exc:
        return DemoRunRow(str(rel), "fail", str(exc))

    if proc.returncode == 0:
        return DemoRunRow(str(rel), "pass")
    tail = (proc.stderr or proc.stdout or "").strip()[-400:]
    return DemoRunRow(str(rel), "fail", f"exit {proc.returncode}: {tail}")


def run_all_demos(
    *,
    timeout: float,
    continue_on_error: bool,
    flagship_only: bool,
) -> list[DemoRunRow]:
    paths = _flagship_paths() if flagship_only else _discover_scripts()
    rows: list[DemoRunRow] = []
    for path in paths:
        row = _run_script(path, timeout=timeout)
        rows.append(row)
        if row.status == "fail" and not continue_on_error:
            break
    return rows


def _print_report(rows: list[DemoRunRow]) -> int:
    width = max((len(row.path) for row in rows), default=4)
    for row in rows:
        note = f"  ({row.message})" if row.message else ""
        print(f"  {row.path:<{width}}  {row.status}{note}")
    failed = sum(row.status == "fail" for row in rows)
    passed = sum(row.status == "pass" for row in rows)
    print(f"\n{passed} passed, {failed} failed, {len(rows)} total")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Subprocess smoke all example scripts (L6)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Per-script timeout seconds (default 120)",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Keep running after a failure",
    )
    parser.add_argument(
        "--flagship-only",
        action="store_true",
        help="Only run paths listed in flagship_manifest.json",
    )
    args = parser.parse_args(argv)
    rows = run_all_demos(
        timeout=args.timeout,
        continue_on_error=args.continue_on_error,
        flagship_only=args.flagship_only,
    )
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
