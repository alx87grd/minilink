"""Run example scripts as subprocess demo checks (nightly / local sweep).

Executes each script's ``__main__`` unchanged (no demo hooks required).
For the smaller flagship whitelist, prefer ``run_flagship_demos.py``.

Usage (from repo root)::

    python tests/demo_checks/run_all_demos.py --help
    python tests/demo_checks/run_all_demos.py --timeout 60 --continue-on-error
    python tests/demo_checks/run_all_demos.py --flagship-scripts --continue-on-error
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
CHECKS_DIR = Path(__file__).resolve().parent
SCRIPTS_ROOT = REPO_ROOT / "examples" / "scripts"
FLAGSHIP_MANIFEST = CHECKS_DIR / "flagship_manifest.json"

# Live keyboard / wall-clock sessions hang until the user quits — not suitable
# for an unattended subprocess sweep. Keep discoverable under examples/; skip here.
INTERACTIVE_DEMO_PREFIXES = ("examples/scripts/realtime/",)
INTERACTIVE_DEMO_PATHS = frozenset(
    {
        "examples/scripts/animation/game_bicycle.py",
    }
)


@dataclass(frozen=True)
class DemoRunRow:
    path: str
    status: str
    message: str = ""


def _is_interactive_demo(path: Path) -> bool:
    rel = path.relative_to(REPO_ROOT).as_posix()
    if any(rel.startswith(prefix) for prefix in INTERACTIVE_DEMO_PREFIXES):
        return True
    return rel in INTERACTIVE_DEMO_PATHS


def _discover_scripts() -> list[Path]:
    return sorted(path for path in SCRIPTS_ROOT.rglob("*.py") if path.is_file())


def _flagship_paths() -> list[Path]:
    entries = json.loads(FLAGSHIP_MANIFEST.read_text(encoding="utf-8"))
    return [REPO_ROOT / entry["path"] for entry in entries]


def _run_script(path: Path, *, timeout: float) -> DemoRunRow:
    rel = path.relative_to(REPO_ROOT)
    if _is_interactive_demo(path):
        return DemoRunRow(str(rel), "skip", "interactive realtime session")
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
    flagship_scripts: bool,
) -> list[DemoRunRow]:
    paths = _flagship_paths() if flagship_scripts else _discover_scripts()
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
    skipped = sum(row.status == "skip" for row in rows)
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped, {len(rows)} total")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Subprocess demo checks: run example scripts via __main__ "
            "(nightly / local). Prefer tests/demo_checks/run_flagship_demos.py "
            "for the flagship whitelist."
        )
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
        "--flagship-scripts",
        action="store_true",
        help=(
            "Only paths in flagship_manifest.json. "
            "Prefer run_flagship_demos.py for the usual flagship gate."
        ),
    )
    parser.add_argument(
        "--flagship-only",
        action="store_true",
        help=argparse.SUPPRESS,  # deprecated alias for --flagship-scripts
    )
    args = parser.parse_args(argv)
    rows = run_all_demos(
        timeout=args.timeout,
        continue_on_error=args.continue_on_error,
        flagship_scripts=args.flagship_scripts or args.flagship_only,
    )
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
