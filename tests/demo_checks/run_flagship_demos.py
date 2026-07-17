"""Run flagship demo scripts via subprocess (demo-check layer).

Runs each manifest entry's ``__main__`` unchanged — demos do **not** need a
``run_smoke()`` hook. Uses ``MPLBACKEND=Agg`` so matplotlib stays headless.

For a full sweep of every example script, use ``run_all_demos.py``.

Usage (from repo root)::

    python tests/demo_checks/run_flagship_demos.py
    python tests/demo_checks/run_flagship_demos.py --demo mpc_minimal
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_CHECKS_DIR = Path(__file__).resolve().parent
MANIFEST_PATH = _CHECKS_DIR / "flagship_manifest.json"


@dataclass(frozen=True)
class DemoRow:
    demo_id: str
    status: str
    message: str = ""


def _load_manifest() -> list[dict]:
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def _run_script(path: Path, *, timeout: float) -> tuple[str, str]:
    """Execute ``path`` as a script; return (status, message)."""
    env = {
        **os.environ,
        "PYTHONPATH": str(REPO_ROOT),
        "MPLBACKEND": "Agg",
        "SDL_VIDEODRIVER": "dummy",
    }
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
        return "fail", f"timeout after {timeout:.0f}s"
    except OSError as exc:
        return "fail", str(exc)

    if proc.returncode == 0:
        return "pass", ""
    tail = (proc.stderr or proc.stdout or "").strip()[-400:]
    return "fail", f"exit {proc.returncode}: {tail}"


def run_flagship_demos(
    *,
    demo_filter: str | None = None,
    timeout: float = 120.0,
) -> list[DemoRow]:
    rows: list[DemoRow] = []
    for entry in _load_manifest():
        demo_id = entry["id"]
        if demo_filter is not None and demo_id != demo_filter:
            continue
        requires = tuple(entry.get("requires") or ())
        missing = False
        for extra in requires:
            if importlib.util.find_spec(extra) is None:
                rows.append(DemoRow(demo_id, "skip", f"missing {extra}"))
                missing = True
                break
        if missing:
            continue

        path = REPO_ROOT / entry["path"]
        if not path.is_file():
            rows.append(DemoRow(demo_id, "fail", f"missing file {entry['path']}"))
            continue

        status, message = _run_script(path, timeout=timeout)
        rows.append(DemoRow(demo_id, status, message))
    return rows


def _print_report(rows: list[DemoRow]) -> int:
    width = max((len(row.demo_id) for row in rows), default=4)
    for row in rows:
        note = f"  ({row.message})" if row.message else ""
        print(f"  {row.demo_id:<{width}}  {row.status}{note}")
    failed = sum(row.status == "fail" for row in rows)
    skipped = sum(row.status == "skip" for row in rows)
    passed = sum(row.status == "pass" for row in rows)
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Flagship demo checks: subprocess each script's __main__ "
            "(no demo source changes required)"
        )
    )
    parser.add_argument("--demo", default=None, help="Run one manifest id")
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Per-script timeout seconds (default 120)",
    )
    args = parser.parse_args(argv)
    rows = run_flagship_demos(demo_filter=args.demo, timeout=args.timeout)
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
