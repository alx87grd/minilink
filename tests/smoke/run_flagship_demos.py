"""Run flagship demo ``run_smoke()`` entry points (L6).

Usage (from repo root)::

    python tests/smoke/run_flagship_demos.py
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_SMOKE_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(_SMOKE_DIR) not in sys.path:
    sys.path.insert(0, str(_SMOKE_DIR))

import helpers as _helpers  # noqa: E402

configure_headless = _helpers.configure_headless
MANIFEST_PATH = _SMOKE_DIR / "flagship_manifest.json"


def _load_demo_module(relative_path: str):
    path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(f"demo_{path.stem}", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load demo module from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class DemoRow:
    demo_id: str
    status: str
    message: str = ""


def _load_manifest() -> list[dict]:
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def run_flagship_demos(*, demo_filter: str | None = None) -> list[DemoRow]:
    configure_headless()
    rows: list[DemoRow] = []
    for entry in _load_manifest():
        demo_id = entry["id"]
        if demo_filter is not None and demo_id != demo_filter:
            continue
        requires = tuple(entry.get("requires") or ())
        for extra in requires:
            if importlib.util.find_spec(extra) is None:
                rows.append(DemoRow(demo_id, "skip", f"missing {extra}"))
                break
        else:
            try:
                module = _load_demo_module(entry["path"])
                run_smoke = getattr(module, "run_smoke", None)
                if run_smoke is None:
                    raise AttributeError(f"{entry['path']} has no run_smoke()")
                run_smoke()
            except Exception as exc:  # noqa: BLE001
                rows.append(DemoRow(demo_id, "fail", str(exc)))
            else:
                rows.append(DemoRow(demo_id, "pass"))
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
    parser = argparse.ArgumentParser(description="Flagship demo smoke runner (L6)")
    parser.add_argument("--demo", default=None, help="Run one manifest id")
    args = parser.parse_args(argv)
    rows = run_flagship_demos(demo_filter=args.demo)
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
