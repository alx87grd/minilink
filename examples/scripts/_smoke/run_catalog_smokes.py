"""Run headless catalog plant smokes (L6).

Usage (from repo root)::

    python examples/scripts/_smoke/run_catalog_smokes.py
    python examples/scripts/_smoke/run_catalog_smokes.py --fast
    python examples/scripts/_smoke/run_catalog_smokes.py --plant Pendulum
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
_SMOKE_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(_SMOKE_DIR) not in sys.path:
    sys.path.insert(0, str(_SMOKE_DIR))

import helpers as _helpers  # noqa: E402

configure_headless = _helpers.configure_headless
smoke_catalog_plant = _helpers.smoke_catalog_plant
from minilink.dynamics.catalog.smoke_registry import (  # noqa: E402
    CATALOG_SMOKE_ENTRIES,
)


@dataclass(frozen=True)
class SmokeRow:
    entry_id: str
    status: str
    message: str = ""


def _have_jax() -> bool:
    return importlib.util.find_spec("jax") is not None


def run_catalog_smokes(
    *,
    fast: bool,
    plant_filter: str | None,
) -> list[SmokeRow]:
    configure_headless()
    rows: list[SmokeRow] = []
    jax_ok = _have_jax()

    for entry in CATALOG_SMOKE_ENTRIES:
        if plant_filter is not None and entry.id != plant_filter:
            continue
        if entry.requires_jax and not jax_ok:
            rows.append(SmokeRow(entry.id, "skip", "jax not installed"))
            continue
        try:
            system = entry.factory()
            smoke_catalog_plant(system, fast=fast)
        except Exception as exc:  # noqa: BLE001 — smoke aggregator reports all failures
            rows.append(SmokeRow(entry.id, "fail", str(exc)))
        else:
            rows.append(SmokeRow(entry.id, "pass"))
    return rows


def _print_report(rows: list[SmokeRow]) -> int:
    width = max((len(row.entry_id) for row in rows), default=6)
    for row in rows:
        note = f"  ({row.message})" if row.message else ""
        print(f"  {row.entry_id:<{width}}  {row.status}{note}")
    failed = sum(row.status == "fail" for row in rows)
    skipped = sum(row.status == "skip" for row in rows)
    passed = sum(row.status == "pass" for row in rows)
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Catalog plant smoke runner (L6)")
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Skip short simulation step (f + geometry only)",
    )
    parser.add_argument(
        "--plant",
        default=None,
        help="Run one catalog entry id (e.g. Pendulum)",
    )
    args = parser.parse_args(argv)
    rows = run_catalog_smokes(fast=args.fast, plant_filter=args.plant)
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
