"""Headless flagship graphics contract check: kinematic PNGs from manifest.

Usage (from repo root)::

    python tests/demo_checks/run_flagship_graphics.py
    python tests/demo_checks/run_flagship_graphics.py --out /tmp/ml-gfx
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.image as mpimg  # noqa: E402
import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.fixtures.kinematic_baseline.render import render_baseline_png  # noqa: E402

MANIFEST_PATH = REPO_ROOT / "tests/fixtures/kinematic_baseline/manifest.json"


def _load_system(entry: dict):
    for extra in entry.get("requires") or []:
        if importlib.util.find_spec(extra) is None:
            return None
    module = importlib.import_module(entry["module"])
    return getattr(module, entry["class"])()


def run_flagship_graphics(*, out_dir: Path) -> list[tuple[str, str]]:
    """Render manifest entries; return (name, status) rows."""
    if not MANIFEST_PATH.is_file():
        raise FileNotFoundError(f"missing manifest: {MANIFEST_PATH}")
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: list[tuple[str, str]] = []

    for entry in manifest:
        name = f"{entry['plant']}__{entry['sample']}"
        sys_obj = _load_system(entry)
        if sys_obj is None:
            rows.append((name, "skip"))
            continue
        try:
            x = np.asarray(entry["x"], dtype=float)
            u = np.asarray(entry["u"], dtype=float)
            t = float(entry["t"])
            png = out_dir / f"{name}.png"
            render_baseline_png(sys_obj, x, u, t, png)
            img = mpimg.imread(png)
            if img.ndim != 3 or not np.all(np.isfinite(img)):
                raise ValueError("non-finite image")
            if float(np.max(img)) - float(np.min(img)) < 1e-6:
                raise ValueError("blank image")
        except Exception as exc:  # noqa: BLE001
            rows.append((name, f"fail: {exc}"))
        else:
            rows.append((name, "pass"))
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Flagship graphics headless contract check"
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory for PNGs (default: temp dir)",
    )
    args = parser.parse_args(argv)
    if args.out is None:
        with tempfile.TemporaryDirectory(prefix="minilink-gfx-") as tmp:
            rows = run_flagship_graphics(out_dir=Path(tmp))
            failed = [name for name, status in rows if status.startswith("fail")]
            for name, status in rows:
                print(f"  {name}: {status}")
            return 1 if failed else 0

    rows = run_flagship_graphics(out_dir=args.out)
    failed = [name for name, status in rows if status.startswith("fail")]
    for name, status in rows:
        print(f"  {name}: {status}")
    if not failed:
        print(f"\nPNGs written under {args.out}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
