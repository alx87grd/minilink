"""G0 flagship graphics draw-list contract (Layer L4).

Each manifest entry maps a README-tier demo to a canonical plant pose. Asserts
finite kinematic transforms and expected primitive counts — no pixel I/O.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np

from tests.unittest.graphics_contract_helpers import geometry_smoke, resolve_draw_frame

REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = REPO_ROOT / "tests" / "fixtures" / "flagship_graphics" / "manifest.json"


def _load_manifest() -> list[dict]:
    if not MANIFEST_PATH.is_file():
        raise FileNotFoundError(f"missing manifest: {MANIFEST_PATH}")
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def _load_plant(entry: dict):
    for extra in entry.get("requires") or []:
        if importlib.util.find_spec(extra) is None:
            return None
    module = importlib.import_module(entry["module"])
    plant = getattr(module, entry["class"])()
    params = entry.get("params") or {}
    if params:
        plant.params.update(params)
    return plant


class TestFlagshipGraphicsContract(unittest.TestCase):
    def test_manifest_nonempty(self):
        manifest = _load_manifest()
        self.assertGreaterEqual(len(manifest), 6)

    def test_kinematic_draw_list_contract(self):
        manifest = _load_manifest()
        for entry in manifest:
            with self.subTest(entry=entry["id"]):
                for extra in entry.get("requires") or []:
                    if importlib.util.find_spec(extra) is None:
                        self.skipTest(f"{extra} not installed")
                plant = _load_plant(entry)
                self.assertIsNotNone(plant)
                x = np.asarray(entry["x"], dtype=float)
                u = np.asarray(entry["u"], dtype=float)
                t = float(entry["t"])
                geometry_smoke(plant, x=x, u=u, t=t)
                frame = resolve_draw_frame(plant, x=x, u=u, t=t)
                min_primitives = int(entry.get("min_primitives", 1))
                self.assertGreaterEqual(len(frame["primitives"]), min_primitives)


if __name__ == "__main__":
    unittest.main()
