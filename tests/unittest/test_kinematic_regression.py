"""Kinematic rendering smoke over the baseline manifest states.

Exercises each catalog plant and pose listed in
``tests/fixtures/kinematic_baseline/manifest.json``: render via the fixture
helper, assert a finite image is produced. PNGs are not stored in the repo;
regenerate the manifest with
``python tests/fixtures/kinematic_baseline/regenerate_manifest.py``.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.image as mpimg  # noqa: E402
import numpy as np  # noqa: E402

from tests.fixtures.kinematic_baseline.render import render_baseline_png

FIXTURE_DIR = Path(__file__).resolve().parents[1] / "fixtures" / "kinematic_baseline"
MANIFEST_PATH = FIXTURE_DIR / "manifest.json"


class TestKinematicRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MANIFEST_PATH.exists():
            raise unittest.SkipTest("baseline manifest missing")
        cls.manifest = json.loads(MANIFEST_PATH.read_text())

    def test_manifest_renders_finite_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            for entry in self.manifest:
                with self.subTest(plant=entry["plant"], sample=entry["sample"]):
                    for extra in entry.get("requires", []):
                        if importlib.util.find_spec(extra) is None:
                            self.skipTest(f"missing extra: {extra}")

                    module = importlib.import_module(entry["module"])
                    sys = getattr(module, entry["class"])()
                    x = np.asarray(entry["x"], dtype=float)
                    u = np.asarray(entry["u"], dtype=float)
                    t = entry["t"]

                    plant = entry["plant"]
                    sample = entry["sample"]
                    out = tmp_dir / f"{plant}__{sample}.png"
                    render_baseline_png(sys, x, u, t, out)

                    img = mpimg.imread(out)
                    self.assertEqual(img.ndim, 3)
                    self.assertGreater(img.shape[0], 0)
                    self.assertGreater(img.shape[1], 0)
                    self.assertTrue(np.all(np.isfinite(img)))


if __name__ == "__main__":
    unittest.main()
