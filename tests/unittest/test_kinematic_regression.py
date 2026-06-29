"""Kinematic rendering smoke over the baseline manifest states.

Exercises each catalog plant and pose listed in
``tests/fixtures/kinematic_baseline/manifest.json``: render via the baseline
generator, assert a finite image is produced. Pixel PNGs are **not** stored in
the repo; for local visual review run
``python scripts/generate_kinematic_baselines.py`` (writes to the fixture dir,
gitignored ``*.png``).
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.image as mpimg  # noqa: E402  (after backend selection)
import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "kinematic_baseline"
MANIFEST_PATH = FIXTURE_DIR / "manifest.json"
GENERATOR_PATH = REPO_ROOT / "scripts" / "generate_kinematic_baselines.py"


def _load_generator():
    """Import the baseline generator by file path (single source of render logic)."""
    spec = importlib.util.spec_from_file_location(
        "generate_kinematic_baselines", GENERATOR_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestKinematicRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MANIFEST_PATH.exists():
            raise unittest.SkipTest("baseline manifest missing")
        cls.baseline = _load_generator()
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

                    out = tmp_dir / entry["file"]
                    self.baseline.render_baseline_png(sys, x, u, t, out)

                    img = mpimg.imread(out)
                    self.assertEqual(img.ndim, 3)
                    self.assertGreater(img.shape[0], 0)
                    self.assertGreater(img.shape[1], 0)
                    self.assertTrue(np.all(np.isfinite(img)))


if __name__ == "__main__":
    unittest.main()
