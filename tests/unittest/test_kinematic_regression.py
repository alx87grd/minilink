"""Pixel-parity regression for the kinematic rendering pipeline.

Re-renders each committed baseline state through the current graphics pipeline and
compares pixel-for-pixel against the PNG checked into
``tests/fixtures/kinematic_baseline/``. Any drift in geometry, transforms, or
camera shows up as a pixel diff.

Regenerate the baselines with ``python scripts/generate_kinematic_baselines.py``
only on a deliberate, reviewed visual change.
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
    # Register before exec so dataclasses can resolve string annotations
    # (`from __future__ import annotations`) against the module namespace.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestKinematicRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MANIFEST_PATH.exists():
            raise unittest.SkipTest(
                "baseline manifest missing; run "
                "`python scripts/generate_kinematic_baselines.py` first"
            )
        cls.baseline = _load_generator()
        cls.manifest = json.loads(MANIFEST_PATH.read_text())

    def test_all_baselines_pixel_identical(self):
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

                    candidate = tmp_dir / entry["file"]
                    self.baseline.render_baseline_png(sys, x, u, t, candidate)

                    expected = mpimg.imread(FIXTURE_DIR / entry["file"])
                    actual = mpimg.imread(candidate)

                    self.assertEqual(
                        actual.shape,
                        expected.shape,
                        msg=f"image size drift for {entry['file']}",
                    )
                    if not np.array_equal(actual, expected):
                        diff = np.abs(actual.astype(float) - expected.astype(float))
                        n_diff = int(np.count_nonzero(diff.any(axis=-1)))
                        self.fail(
                            f"pixel drift for {entry['file']}: "
                            f"{n_diff} pixels differ, max channel diff {diff.max():.4f}"
                        )


if __name__ == "__main__":
    unittest.main()
