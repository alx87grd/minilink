"""v0.1 wheel: hatch config, research lane out, teaching surface in."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CHECK_WHEEL = REPO / "tests" / "demo_checks" / "check_wheel.py"


class TestWheelConfig(unittest.TestCase):
    def test_wheel_and_sdist_exclude_research_lane(self):
        text = (REPO / "pyproject.toml").read_text()
        self.assertIn('"minilink/experimental/**"', text)
        self.assertIn('"/minilink/experimental"', text)
        self.assertIn('"/examples/experimental"', text)
        self.assertIn('"/examples/projects"', text)

    def test_full_extra_is_declared(self):
        text = (REPO / "pyproject.toml").read_text()
        self.assertIn("full = [", text)
        self.assertIn("minilink[diagrams,visualization,plotting,symbolic,jax,rl]", text)


class TestBuiltWheel(unittest.TestCase):
    def test_wheel_and_sdist_omit_research_lane(self):
        try:
            import build  # noqa: F401
        except ImportError:
            self.skipTest("build is not installed")

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            subprocess.check_call(
                [sys.executable, "-m", "build", "--outdir", str(out)],
                cwd=REPO,
            )
            wheels = list(out.glob("*.whl"))
            sdists = list(out.glob("*.tar.gz"))
            self.assertEqual(len(wheels), 1, wheels)
            self.assertEqual(len(sdists), 1, sdists)
            subprocess.check_call(
                [sys.executable, str(CHECK_WHEEL), *map(str, wheels + sdists)],
            )
