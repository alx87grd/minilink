"""L6 smoke runner subprocess tests (catalog + flagship demos)."""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


class TestSmokeRunners(unittest.TestCase):
    def _run(self, script: str, *args: str) -> subprocess.CompletedProcess[str]:
        path = REPO_ROOT / script
        env = {**os.environ, "PYTHONPATH": str(REPO_ROOT)}
        return subprocess.run(
            [sys.executable, str(path), *args],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )

    def test_catalog_smokes_fast_exit_zero(self):
        proc = self._run(
            "examples/scripts/_smoke/run_catalog_smokes.py",
            "--fast",
        )
        if proc.returncode != 0:
            self.fail(
                f"catalog smokes failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )

    def test_flagship_demos_exit_zero(self):
        proc = self._run("examples/scripts/_smoke/run_flagship_demos.py")
        if proc.returncode != 0:
            self.fail(
                f"flagship demos failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )


if __name__ == "__main__":
    unittest.main()
