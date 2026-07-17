"""Demo-check runner subprocess tests (catalog + flagship demos + graphics).

Thin CI bridge: invokes ``tests/demo_checks/`` runners; does not duplicate their
assertions. Without JAX, JAX-required flagships skip here — the CI
``regression`` job (``.github/workflows/test.yml``) re-runs
``run_flagship_demos.py`` with JAX installed.
"""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


class TestDemoCheckRunners(unittest.TestCase):
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

    def test_catalog_checks_fast_exit_zero(self):
        proc = self._run(
            "tests/demo_checks/run_catalog_checks.py",
            "--fast",
        )
        if proc.returncode != 0:
            self.fail(
                f"catalog checks failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )

    def test_flagship_demos_exit_zero(self):
        proc = self._run("tests/demo_checks/run_flagship_demos.py")
        if proc.returncode != 0:
            self.fail(
                f"flagship demos failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )

    def test_flagship_graphics_exit_zero(self):
        proc = self._run("tests/demo_checks/run_flagship_graphics.py")
        if proc.returncode != 0:
            self.fail(
                f"flagship graphics failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )

    def test_run_study_list_exit_zero(self):
        proc = self._run("benchmarks/run_study.py", "--list")
        if proc.returncode != 0:
            self.fail(
                f"run_study --list failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )
        self.assertIn("f_eval", proc.stdout)


if __name__ == "__main__":
    unittest.main()
