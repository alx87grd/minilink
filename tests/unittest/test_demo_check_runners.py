"""Demo-check runner subprocess tests (catalog + flagship demos + graphics).

Thin CI bridge: invokes ``tests/demo_checks/`` runners; does not duplicate their
assertions. Without JAX, JAX-required flagships skip here — the CI
``regression`` job (``.github/workflows/test.yml``) re-runs
``run_flagship_demos.py`` with JAX installed.

Notebook smoke checks run in the CI ``regression`` job (and via
``tests/run/run_notebook_checks.py``). Opt in here with
``MINILINK_NOTEBOOK_CHECKS=1`` so default ``pytest`` stays fast.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Env / symbols that belong only in tests/demo_checks — never in teaching demos.
_FORBIDDEN_DEMO_HARNESS = re.compile(
    r"MINILINK_NOTEBOOK_SMOKE|_NOTEBOOK_SMOKE\b|MINILINK_.*_SMOKE"
)
_DEMO_ROOTS = (
    REPO_ROOT / "examples" / "notebooks",
    REPO_ROOT / "examples" / "scripts",
)


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

    def test_examples_have_no_smoke_env_hooks(self):
        """Teaching demos must not branch on CI/smoke env vars."""
        hits: list[str] = []
        for root in _DEMO_ROOTS:
            if not root.is_dir():
                continue
            for path in root.rglob("*"):
                if path.suffix not in {".py", ".ipynb"} or not path.is_file():
                    continue
                text = path.read_text(encoding="utf-8")
                if _FORBIDDEN_DEMO_HARNESS.search(text):
                    hits.append(path.relative_to(REPO_ROOT).as_posix())
        if hits:
            self.fail(
                "examples/notebooks and examples/scripts must not contain smoke/"
                "CI harness hooks (adapt in tests/demo_checks instead):\n  "
                + "\n  ".join(hits)
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

    def test_notebook_checks_exit_zero(self):
        if os.environ.get("MINILINK_NOTEBOOK_CHECKS") != "1":
            self.skipTest("set MINILINK_NOTEBOOK_CHECKS=1 to run notebook smoke")
        proc = self._run("tests/demo_checks/run_notebook_checks.py")
        if proc.returncode != 0:
            self.fail(
                f"notebook checks failed (exit {proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )


if __name__ == "__main__":
    unittest.main()
