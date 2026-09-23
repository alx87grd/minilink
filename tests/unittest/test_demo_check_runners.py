"""Demo-check runner subprocess tests (catalog + flagship demos + graphics).

Thin CI bridge: invokes ``tests/demo_checks/`` runners; does not duplicate their
assertions. Without optional extras (JAX, …), those flagships skip here — the
CI ``regression`` job re-runs ``run_flagship_demos.py`` with extras.
Interactive flagships (viewer / prompt demos) always skip.

Notebook smoke checks run in the CI ``regression`` job (and via
``tests/run/run_notebook_checks.py``). Opt in here with
``MINILINK_NOTEBOOK_CHECKS=1`` so default ``pytest`` stays fast.

``TestDemoCheckManifests`` checks the data those runners read (``requires``
lists, demo and notebook ids) and that the ``tests/run`` regression launcher
passes the CI job's flags.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import subprocess
import sys
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock

from tests.demo_checks import run_flagship_demos as flagship_runner
from tests.demo_checks import run_notebook_checks as notebook_runner
from tests.run import _common as launcher

REPO_ROOT = Path(__file__).resolve().parents[2]

# Env / symbols that belong only in tests/demo_checks — never in teaching demos.
_FORBIDDEN_DEMO_HARNESS = re.compile(
    r"MINILINK_NOTEBOOK_SMOKE|_NOTEBOOK_SMOKE\b|MINILINK_.*_SMOKE"
)
_DEMO_ROOTS = (
    REPO_ROOT / "examples" / "tutorial",
    REPO_ROOT / "examples" / "teaching",
    REPO_ROOT / "examples" / "demos",
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
                "examples/tutorial, examples/teaching, and examples/demos must not "
                "contain smoke/CI harness hooks (adapt in tests/demo_checks "
                "instead):\n  " + "\n  ".join(hits)
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


# Runners skip an entry when ``importlib.util.find_spec(name)`` is None for one
# of its ``requires``, so each name must be an import name, not a distribution
# name (``stable-baselines3``, ``pyyaml``): a wrong name skips forever. These
# are the import names of the pyproject extras plus stable_baselines3; add one
# here when a manifest first requires a new package.
_OPTIONAL_IMPORT_NAMES = frozenset(
    {
        "cyipopt",
        "graphviz",
        "gymnasium",
        "jax",
        "jaxlib",
        "meshcat",
        "plotly",
        "pygame",
        "stable_baselines3",
        "sympy",
    }
)
_REQUIRES_MANIFESTS = (
    "tests/demo_checks/flagship_manifest.json",
    "tests/demo_checks/notebook_overrides.json",
    "tests/fixtures/flagship_graphics/manifest.json",
    "tests/fixtures/kinematic_baseline/manifest.json",
)


def _load_json(relative_path: str):
    return json.loads((REPO_ROOT / relative_path).read_text(encoding="utf-8"))


def _without_graphviz():
    """Patch ``find_spec`` so the runners see no graphviz (no ``diagrams`` extra)."""
    find_spec = importlib.util.find_spec

    def find_spec_without_graphviz(name, *args, **kwargs):
        if name == "graphviz":
            return None
        return find_spec(name, *args, **kwargs)

    return mock.patch("importlib.util.find_spec", find_spec_without_graphviz)


class TestDemoCheckManifests(unittest.TestCase):
    def test_requires_are_import_names(self):
        for relative_path in _REQUIRES_MANIFESTS:
            manifest = _load_json(relative_path)
            entries = manifest.values() if isinstance(manifest, dict) else manifest
            for entry in entries:
                for name in entry.get("requires") or []:
                    with self.subTest(manifest=relative_path, name=name):
                        self.assertIn(name, _OPTIONAL_IMPORT_NAMES)

    def test_graphics_demo_ids_name_flagships(self):
        flagship_ids = {
            entry["id"]
            for entry in _load_json("tests/demo_checks/flagship_manifest.json")
        }
        for entry in _load_json("tests/fixtures/flagship_graphics/manifest.json"):
            if "demo_id" in entry:
                with self.subTest(entry=entry["id"]):
                    self.assertIn(entry["demo_id"], flagship_ids)

    def test_hybrid_diagram_flagships_skip_without_graphviz(self):
        """``hybrid.plot_diagram()`` imports graphviz (the ``diagrams`` extra)."""
        with _without_graphviz():
            for demo_id in ("mpc_integrator_numpy", "mpc_car_minimal"):
                with self.subTest(demo=demo_id):
                    [row] = flagship_runner.run_flagship_demos(demo_filter=demo_id)
                    self.assertEqual(row.status, "skip")

    def test_hybrid_diagram_notebook_skips_without_graphviz(self):
        """``06_hybrid`` calls ``hybrid.plot_diagram()`` too."""
        execute = mock.patch.object(
            notebook_runner, "_execute_notebook", return_value=("pass", "")
        )
        with _without_graphviz(), execute:
            [row] = notebook_runner.run_notebook_checks(
                notebook_filter="tutorial_06_hybrid"
            )
        self.assertEqual(row.status, "skip")

    def test_notebook_ids_are_unique(self):
        ids = notebook_runner.notebook_ids(notebook_runner._discover_notebooks())
        repeated = sorted(
            notebook_id for notebook_id, n in Counter(ids.values()).items() if n > 1
        )
        self.assertEqual(repeated, [])
        # The ids the ``--notebook`` usage and help text cite.
        self.assertEqual(
            ids["examples/tutorial/showcase_minilink.ipynb"], "showcase_minilink"
        )
        self.assertEqual(ids["examples/tutorial/00_core.ipynb"], "tutorial_00_core")

    def test_regression_launcher_ci_mode_matches_ci_workflow(self):
        workflow = (REPO_ROOT / ".github/workflows/test.yml").read_text(
            encoding="utf-8"
        )
        command = re.search(
            r"python benchmarks/run_regression_check\.py((?:.*\\\n)*.*)", workflow
        )
        ci_args = command.group(1).replace("\\\n", " ").split()
        with mock.patch.object(launcher, "run_command", return_value=0) as run:
            launcher.run_regression(ci_mode=True)
        launcher_args = run.call_args.args[0][2:]
        self.assertEqual(launcher_args, ci_args)


if __name__ == "__main__":
    unittest.main()
