"""The repo's contract with itself: rules that used to drift because nothing checked them.

Four rules of RULES.md are cheap to verify and were all violated within days of
being written, so they are tests now rather than prose:

- 6.9 public-facing prose never names another tool (the hand-written grep was
  BRE, so its alternation was literal and the gate passed on anything);
- 7.6 internal documentation links resolve, paths and anchors alike (the review
  ladder's own table of contents pointed at two slugs that do not exist);
- 5.8 no leading-underscore pseudo-privacy on the System family or the
  teaching-lane simulators;
- 6.7 teaching-lane code that picks Ipopt probes for ``cyipopt`` first, so it
  still runs on an install without it (a cart-pole demo hard-coded it).

Every notebook opens in Colab, so its setup cell is checked too: it clones the
default branch and puts the clone on the path before minilink is imported (one
notebook cloned a stale working branch).

The test suite and the merge gate drifted the same way, so they are tests too:

- 6.5 a missing extra skips its own tests only (a mid-file
  ``pytest.importorskip("jax")`` skipped every NumPy test of its module);
- the one CI job that installs jax runs ``pytest`` (it ran the demos only, so no
  JAX test gated a merge).
"""

from __future__ import annotations

import ast
import json
import os
import pathlib
import re
import subprocess
import sys
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]

# 6.9 — the tools public prose must not name.
OTHER_TOOLS = re.compile(r"simulink|matlab|drake|casadi|mujoco", re.IGNORECASE)

PROSE_FILES = (
    "README.md",
    "docs/index.rst",
)
PROSE_NOTEBOOKS = (
    "examples/tutorial/showcase_minilink.ipynb",
    "examples/tutorial/showcase_jax.ipynb",
    "examples/tutorial/showcase_from_rl_to_bode.ipynb",
)

# 7.6 — documents whose internal links must resolve.  Dated audits under
# docs/reviews/ are history and are deliberately excluded.
LINKED_DOCS = (
    "CLAUDE.md",
    "CONSTITUTION.md",
    "RULES.md",
    "AGENTS.md",
    "DESIGN.md",
    "ROADMAP.md",
    "README.md",
    "install.md",
    "examples/README.md",
    "tests/README.md",
    "benchmarks/README.md",
)
LINKED_DOC_TREES = ("docs/plans",)

# Colab — the one setup cell every notebook that imports minilink carries.
# ``-b main`` names the default branch and is allowed; any other pin is not.
COLAB_SETUP_LINES = (
    'if "google.colab" in sys.modules:',
    'get_ipython().run_line_magic("matplotlib", "inline")',
    'sys.path.insert(0, "/content/minilink")',
)
COLAB_CLONE = re.compile(
    r'get_ipython\(\)\.system\("git clone (-b main )?'
    r'https://github\.com/alx87grd/minilink"\)'
)
IMPORTS_MINILINK = re.compile(r"^\s*(from|import) minilink\b", re.MULTILINE)

MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
HEADING = re.compile(r"^#{1,6}\s+(.*)$", re.MULTILINE)

# 5.8 — the classes the rule names.  simulation/realtime/ is provisional
# research lane (ROADMAP §3, TRL 2) and is out of scope until its review.
NAMED_CLASS_MODULES = (
    "minilink/core/system.py",
    "minilink/core/facades.py",
    "minilink/core/diagram.py",
    "minilink/simulation/simulator.py",
    "minilink/simulation/static_simulator.py",
)

# 6.7 — the teaching lane, which must run without the optional Ipopt build.
TEACHING_EXAMPLE_ROOTS = ("examples/tutorial", "examples/teaching", "examples/demos")


# 6.5 — the unit-test modules that are JAX from end to end, the only ones a
# missing jax may skip whole.  Elsewhere the ``jax`` marker skips the JAX tests
# and the NumPy tests beside them still run.
JAX_ONLY_TEST_MODULES = [
    "tests/unittest/test_engine_jax.py",
    "tests/unittest/test_jax_planning.py",
    "tests/unittest/test_rl_planner.py",
    "tests/unittest/test_ur5_jax.py",
]

# Collects the unit tests with jax blocked and prints, as JSON, the modules
# skipped or broken at collection and the jax-marked tests left to run.
COLLECT_WITHOUT_JAX = r"""
import json, sys
sys.modules["jax"] = None
import pytest


class Collection:
    def __init__(self):
        self.skipped_modules, self.broken_modules, self.jax_tests_left = [], [], []

    def pytest_collectreport(self, report):
        if report.skipped:
            self.skipped_modules.append(report.nodeid)
        if report.failed:
            self.broken_modules.append(report.nodeid)

    def pytest_collection_finish(self, session):
        for item in session.items:
            if item.get_closest_marker("jax") and not item.get_closest_marker("skip"):
                self.jax_tests_left.append(item.nodeid)


collection = Collection()
pytest.main(["--collect-only", "-p", "no:cacheprovider", "tests/unittest"],
            plugins=[collection])
print(json.dumps(vars(collection)))
"""

# The merge gate: a CI job that installs the jax extra must also run pytest.
CI_WORKFLOW = ".github/workflows/test.yml"
INSTALLS_JAX = re.compile(r"pip install[^\n]*\[[^\]]*\b(jax|full)\b[^\]]*\]")
RUNS_PYTEST = re.compile(r"^\s*(python -m )?pytest\b", re.MULTILINE)


def heading_slug(heading: str) -> str:
    """GitHub's anchor rule: lowercase, drop punctuation, one hyphen per space."""
    text = re.sub(r"[^\w\s-]", "", heading.strip().lower())
    return text.replace(" ", "-")


def notebook_markdown(path: pathlib.Path) -> str:
    cells = json.loads(path.read_text()).get("cells", [])
    return "\n".join(
        "".join(cell.get("source", []))
        for cell in cells
        if cell.get("cell_type") == "markdown"
    )


def example_code(path: pathlib.Path) -> str:
    """A script's source, or a notebook's code cells without shell and magic lines."""
    if path.suffix == ".py":
        return path.read_text()
    cells = json.loads(path.read_text()).get("cells", [])
    code = "\n".join(
        "".join(cell.get("source", []))
        for cell in cells
        if cell.get("cell_type") == "code"
    )
    return "\n".join(
        line for line in code.splitlines() if not line.lstrip().startswith(("%", "!"))
    )


class TestPublicProse(unittest.TestCase):
    """RULES 6.9 — minilink is described on its own terms, never against another tool."""

    def test_prose_files_name_no_other_tool(self):
        for name in PROSE_FILES:
            text = (REPO / name).read_text()
            found = OTHER_TOOLS.findall(text)
            self.assertEqual(found, [], f"{name} names another tool: {set(found)}")

    def test_showcase_markdown_names_no_other_tool(self):
        for name in PROSE_NOTEBOOKS:
            text = notebook_markdown(REPO / name)
            found = OTHER_TOOLS.findall(text)
            self.assertEqual(found, [], f"{name} names another tool: {set(found)}")


class TestDocumentLinks(unittest.TestCase):
    """RULES 7.6 — every link is a maintenance edge, so the edges are checked."""

    def documents(self):
        for name in LINKED_DOCS:
            yield REPO / name
        for tree in LINKED_DOC_TREES:
            yield from sorted((REPO / tree).rglob("*.md"))

    def test_internal_links_resolve(self):
        anchors: dict[pathlib.Path, set[str]] = {}

        def headings_of(path: pathlib.Path) -> set[str]:
            if path not in anchors:
                anchors[path] = {
                    heading_slug(h) for h in HEADING.findall(path.read_text())
                }
            return anchors[path]

        for doc in self.documents():
            text = doc.read_text()
            where = doc.relative_to(REPO)
            for target in MARKDOWN_LINK.findall(text):
                if target.startswith(("http://", "https://", "mailto:", "#/")):
                    continue
                path, _, anchor = target.partition("#")
                destination = doc if not path else (doc.parent / path).resolve()
                self.assertTrue(
                    destination.exists(),
                    f"{where} links to a missing path: {target}",
                )
                if anchor and destination.suffix == ".md":
                    self.assertIn(
                        anchor,
                        headings_of(destination),
                        f"{where} links to a missing anchor: {target}",
                    )


class TestInternalNaming(unittest.TestCase):
    """RULES 5.8 — section comments mark internal machinery, not underscores."""

    def test_no_pseudo_private_methods_on_the_named_classes(self):
        for name in NAMED_CLASS_MODULES:
            tree = ast.parse((REPO / name).read_text())
            for node in ast.walk(tree):
                if not isinstance(node, ast.ClassDef):
                    continue
                for item in node.body:
                    if not isinstance(item, ast.FunctionDef):
                        continue
                    private = item.name.startswith("_") and not item.name.endswith("_")
                    self.assertFalse(
                        private,
                        f"{name}: {node.name}.{item.name} uses a leading underscore; "
                        "give it a plain name under a '# Internal machinery' comment",
                    )


class TestOptionalIpopt(unittest.TestCase):
    """RULES 6.7 — Ipopt is optional, so the teaching lane falls back without it."""

    def test_code_that_picks_ipopt_probes_for_cyipopt(self):
        for root in TEACHING_EXAMPLE_ROOTS:
            for path in sorted((REPO / root).rglob("*")):
                if path.suffix not in (".py", ".ipynb"):
                    continue
                tree = ast.parse(example_code(path))
                constants = [
                    node.value
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Constant)
                ]
                if "ipopt" not in constants:
                    continue
                self.assertTrue(
                    "cyipopt" in constants,
                    f"{path.relative_to(REPO)} picks 'ipopt' without probing "
                    "importlib.util.find_spec('cyipopt'); fall back to 'scipy_slsqp'",
                )


class TestColabSetup(unittest.TestCase):
    """Every notebook runs in Colab from a clone of the default branch."""

    def test_notebooks_that_import_minilink_set_up_colab_first(self):
        for path in sorted((REPO / "examples").rglob("*.ipynb")):
            name = path.relative_to(REPO)
            code = example_code(path)
            first_import = IMPORTS_MINILINK.search(code)
            if first_import is None:
                continue
            setup = code[: first_import.start()]
            for line in COLAB_SETUP_LINES:
                self.assertIn(line, setup, f"{name}: Colab setup cell lacks {line!r}")
            self.assertRegex(
                setup,
                COLAB_CLONE,
                f"{name}: Colab cell must clone the default branch "
                "(git clone https://github.com/alx87grd/minilink)",
            )


class TestOptionalDependencies(unittest.TestCase):
    """RULES 6.5 — a missing extra skips its own tests, never the NumPy ones."""

    def test_a_missing_jax_skips_only_the_jax_tests(self):
        env = {**os.environ, "PYTHONPATH": str(REPO), "MPLBACKEND": "Agg"}
        result = subprocess.run(
            [sys.executable, "-c", COLLECT_WITHOUT_JAX],
            cwd=REPO,
            env=env,
            capture_output=True,
            text=True,
            timeout=600,
        )
        self.assertEqual(result.returncode, 0, result.stderr[-3000:])
        collection = json.loads(result.stdout.strip().splitlines()[-1])

        self.assertEqual(collection["broken_modules"], [])
        self.assertEqual(sorted(collection["skipped_modules"]), JAX_ONLY_TEST_MODULES)
        self.assertEqual(collection["jax_tests_left"], [])


class TestMergeGate(unittest.TestCase):
    """The JAX tests gate a merge, not only the JAX demos."""

    def test_the_job_that_installs_jax_runs_pytest(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML is not installed")

        workflow = yaml.safe_load((REPO / CI_WORKFLOW).read_text())
        runs_by_job = {
            name: [step.get("run", "") for step in job["steps"]]
            for name, job in workflow["jobs"].items()
        }
        jax_jobs = [
            name
            for name, runs in runs_by_job.items()
            if any(INSTALLS_JAX.search(run) for run in runs)
        ]

        self.assertTrue(jax_jobs, f"{CI_WORKFLOW}: no job installs the jax extra")
        for name in jax_jobs:
            self.assertTrue(
                any(RUNS_PYTEST.search(run) for run in runs_by_job[name]),
                f"{CI_WORKFLOW}: job '{name}' installs jax but never runs pytest",
            )


if __name__ == "__main__":
    unittest.main()
