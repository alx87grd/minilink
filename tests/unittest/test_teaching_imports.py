"""Student-facing code imports through the teaching surface (RULES 4.2, ROADMAP §2).

Every ``from minilink... import`` in ``examples/tutorial/``, ``examples/teaching/``,
and ``examples/demos/`` must target the root prelude or a band facade — or be listed,
per file, in ``teaching_import_allowlist.txt``. The allowlist only shrinks: it records the
deep imports that predate the rule so the test is green today and the
rewrite can proceed folder by folder.
"""

from __future__ import annotations

import ast
import json
import pathlib
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
ALLOWLIST = pathlib.Path(__file__).with_name("teaching_import_allowlist.txt")

# Facades students import through (band packages), plus the documented
# "factory named like its module" exception.
TEACHING_MODULES = {
    "minilink",
    "minilink.catalog",
    "minilink.core",
    "minilink.blocks",
    "minilink.control",
    "minilink.control.lqr",
    "minilink.control.mpc",
    "minilink.analysis",
    "minilink.analysis.linearize",
    "minilink.analysis.discretize",
    "minilink.simulation",
    "minilink.planning",
    "minilink.optimization",
    "minilink.interfaces.gymnasium",
}

ROOTS = ("examples/tutorial", "examples/teaching", "examples/demos")


def _sources():
    for root in ROOTS:
        for path in sorted((REPO / root).rglob("*.py")):
            yield path, path.read_text(errors="ignore")
        for path in sorted((REPO / root).rglob("*.ipynb")):
            cells = json.loads(path.read_text()).get("cells", [])
            code = "\n".join(
                "".join(c.get("source", []))
                for c in cells
                if c.get("cell_type") == "code"
            )
            code = "\n".join(
                line
                for line in code.splitlines()
                if not line.lstrip().startswith(("%", "!"))
            )
            yield path, code


def deep_imports():
    """Yield ``(relative_path, module)`` for every non-facade minilink import."""
    for path, code in _sources():
        try:
            tree = ast.parse(code)
        except SyntaxError:
            continue
        rel = path.relative_to(REPO).as_posix()
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.ImportFrom)
                and node.module
                and node.module.startswith("minilink")
            ):
                if node.module not in TEACHING_MODULES:
                    yield rel, node.module
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if (
                        alias.name.startswith("minilink.")
                        and alias.name not in TEACHING_MODULES
                    ):
                        yield rel, alias.name


def load_allowlist():
    if not ALLOWLIST.exists():
        return set()
    return {
        tuple(line.split())
        for line in ALLOWLIST.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    }


class TestTeachingImports(unittest.TestCase):
    def test_student_facing_imports_go_through_the_teaching_surface(self):
        allowed = load_allowlist()
        offenders = sorted({(rel, mod) for rel, mod in deep_imports()} - allowed)
        self.assertEqual(
            offenders,
            [],
            "Deep imports in student-facing code (use a band facade, or add to the "
            "allowlist only for names no facade exports yet):\n"
            + "\n".join(f"  {rel}: {mod}" for rel, mod in offenders),
        )

    def test_allowlist_has_no_stale_entries(self):
        present = {(rel, mod) for rel, mod in deep_imports()}
        stale = sorted(load_allowlist() - present)
        self.assertEqual(
            stale, [], f"Remove from the allowlist (no longer used): {stale}"
        )


if __name__ == "__main__":
    unittest.main()
