"""A demo holds at most three knobs in capitals at the top; the rest of its numbers live in its story (RULES 6.16).

A knob is a module-level name in capitals. The demos that predate the rule are listed in
``demo_knob_allowlist.txt`` with their count, a ceiling: the list only shrinks, and a
count only falls.
"""

import ast
import pathlib
import re

REPO = pathlib.Path(__file__).resolve().parents[2]
DEMOS = REPO / "examples" / "demos"
ALLOWLIST = pathlib.Path(__file__).with_name("demo_knob_allowlist.txt")

# The most knobs a demo keeps on top; more make it a research script
MAX_KNOBS = 3

# A name in capitals: a letter, then capitals, digits and underscores
KNOB = re.compile(r"^[A-Z][A-Z0-9_]*$")


def knobs_of(path):
    """The module-level names in capitals a script assigns."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = set()
    for node in tree.body:
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        else:
            continue
        for target in targets:
            elements = target.elts if isinstance(target, ast.Tuple) else [target]
            for element in elements:
                if isinstance(element, ast.Name) and KNOB.match(element.id):
                    names.add(element.id)

    return names


def knob_counts():
    """``{relative path: count}`` for every demo script."""
    return {
        path.relative_to(REPO).as_posix(): len(knobs_of(path))
        for path in sorted(DEMOS.rglob("*.py"))
    }


def load_allowlist():
    """``{relative path: ceiling}`` from the allowlist."""
    rows = {}
    for line in ALLOWLIST.read_text(encoding="utf-8").splitlines():
        if line.strip() and not line.startswith("#"):
            path, ceiling = line.split()
            rows[path] = int(ceiling)

    return rows


def test_a_demo_keeps_at_most_three_knobs():
    """A demo over three knobs is a listed one, at or under its ceiling."""
    allowed = load_allowlist()

    over = {
        path: count
        for path, count in knob_counts().items()
        if count > allowed.get(path, MAX_KNOBS)
    }

    assert not over, (
        f"Demos over {MAX_KNOBS} knobs in capitals (name the narrative's numbers just "
        f"before the object they set, RULES 6.16): {over}"
    )


def test_the_allowlist_only_shrinks():
    """A listed demo that now fits, or holds fewer knobs, lowers or leaves the list."""
    counts = knob_counts()

    stale = {
        path: (ceiling, counts.get(path))
        for path, ceiling in load_allowlist().items()
        if counts.get(path, 0) <= MAX_KNOBS or counts[path] < ceiling
    }

    assert not stale, (
        "Lower these ceilings to the demo's count, or remove the rows of demos within "
        f"{MAX_KNOBS} knobs or gone (ceiling, count now): {stale}"
    )
