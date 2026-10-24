"""The public-name inventory: every facade name, the files that use it, and the ones no test uses.

Run from the repository root::

    python benchmarks/public_names.py              # the table, one row per public object
    python benchmarks/public_names.py --untested   # the public names no test file uses
    python benchmarks/public_names.py --json OUT   # the rows as JSON

A public object is one exported by a facade's ``__all__`` (keyed by identity, so a
name re-exported by several facades is one row). A file uses it when it imports it
(``from minilink.x import Name``, deep or facade alike, relative imports resolved) or
reaches it by attribute from an imported module (``ml.control.PID``); a System verb
that shares a tool's name also counts when called as a method (``sys.linearize()``).
Notebook code cells count as files. ``tests/unittest/test_public_names.py`` holds the
``--untested`` list empty.
"""

from __future__ import annotations

import argparse
import ast
import importlib
import json
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FACADES = (
    "minilink",
    "minilink.catalog",
    "minilink.core",
    "minilink.blocks",
    "minilink.control",
    "minilink.analysis",
    "minilink.estimation",
    "minilink.simulation",
    "minilink.optimization",
    "minilink.planning",
    "minilink.learning",
    "minilink.interfaces",
    "minilink.dynamics",
    "minilink.graphical.catalog",
)

# The System verbs named as the tools they delegate to: a method call counts as a use
METHOD_VERBS = (
    "linearize",
    "jacobian",
    "find_equilibrium",
    "transfer_function",
    "plot_bode",
    "plot_pzmap",
    "plot_root_locus",
    "plot_step_response",
    "animate_modal",
)

# Folders never scanned
SKIPPED = {"__pycache__", "_build", "build", "dist", "node_modules"}

# Where a file sits, by its path under the repository root (first match wins)
COLUMNS = (
    ("tests", "tests/"),
    ("course", "examples/teaching/courses/"),
    ("topic", "examples/teaching/topics/"),
    ("tutorial", "examples/tutorial/"),
    ("demo", "examples/demos/"),
    ("project", "examples/projects/"),
    ("experimental", "experimental/"),
    ("experimental", "minilink/experimental/"),
    ("benchmark", "benchmarks/"),
    ("library", "minilink/"),
)


def registry() -> dict[int, dict]:
    """Every public object of the facades, by identity: its names, its facades and its home module."""
    rows = {}
    for facade in FACADES:
        module = importlib.import_module(facade)
        for name in module.__all__:
            obj = getattr(module, name)
            if isinstance(obj, types.ModuleType):
                continue
            row = rows.setdefault(
                id(obj),
                {
                    "name": name,
                    "names": set(),
                    "facades": set(),
                    "home": getattr(obj, "__module__", None) or facade,
                    "object": obj,
                },
            )
            row["names"].add(name)
            row["facades"].add(facade)

    return rows


def inventory() -> list[dict]:
    """One row per public object: its names, facades, home, and the files using it per column."""
    rows = registry()
    by_id = {key: {column: set() for column, _ in COLUMNS} for key in rows}
    verbs = {
        row["name"]: key for key, row in rows.items() if row["name"] in METHOD_VERBS
    }

    for path in source_files():
        relative = path.relative_to(ROOT).as_posix()
        column = column_of(relative)
        tree = parse(path)
        if column is None or tree is None:
            continue
        for obj_id in used_ids(tree, relative, verbs):
            if obj_id not in by_id:
                continue
            if column == "library" and is_home_or_facade(relative, rows[obj_id]):
                continue
            by_id[obj_id][column].add(relative)

    table = []
    for key, row in rows.items():
        users = {column: sorted(files) for column, files in by_id[key].items()}
        table.append(
            {
                "name": row["name"],
                "names": sorted(row["names"]),
                "facades": sorted(row["facades"]),
                "home": row["home"],
                "users": users,
            }
        )

    return sorted(table, key=lambda row: (row["home"], row["name"]))


def untested(table=None) -> list[str]:
    """The public names no file under ``tests/`` uses, as ``facade.Name``."""
    table = inventory() if table is None else table

    return sorted(
        f"{row['facades'][0]}.{row['name']}"
        for row in table
        if not row["users"]["tests"]
    )


###############################################################################
# Internal machinery
###############################################################################


def source_files():
    """The Python files and notebooks of the repository, in a stable order."""
    for pattern in ("*.py", "*.ipynb"):
        for path in sorted(ROOT.rglob(pattern)):
            parts = path.relative_to(ROOT).parts
            if any(part.startswith(".") or part in SKIPPED for part in parts):
                continue
            yield path


def column_of(relative):
    """The column of a repository path, or ``None`` for a file outside them."""
    for column, prefix in COLUMNS:
        if relative.startswith(prefix):
            return column

    return None


def parse(path):
    """The AST of a Python file or of a notebook's code cells (``%`` and ``!`` lines dropped)."""
    try:
        if path.suffix == ".ipynb":
            cells = json.loads(path.read_text(encoding="utf-8"))["cells"]
            lines = [
                line
                for cell in cells
                if cell["cell_type"] == "code"
                for line in "".join(cell["source"]).splitlines()
                if not line.lstrip().startswith(("%", "!"))
            ]
            return ast.parse("\n".join(lines))
        return ast.parse(path.read_text(encoding="utf-8"))
    except (SyntaxError, ValueError, KeyError):
        return None


def used_ids(tree, relative, verbs):
    """The ids of the objects a file imports or reaches by attribute from an imported module."""
    package = package_of(relative)
    aliases = {}  # a local name bound to a minilink module
    found = set()

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("minilink"):
                    top = alias.asname or alias.name.split(".")[0]
                    aliases[top] = alias.name if alias.asname else "minilink"
        elif isinstance(node, ast.ImportFrom):
            module_name = resolved_module(node, package)
            if not module_name or not module_name.startswith("minilink"):
                continue
            for alias in node.names:
                obj = imported(module_name, alias.name)
                if isinstance(obj, types.ModuleType):
                    aliases[alias.asname or alias.name] = obj.__name__
                elif obj is not None:
                    found.add(id(obj))

    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            obj = attribute_object(node, aliases)
            if obj is not None and not isinstance(obj, types.ModuleType):
                found.add(id(obj))
            if node.attr in verbs and isinstance(node.ctx, ast.Load):
                found.add(verbs[node.attr])

    return found


def package_of(relative):
    """The dotted package of a file under ``minilink/`` (for relative imports), else ``None``."""
    if not relative.startswith("minilink/"):
        return None
    parts = Path(relative).with_suffix("").parts

    return ".".join(parts[:-1])


def resolved_module(node, package):
    """The absolute module of a ``from … import`` (a relative one resolved against ``package``)."""
    if node.level == 0:
        return node.module
    if package is None:
        return None
    base = package.split(".")[: len(package.split(".")) - node.level + 1]

    return ".".join(base + ([node.module] if node.module else []))


def imported(module_name, name):
    """The object ``from module_name import name`` binds, or ``None`` when it does not resolve."""
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return None
    if hasattr(module, name):
        return getattr(module, name)
    try:
        return importlib.import_module(f"{module_name}.{name}")
    except Exception:
        return None


def attribute_object(node, aliases):
    """The object a dotted attribute chain on a module alias names (``ml.control.PID``), or ``None``."""
    chain = []
    while isinstance(node, ast.Attribute):
        chain.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name) or node.id not in aliases:
        return None

    obj = importlib.import_module(aliases[node.id])
    for attr in reversed(chain):
        if not isinstance(obj, types.ModuleType):
            return None
        try:
            obj = getattr(obj, attr)
        except Exception:
            try:
                obj = importlib.import_module(f"{obj.__name__}.{attr}")
            except Exception:
                return None

    return obj


def is_home_or_facade(relative, row):
    """Whether a library file is the object's defining module or a facade's ``__init__.py``."""
    module = Path(relative).with_suffix("").as_posix().replace("/", ".")
    if module.endswith(".__init__"):
        module = module[: -len(".__init__")]
        if module in FACADES:
            return True

    return module == row["home"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--untested", action="store_true", help="list untested names")
    parser.add_argument("--json", metavar="OUT", help="write the rows as JSON")
    args = parser.parse_args(argv)

    table = inventory()
    if args.json:
        Path(args.json).write_text(json.dumps(table, indent=1) + "\n", encoding="utf-8")
    if args.untested:
        names = untested(table)
        print("\n".join(names) if names else "every public name has a test")
        return 1 if names else 0

    columns = list(dict.fromkeys(column for column, _ in COLUMNS))
    print(f"{'name':40s} {'home':55s} " + " ".join(f"{c[:5]:>5s}" for c in columns))
    for row in table:
        counts = " ".join(f"{len(row['users'][c]):>5d}" for c in columns)
        print(f"{row['name']:40s} {row['home']:55s} {counts}")

    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    sys.exit(main())
