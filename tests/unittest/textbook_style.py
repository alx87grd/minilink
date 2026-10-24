"""The textbook-rule census: AST checks of RULES §5 over the library, by tier.

Run as a script to rewrite the allowlist after a module reaches the rule::

    python tests/unittest/textbook_style.py --write

``test_textbook_style.py`` fails on a new offence and on an allowlist entry the
code no longer earns, so the list only shrinks.
"""

from __future__ import annotations

import ast
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LIBRARY = ROOT / "minilink"
ALLOWLIST = Path(__file__).with_name("textbook_style_allowlist.txt")

# Machine-facing infrastructure (CONSTITUTION §6) and the provisional realtime band:
# judged on robustness, not on reading like the page.
EXEMPT = (
    "minilink/experimental/",
    "minilink/core/compile/",
    "minilink/graphical/animation/renderers/",
    "minilink/interfaces/",
    "minilink/simulation/realtime/",
)

# Orchestrators and plots: the structure checks only (5.8, 5.9, 5.12).
STRUCTURE = (
    "minilink/graphical/",
    "minilink/core/composition.py",
    "minilink/core/wiring.py",
    "minilink/core/discrete/hybrid_composition.py",
    "minilink/core/discrete/hybrid_diagram.py",
    "minilink/simulation/simulator.py",
    "minilink/simulation/discrete/hybrid_simulator.py",
    "minilink/planning/results.py",
    "minilink/planning/comparison.py",
    "minilink/planning/dynamic_programming/progress.py",
)

# Methods and functions whose body is an equation path or an algorithm step.
EQUATION_NAMES = frozenset(
    {
        "f",
        "h",
        "g",
        "tf",
        "ctl",
        "step",
        "margin",
        "value",
        "sdf",
        "sample",
        "log_prob",
        "entropy",
        "loss",
        "forward_dynamics",
        "inverse_dynamics",
        "forward_kinematics",
        "H",
        "C",
        "B",
        "J",
        "d",
        "q2x",
        "x2q",
        "A",
        "D",
        "N",
        "generalized_force",
    }
)

# Names a protocol outside minilink chooses (IPython, enum, pickling).
PROTOCOL_HOOKS = frozenset(
    {
        "_repr_html_",
        "_repr_latex_",
        "_repr_svg_",
        "_repr_png_",
        "_repr_mimebundle_",
        "_repr_pretty_",
        "_ipython_display_",
        "_missing_",
        "_generate_next_value_",
    }
)

FIRST_SCREEN_LINES = 60

# The one module that imports JAX for everyone else (RULES 5.12).
JAX_OWNER = "minilink/core/backends.py"

CHECKS = {
    "underscore": "RULES 5.8: a helper or method carries a plain name",
    "return-math": "RULES 5.3: the equation is a named line, never on `return`",
    "self-in-math": "RULES 5.3: no `self.` in a math line; unpack it first",
    "np-under-xp": "RULES 5.2: once `xp` is bound, the algebra uses `xp`",
    "first-screen": "RULES 5.9: the primary class or function opens the module",
    "bare-jax": "RULES 5.12: `require_jax()`, never a bare `import jax`",
    "banner": "RULES 5.8: `Public API` / `Internal machinery` sit between two `#` lines",
    "old-banner": "RULES 5.8: no `# ====` / `# ----` banners; sub-sections are plain",
    "title": "RULES 5.23: the module docstring is a one-line title",
    "markers": "RULES 5.8: `# Internal machinery` comes with `# Public API`",
    "quoted-annotation": "RULES 5.1: unquoted forward references under postponed annotations",
    "next-index": "RULES 5.22: the next index is written `x_{k+1}`",
    "lazy-import": "RULES 5.27: a lazy minilink import says why",
    "class-alias": "RULES 7.8: no class attribute aliasing another class's method",
    "array-on-return": "RULES 5.3: never build an array on the `return` line",
    "unnamed-matrix": "RULES 5.3: a complex matrix gets named entries, then its assembly",
    "grouped-unpack": "RULES 5.3: one parameter or field per line",
}

BANNER = re.compile(r"^\s*#{20,}\s*$")
OLD_BANNER = re.compile(r"^\s*#\s*([=\-~*]{10,}|.*\s[=\-]{4,}\s*$)")
SECTION = re.compile(r"^\s*# (Public API|Internal machinery)\b")
NEXT_INDEX = re.compile(r"#.*(_k\+1|\[k\s*\+\s*1\])")


def tier(rel: str) -> str:
    """``"exempt"``, ``"structure"`` or ``"textbook"`` for a repo-relative path."""
    if rel.startswith(EXEMPT):
        return "exempt"
    if rel.startswith(STRUCTURE):
        return "structure"
    return "textbook"


def census() -> Counter:
    """Every offence in the library, as ``Counter({"check<TAB>module::where": count})``."""
    found = Counter()
    for path in sorted(LIBRARY.rglob("*.py")):
        rel = path.relative_to(ROOT).as_posix()
        level = tier(rel)
        if level == "exempt":
            continue

        text = path.read_text(encoding="utf-8")
        tree = ast.parse(text)
        for check, where in module_offences(tree, rel, level, text.splitlines()):
            found[f"{check}\t{rel}::{where}"] += 1

    return found


def read_allowlist() -> Counter:
    allowed = Counter()
    if not ALLOWLIST.exists():
        return allowed

    for line in ALLOWLIST.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        key, _, count = line.rpartition("\t")
        allowed[key] = int(count)

    return allowed


def write_allowlist(found: Counter) -> None:
    header = [
        "# The textbook-rule backlog: check, module::where, count.",
        "# Shrinks only. Regenerate: python tests/unittest/textbook_style.py --write",
    ]
    rows = [f"{key}\t{found[key]}" for key in sorted(found)]
    ALLOWLIST.write_text("\n".join(header + rows) + "\n", encoding="utf-8")


###############################################################################
# Internal machinery
###############################################################################


def module_offences(tree: ast.Module, rel: str, level: str, lines):
    """Yield ``(check, where)`` for one module."""
    yield from underscore_names(tree)
    if rel != JAX_OWNER:
        yield from bare_jax_imports(tree)

    if not rel.endswith("__init__.py") and first_public_line(tree) > FIRST_SCREEN_LINES:
        yield "first-screen", "<module>"

    yield from layout_offences(tree, lines)
    yield from class_aliases(tree)
    yield from lazy_imports(tree, lines)
    if postponed_annotations(tree):
        yield from quoted_annotations(tree)

    if level != "textbook":
        return

    for qualname, fn in functions(tree):
        if fn.name in EQUATION_NAMES:
            yield from array_offences(fn, qualname)
        yield from grouped_unpacks(fn, qualname)

    for qualname, fn in functions(tree):
        if fn.name in EQUATION_NAMES:
            for node in ast.walk(fn):
                if isinstance(node, ast.Return) and has_math(node.value):
                    yield "return-math", qualname
                if is_math_line(node) and reads_self(node.value):
                    yield "self-in-math", qualname

        if binds_xp(fn):
            for node in ast.walk(fn):
                if is_np_call(node):
                    yield "np-under-xp", qualname


def layout_offences(tree, lines):
    """Banners, markers, the title and the next-index spelling: what the source text shows."""
    doc = ast.get_docstring(tree, clean=False)
    if (
        doc is not None
        and len([ln for ln in doc.strip().splitlines() if ln.strip()]) > 1
    ):
        yield "title", "<module>"

    sections = set()
    for i, line in enumerate(lines):
        match = SECTION.match(line)
        if match:
            if not line.startswith(" "):
                sections.add(match.group(1))
            above = lines[i - 1] if i > 0 else ""
            below = lines[i + 1] if i + 1 < len(lines) else ""
            if not (BANNER.match(above) and BANNER.match(below)):
                yield "banner", f"line {i + 1}"
        elif OLD_BANNER.match(line) and not BANNER.match(line):
            yield "old-banner", f"line {i + 1}"
        if NEXT_INDEX.search(line):
            yield "next-index", f"line {i + 1}"

    if "Internal machinery" in sections and "Public API" not in sections:
        yield "markers", "<module>"


def class_aliases(tree):
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for item in node.body:
            value = item.value if isinstance(item, ast.Assign) else None
            if (
                isinstance(value, ast.Attribute)
                and isinstance(value.value, ast.Name)
                and value.value.id[:1].isupper()
                and value.value.id != node.name
            ):
                yield "class-alias", f"{node.name}.{ast.unparse(item.targets[0])}"


def lazy_imports(tree, lines):
    """A minilink import inside a function needs a reason: a comment on its line or above.

    One comment covers a block of consecutive imports: an import right below a
    commented one inherits its reason.
    """
    for qualname, fn in functions(tree):
        imports = sorted(
            (
                node
                for node in ast.walk(fn)
                if isinstance(node, (ast.Import, ast.ImportFrom))
            ),
            key=lambda node: node.lineno,
        )
        reasoned_until = -1
        for node in imports:
            here = lines[node.lineno - 1]
            above = lines[node.lineno - 2] if node.lineno > 1 else ""
            reasoned = (
                "#" in here
                or above.strip().startswith("#")
                or reasoned_until == node.lineno - 1
            )
            if reasoned:
                reasoned_until = node.end_lineno

            module = (
                node.module if isinstance(node, ast.ImportFrom) else node.names[0].name
            )
            if (module or "").startswith("minilink") and not reasoned:
                yield "lazy-import", f"{qualname}: {module}"


def postponed_annotations(tree):
    return any(
        isinstance(node, ast.ImportFrom)
        and node.module == "__future__"
        and any(alias.name == "annotations" for alias in node.names)
        for node in tree.body
    )


def quoted_annotations(tree):
    for node in ast.walk(tree):
        annotations = []
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args.posonlyargs + node.args.args + node.args.kwonlyargs
            annotations += [arg.annotation for arg in args]
            annotations += [node.args.vararg and node.args.vararg.annotation]
            annotations += [node.args.kwarg and node.args.kwarg.annotation]
            annotations.append(node.returns)
        elif isinstance(node, ast.AnnAssign):
            annotations.append(node.annotation)
        for annotation in annotations:
            if isinstance(annotation, ast.Constant) and isinstance(
                annotation.value, str
            ):
                yield "quoted-annotation", f"line {annotation.lineno}"


def array_offences(fn, qualname):
    """An array built on ``return``, or a complex matrix literal with unnamed entries."""
    for node in ast.walk(fn):
        if isinstance(node, ast.Return) and is_array_call(node.value):
            yield "array-on-return", qualname
        if is_array_call(node) and node.args and isinstance(node.args[0], ast.List):
            rows = node.args[0].elts
            if rows and all(isinstance(row, ast.List) for row in rows):
                entries = [entry for row in rows for entry in row.elts]
                if any(operator_count(entry) > 1 for entry in entries):
                    yield "unnamed-matrix", qualname


def is_array_call(node) -> bool:
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in ("array", "asarray")
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id in ("xp", "np", "jnp")
    )


def operator_count(node) -> int:
    return sum(isinstance(sub, (ast.BinOp, ast.Call)) for sub in ast.walk(node))


def grouped_unpacks(fn, qualname):
    """``a, b = params["a"], params["b"]`` or ``a, b = self.a, self.b``: one per line instead."""
    for node in ast.walk(fn):
        if not (isinstance(node, ast.Assign) and isinstance(node.value, ast.Tuple)):
            continue
        items = node.value.elts
        if len(items) < 2:
            continue
        if all(is_param_read(item) for item in items):
            yield "grouped-unpack", qualname


def is_param_read(node) -> bool:
    if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name):
        return node.value.id == "params" and isinstance(node.slice, ast.Constant)
    return (
        isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "self"
    )


def underscore_names(tree: ast.Module):
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and private(
            node.name
        ):
            yield "underscore", node.name

        if isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(
                    item, (ast.FunctionDef, ast.AsyncFunctionDef)
                ) and private(item.name):
                    yield "underscore", f"{node.name}.{item.name}"


def private(name: str) -> bool:
    dunder = name.startswith("__") and name.endswith("__")
    return name.startswith("_") and not dunder and name not in PROTOCOL_HOOKS


def bare_jax_imports(tree: ast.Module):
    if_type_checking = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and "TYPE_CHECKING" in ast.unparse(node.test):
            if_type_checking.update(id(child) for child in ast.walk(node))

    for node in ast.walk(tree):
        if id(node) in if_type_checking:
            continue
        if isinstance(node, ast.Import) and any(
            alias.name == "jax" or alias.name.startswith("jax.") for alias in node.names
        ):
            yield "bare-jax", f"import {node.names[0].name}"
        if (
            isinstance(node, ast.ImportFrom)
            and (node.module or "").split(".")[0] == "jax"
        ):
            yield "bare-jax", f"from {node.module}"


def first_public_line(tree: ast.Module) -> int:
    """Where the primary definition starts, not counting the module constants above it.

    Constants a primary function reads sit above it, as in dp.py; they are part of the
    first screen's reading order, not a preamble.
    """
    constant_lines = 0
    for node in tree.body:
        is_def = isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
        if is_def and not private(node.name):
            starts = [node.lineno] + [d.lineno for d in node.decorator_list]
            return min(starts) - constant_lines

        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            constant_lines += node.end_lineno - node.lineno + 1

    return 0


def functions(tree: ast.Module):
    """Every function with its qualified name, nested ones included."""

    def walk(node, prefix):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                name = f"{prefix}{child.name}"
                yield name, child
                yield from walk(child, f"{name}.")
            elif isinstance(child, ast.ClassDef):
                yield from walk(child, f"{prefix}{child.name}.")

    yield from walk(tree, "")


def has_math(node) -> bool:
    """Arithmetic outside index expressions."""
    if node is None:
        return False

    index_nodes = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Subscript):
            index_nodes.update(id(s) for s in ast.walk(sub.slice))

    for sub in ast.walk(node):
        if id(sub) in index_nodes:
            continue
        if isinstance(sub, ast.BinOp):
            return True
        if isinstance(sub, ast.UnaryOp) and isinstance(sub.op, ast.USub):
            if not isinstance(sub.operand, ast.Constant):
                return True

    return False


def is_math_line(node) -> bool:
    if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign, ast.Return)):
        return has_math(node.value)

    return False


def reads_self(node) -> bool:
    return any(
        isinstance(sub, ast.Attribute)
        and isinstance(sub.value, ast.Name)
        and sub.value.id == "self"
        for sub in ast.walk(node)
    )


def binds_xp(fn) -> bool:
    for node in ast.walk(fn):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                names = target.elts if isinstance(target, ast.Tuple) else [target]
                if any(isinstance(n, ast.Name) and n.id == "xp" for n in names):
                    return True

    return False


def is_np_call(node) -> bool:
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "np"
    )


if __name__ == "__main__":
    if "--write" not in sys.argv:
        sys.exit("usage: python tests/unittest/textbook_style.py --write")

    write_allowlist(census())
    print(f"wrote {ALLOWLIST.relative_to(ROOT)}")
