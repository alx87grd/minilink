#!/usr/bin/env python3
"""L1 pytest consolidation: merge domain test modules with in-file deduplication."""

from __future__ import annotations

import argparse
import ast
import hashlib
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

UNITTEST = Path(__file__).resolve().parents[1] / "tests" / "unittest"

MERGE_MAP: dict[str, list[str]] = {
    "test_core.py": [
        "test_core.py",
        "test_composition.py",
        "test_standard_feedback.py",
        "test_system_evolution_maps.py",
    ],
    "test_backends.py": [
        "test_backends.py",
        "test_backend_native_math_helpers.py",
    ],
    "test_compile.py": [
        "test_compile_pipeline.py",
        "test_compile_static.py",
        "test_compile_step_leaf.py",
        "test_evaluator_api.py",
        "test_evaluator_tiers.py",
        "test_mathematical_program_evaluators.py",
    ],
    "test_diagrams.py": [
        "test_diagrams.py",
        "test_wiring_mixin.py",
        "test_facades_split.py",
    ],
    "test_simulation.py": [
        "test_simulator.py",
        "test_static_simulator.py",
        "test_discontinuous_solvers.py",
        "test_integrate_zoh.py",
        "test_computer.py",
        "test_as_computer.py",
    ],
    "test_step_discrete.py": [
        "test_step_system.py",
        "test_step_diagram.py",
        "test_step_diagram_topology.py",
        "test_step_diagram_rollout.py",
        "test_step_rollout.py",
        "test_step_diagram_jax.py",
        "test_facades_rollout.py",
    ],
    "test_hybrid.py": [
        "test_hybrid_boundary_connect.py",
        "test_hybrid_closed_loop.py",
        "test_hybrid_simulator.py",
        "test_hybrid_topology.py",
        "test_hybrid_multi_rate.py",
        "test_hybrid_fine_recording.py",
        "test_smc_hybrid.py",
    ],
    "test_dynamics_catalog.py": [
        "test_catalog_migration.py",
        "test_catalog_plant_contracts.py",
        "test_manipulators.py",
        "test_dynamic_bicycle_uy.py",
    ],
    "test_mechanical_robotics.py": [
        "test_mechanical.py",
        "test_generalized_mechanical.py",
        "test_mechanical_jax.py",
        "test_manipulator.py",
        "test_robotic.py",
        "test_inverse_kinematics.py",
        "test_modelbased.py",
    ],
    "test_blocks.py": [
        "test_blocks.py",
        "test_signal_blocks.py",
        "test_signal_colors.py",
        "test_sources_white_noise.py",
        "test_neural_blocks.py",
    ],
    "test_control_analysis.py": [
        "test_analysis_control.py",
        "test_control_linear.py",
        "test_modal.py",
        "test_frequency.py",
        "test_phase_plane.py",
        "test_discretize.py",
        "test_state_space_system.py",
    ],
    "test_costs_optimizer.py": [
        "test_costs.py",
        "test_optimizer.py",
    ],
    "test_planning.py": [
        "test_planning_architecture.py",
        "test_planning_ui_constructors.py",
        "test_rrt.py",
        "test_spatial.py",
        "test_reference_paths.py",
        "test_dynamic_programming.py",
    ],
    "test_mpc.py": [
        "test_model_predictive_controller.py",
        "test_mpc_algebraic_controller.py",
        "test_mpc_warm_start_controller.py",
        "test_mpc_planner.py",
        "test_mpc_solve_trajectory_from.py",
        "test_mpc_export_computer.py",
        "test_mpc_numpy_rebuild.py",
        "test_mpc_hybrid_straight_line.py",
        "test_mpc_hybrid_warm_start_parity.py",
        "test_mpc_hybrid_demo_parity.py",
    ],
    "test_graphics.py": [
        "test_camera_transform.py",
        "test_world_frame_contract.py",
        "test_kinematic_regression.py",
        "test_overlays.py",
        "test_visualization_optional.py",
        "test_advanced_plotting.py",
        "test_plotly_renderer.py",
    ],
    "test_engine_jax.py": [
        "test_engine_world_system.py",
        "test_contact_engine_jax.py",
        "test_ancf_tire_jax.py",
    ],
    "test_jax_planning.py": [
        "test_jax_direct_collocation.py",
        "test_jax_kinematic_bicycle.py",
    ],
}

_FUTURE_IMPORT_RE = re.compile(r"^from __future__ import .+\n", re.MULTILINE)


@dataclass
class MergeState:
    seen_tests: set[str] = field(default_factory=set)
    seen_test_hashes: set[str] = field(default_factory=set)
    seen_imports: set[str] = field(default_factory=set)
    seen_helper_names: set[str] = field(default_factory=set)
    helper_hash_to_name: dict[str, str] = field(default_factory=dict)


def _strip_future_imports(source: str) -> str:
    return _FUTURE_IMPORT_RE.sub("", source)


def _helper_stem(filename: str) -> str:
    return filename.replace("test_", "").replace(".py", "")


def _is_import(node: ast.stmt) -> bool:
    return isinstance(node, (ast.Import, ast.ImportFrom))


def _is_main_guard(node: ast.stmt) -> bool:
    if not isinstance(node, ast.If):
        return False
    test = node.test
    if not isinstance(test, ast.Compare):
        return False
    left = test.left
    if not isinstance(left, ast.Name) or left.id != "__name__":
        return False
    return any(
        isinstance(op, ast.Eq)
        and isinstance(comp, ast.Constant)
        and comp.value == "__main__"
        for op, comp in zip(test.ops, test.comparators, strict=False)
    )


def _is_helper(node: ast.stmt) -> bool:
    if isinstance(node, ast.ClassDef):
        return not node.name.startswith("Test")
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return not node.name.startswith("test_")
    return False


def _is_test_collector(node: ast.stmt) -> bool:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return node.name.startswith("test_")
    if isinstance(node, ast.ClassDef):
        return node.name.startswith("Test")
    return False


def _split_module_docstring(
    body: list[ast.stmt],
) -> tuple[list[ast.stmt], list[ast.stmt]]:
    if (
        body
        and isinstance(body[0], ast.Expr)
        and isinstance(body[0].value, ast.Constant)
        and isinstance(body[0].value.value, str)
    ):
        return body[:1], body[1:]
    return [], body


def _import_key(node: ast.stmt) -> str:
    return ast.unparse(node)


def _node_dump(node: ast.AST) -> str:
    return ast.dump(node, annotate_fields=False, include_attributes=False)


def _function_equiv_hash(node: ast.FunctionDef | ast.AsyncFunctionDef) -> str:
    local_names: list[str] = []
    for stmt in node.body:
        if isinstance(stmt, ast.Assign):
            for target in stmt.targets:
                if isinstance(target, ast.Name) and target.id not in local_names:
                    local_names.append(target.id)
    rename = {name: f"_v{i}" for i, name in enumerate(local_names)}

    class LocalRenamer(ast.NodeTransformer):
        def visit_Name(self, node: ast.Name) -> ast.Name:
            if node.id in rename:
                node.id = rename[node.id]
            return node

    parsed = ast.parse(ast.unparse(node))
    fn_node = parsed.body[0]
    if not isinstance(fn_node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        raise TypeError(type(fn_node))
    renamed = LocalRenamer().visit(ast.fix_missing_locations(fn_node))
    payload = ast.unparse(renamed.body)
    return hashlib.sha256(payload.encode()).hexdigest()


def _def_equiv_hash(node: ast.stmt) -> str:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return _function_equiv_hash(node)
    if isinstance(node, ast.ClassDef):
        return hashlib.sha256(
            _node_dump(ast.Module(body=node.body, type_ignores=[])).encode()
        ).hexdigest()
    raise TypeError(type(node))


def _collect_test_names(source: str) -> set[str]:
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return set()
    names: set[str] = set()
    for node in tree.body:
        if _is_test_collector(node):
            names.add(node.name)
    return names


class _Renamer(ast.NodeTransformer):
    def __init__(self, renames: dict[str, str]) -> None:
        self.renames = renames

    def visit_Name(self, node: ast.Name) -> ast.Name:
        if node.id in self.renames:
            node.id = self.renames[node.id]
        return node

    def visit_FunctionDef(self, node: ast.FunctionDef) -> ast.FunctionDef:
        if node.name in self.renames:
            node.name = self.renames[node.name]
        self.generic_visit(node)
        return node

    def visit_AsyncFunctionDef(
        self, node: ast.AsyncFunctionDef
    ) -> ast.AsyncFunctionDef:
        if node.name in self.renames:
            node.name = self.renames[node.name]
        self.generic_visit(node)
        return node

    def visit_ClassDef(self, node: ast.ClassDef) -> ast.ClassDef:
        if node.name in self.renames:
            node.name = self.renames[node.name]
        self.generic_visit(node)
        return node


def _apply_renames(source: str, renames: dict[str, str]) -> str:
    if not renames:
        return source
    tree = ast.parse(source)
    tree = _Renamer(renames).visit(tree)
    ast.fix_missing_locations(tree)
    return ast.unparse(tree) + "\n"


def _process_section(
    source: str,
    *,
    stem: str,
    state: MergeState,
    keep_docstring: bool,
    marker: str | None,
) -> str:
    source = _strip_future_imports(source.strip())
    if not source:
        return ""

    tree = ast.parse(source)
    doc_nodes, body = _split_module_docstring(tree.body)

    kept: list[ast.stmt] = []
    renames: dict[str, str] = {}

    for node in body:
        if _is_main_guard(node):
            continue
        if _is_import(node):
            key = _import_key(node)
            if key in state.seen_imports:
                continue
            state.seen_imports.add(key)
            kept.append(node)
            continue

        if _is_helper(node) and isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
        ):
            equiv = _def_equiv_hash(node)
            if equiv in state.helper_hash_to_name:
                renames[node.name] = state.helper_hash_to_name[equiv]
                continue
            if node.name in state.seen_helper_names:
                new_name = f"{node.name}_{stem}"
                renames[node.name] = new_name
                state.seen_helper_names.add(new_name)
                state.helper_hash_to_name[equiv] = new_name
                node.name = new_name
            else:
                state.seen_helper_names.add(node.name)
                state.helper_hash_to_name[equiv] = node.name
            kept.append(node)
            continue

        if isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef)
        ) and node.name.startswith("test_"):
            equiv = _def_equiv_hash(node)
            if node.name in state.seen_tests:
                raise RuntimeError(f"duplicate test name {node.name!r}")
            if equiv in state.seen_test_hashes:
                continue
            state.seen_tests.add(node.name)
            state.seen_test_hashes.add(equiv)
            kept.append(node)
            continue

        if isinstance(node, ast.ClassDef) and node.name.startswith("Test"):
            if node.name in state.seen_tests:
                raise RuntimeError(f"duplicate test class {node.name!r}")
            state.seen_tests.add(node.name)
            kept.append(node)
            continue

        kept.append(node)

    if not kept and not (keep_docstring and doc_nodes):
        return ""

    module_body: list[ast.stmt] = []
    if keep_docstring and doc_nodes:
        module_body.extend(doc_nodes)
    module_body.extend(kept)

    module = ast.Module(body=module_body, type_ignores=[])
    ast.fix_missing_locations(module)
    text = ast.unparse(module) + "\n"
    text = _apply_renames(text, renames)

    if marker:
        return f"\n{text}"
    return text + "\n"


def merge_files(sources: list[str], read_source) -> str:
    state = MergeState()
    parts: list[str] = []

    first = read_source(sources[0])
    first = _strip_future_imports(first.strip()) + "\n"
    tree = ast.parse(first)
    for node in tree.body:
        if _is_import(node):
            state.seen_imports.add(_import_key(node))
        elif _is_helper(node) and isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
        ):
            state.seen_helper_names.add(node.name)
            state.helper_hash_to_name[_def_equiv_hash(node)] = node.name
    state.seen_tests |= _collect_test_names(first)

    # Drop __main__ from the first module; pytest collects tests directly.
    first_tree = ast.parse(first)
    first_body = [n for n in first_tree.body if not _is_main_guard(n)]
    first_tree.body = first_body
    ast.fix_missing_locations(first_tree)
    parts.append(ast.unparse(first_tree) + "\n")

    for name in sources[1:]:
        section = read_source(name)
        stem = _helper_stem(name)
        chunk = _process_section(
            section,
            stem=stem,
            state=state,
            keep_docstring=False,
            marker=name,
        )
        if chunk:
            parts.append(chunk)

    return "".join(parts)


def _read_from_git(rev: str):
    def read(name: str) -> str:
        path = f"tests/unittest/{name}"
        return subprocess.check_output(
            ["git", "show", f"{rev}:{path}"],
            text=True,
            cwd=UNITTEST.parents[1],
        )

    return read


def _read_from_disk(_rev: str | None):
    def read(name: str) -> str:
        path = UNITTEST / name
        if not path.exists():
            raise FileNotFoundError(path)
        return path.read_text(encoding="utf-8")

    return read


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--from-git",
        metavar="REV",
        help="Read absorbed sources from git (e.g. 8c81aed^) instead of disk",
    )
    parser.add_argument(
        "--no-delete",
        action="store_true",
        help="Do not delete absorbed source files",
    )
    args = parser.parse_args()

    read_source = (
        _read_from_git(args.from_git) if args.from_git else _read_from_disk(None)
    )

    absorbed: set[str] = set()
    before_lines = 0
    after_lines = 0

    for target, sources in MERGE_MAP.items():
        out = UNITTEST / target
        if out.exists():
            before_lines += len(out.read_text(encoding="utf-8").splitlines())

        merged = merge_files(sources, read_source)
        out.write_text(merged, encoding="utf-8")
        line_count = len(merged.splitlines())
        after_lines += line_count
        print(f"Wrote {target} ({len(sources)} sources, {line_count} lines)")
        for src in sources:
            if src != target:
                absorbed.add(src)

    if not args.no_delete:
        for name in sorted(absorbed):
            path = UNITTEST / name
            if path.exists():
                path.unlink()
                print(f"Deleted {name}")

    print(
        f"\nLine total: {before_lines} -> {after_lines} ({after_lines - before_lines:+d})"
    )
    remaining = sorted(p.name for p in UNITTEST.glob("test_*.py"))
    print(f"Remaining test files ({len(remaining)}):")
    for name in remaining:
        print(f"  {name}")


if __name__ == "__main__":
    main()
