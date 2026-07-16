#!/usr/bin/env python3
"""One-shot L1 pytest consolidation: merge domain test modules per plan."""

from __future__ import annotations

import ast
import re
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

KEEP = {
    "test_geometry.py",
    "test_symbolic.py",
    "test_benchmark_smoke.py",
    "test_smoke_runners.py",
    "conftest.py",
    "planning_helpers.py",
    "graphics_contract_helpers.py",
}


_FUTURE_IMPORT_RE = re.compile(r"^from __future__ import .+\n", re.MULTILINE)


def _strip_future_imports(source: str) -> str:
    return _FUTURE_IMPORT_RE.sub("", source)


def _helper_stem(filename: str) -> str:
    return filename.replace("test_", "").replace(".py", "")


def _top_level_helper_defs(source: str) -> list[str]:
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    names: list[str] = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and not node.name.startswith("Test"):
            names.append(node.name)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if not node.name.startswith("test_"):
                names.append(node.name)
    return names


def _rename_helpers(source: str, renames: dict[str, str]) -> str:
    if not renames:
        return source
    tree = ast.parse(source)

    class Renamer(ast.NodeTransformer):
        def visit_Name(self, node: ast.Name) -> ast.Name:
            if node.id in renames:
                node.id = renames[node.id]
            return node

        def visit_FunctionDef(self, node: ast.FunctionDef) -> ast.FunctionDef:
            if node.name in renames:
                node.name = renames[node.name]
            self.generic_visit(node)
            return node

        def visit_AsyncFunctionDef(
            self, node: ast.AsyncFunctionDef
        ) -> ast.AsyncFunctionDef:
            if node.name in renames:
                node.name = renames[node.name]
            self.generic_visit(node)
            return node

        def visit_ClassDef(self, node: ast.ClassDef) -> ast.ClassDef:
            if node.name in renames:
                node.name = renames[node.name]
            self.generic_visit(node)
            return node

    tree = Renamer().visit(tree)
    ast.fix_missing_locations(tree)
    return ast.unparse(tree) + "\n"


def _collect_test_level_names(source: str) -> set[str]:
    """Names that must be unique across merged modules (pytest collectors)."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return set()
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.name.startswith("test_"):
                names.add(node.name)
        elif isinstance(node, ast.ClassDef) and node.name.startswith("Test"):
            names.add(node.name)
    return names


def merge_files(sources: list[str]) -> str:
    parts: list[str] = []
    seen_tests: set[str] = set()
    seen_helpers: set[str] = set()
    for i, name in enumerate(sources):
        path = UNITTEST / name
        text = path.read_text(encoding="utf-8")
        if i == 0:
            body = text.rstrip() + "\n"
            seen_tests |= _collect_test_level_names(text)
            seen_helpers |= set(_top_level_helper_defs(text))
            parts.append(body)
        else:
            body = _strip_future_imports(text.strip())
            if not body:
                continue
            test_names = _collect_test_level_names(body)
            dupes = seen_tests & test_names
            if dupes:
                raise RuntimeError(f"{name}: duplicate test names {sorted(dupes)}")
            seen_tests |= test_names

            renames: dict[str, str] = {}
            stem = _helper_stem(name)
            for helper in _top_level_helper_defs(body):
                if helper in seen_helpers:
                    renames[helper] = f"{helper}_{stem}"
                else:
                    seen_helpers.add(helper)
            body = _rename_helpers(body, renames)
            seen_helpers.update(renames.values())

            parts.append(f"\n\n# --- merged from {name} ---\n\n{body}\n")
    return "".join(parts)


def main() -> None:
    absorbed: set[str] = set()
    for target, sources in MERGE_MAP.items():
        merged = merge_files(sources)
        out = UNITTEST / target
        out.write_text(merged, encoding="utf-8")
        print(
            f"Wrote {target} ({len(sources)} sources, {len(merged.splitlines())} lines)"
        )
        for src in sources:
            if src != target:
                absorbed.add(src)

    for name in sorted(absorbed):
        path = UNITTEST / name
        if path.exists():
            path.unlink()
            print(f"Deleted {name}")

    remaining = sorted(p.name for p in UNITTEST.glob("test_*.py"))
    print(f"\nRemaining test files ({len(remaining)}):")
    for name in remaining:
        print(f"  {name}")


if __name__ == "__main__":
    main()
