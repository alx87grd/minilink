"""Every public plot takes renderer=, show= and ax=: an AST walk over the library."""

import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2] / "minilink"

# The plot_ names that draw nothing: a choice and a converter
NOT_FIGURES = {"plot_space_of", "plot_result_of"}

# The plots that do not take every keyword yet, and the keywords they lack. The
# list only shrinks: a name that takes them all must leave it.
NOT_YET = {
    ("analysis/frequency.py", "plot_bode"): {"ax"},
    ("analysis/frequency.py", "plot_pzmap"): {"ax"},
    ("analysis/frequency.py", "plot_root_locus"): {"ax"},
    ("analysis/frequency.py", "plot_nyquist"): {"ax"},
    ("analysis/lyapunov.py", "plot_region_of_attraction"): {"renderer"},
    ("analysis/time_response.py", "plot_step_response"): {"ax"},
    ("control/state.py", "plot_gain_schedule"): {"renderer"},
    ("core/facades/plotting.py", "plot_trajectory"): {"ax"},
    ("core/facades/plotting.py", "plot_cost"): {"ax"},
    ("core/facades/plotting.py", "plot_input_output_map"): {"renderer"},
    ("core/facades/plotting.py", "plot_control_law"): {"renderer"},
    ("core/facades/plotting.py", "plot_diagram"): {"renderer", "ax"},
    ("core/facades/plotting.py", "plot_bode"): {"ax"},
    ("core/facades/plotting.py", "plot_root_locus"): {"ax"},
    ("core/facades/plotting.py", "plot_pzmap"): {"ax"},
    ("core/facades/plotting.py", "plot_nyquist"): {"ax"},
    ("core/facades/plotting.py", "plot_step_response"): {"ax"},
    ("core/facades/plotting.py", "plot_phase_plane"): {"ax"},
    ("core/facades/plotting.py", "plot_rollout"): {"ax"},
    ("graphical/diagrams/dot.py", "plot_diagram"): {"renderer", "ax"},
    ("graphical/phase_plane/phase_plane.py", "plot_phase_plane"): {"ax"},
    ("graphical/port_map.py", "plot_input_output_map"): {"renderer"},
    ("graphical/port_map.py", "plot_control_law"): {"renderer"},
    ("graphical/signals/time_signals.py", "plot_time_signals"): {"ax"},
    ("learning/reinforcement_learning/planner.py", "plot_learning_curve"): {"renderer"},
    ("learning/reinforcement_learning/tabular.py", "plot_learning_curve"): {"renderer"},
    ("planning/comparison.py", "plot_control_law"): {"renderer", "ax"},
    ("planning/comparison.py", "plot_cost_to_go"): {"renderer", "ax"},
    ("planning/comparison.py", "plot_trajectory"): {"renderer", "ax"},
    ("planning/dynamic_programming/discretizer.py", "plot_cost_to_go"): {
        "renderer",
        "show",
        "ax",
    },
    ("planning/dynamic_programming/plotting.py", "plot_cost_to_go"): {"renderer"},
    ("planning/dynamic_programming/plotting.py", "plot_control_law"): {"renderer"},
    ("planning/dynamic_programming/plotting.py", "plot_value"): {"renderer"},
    ("planning/dynamic_programming/plotting.py", "plot_value_3d"): {"renderer", "ax"},
    ("planning/dynamic_programming/plotting.py", "plot_control_law_field"): {
        "renderer"
    },
    ("planning/dynamic_programming/plotting.py", "plot_field"): {"renderer"},
    ("planning/planner.py", "plot_trajectory"): {"renderer", "show", "ax"},
    ("planning/planner.py", "plot_cost"): {"renderer", "show", "ax"},
    ("planning/plotting.py", "plot_solution_trajectory"): {"ax"},
    ("planning/results.py", "plot_trajectory"): {"ax"},
    ("planning/results.py", "plot_cost"): {"ax"},
    ("planning/search/plotting.py", "plot_tree"): {"renderer"},
    ("planning/search/rrt.py", "plot_tree"): {"renderer", "show", "ax"},
    ("planning/spatial/plotting.py", "plot_scene"): {"renderer"},
    ("planning/spatial/plotting.py", "plot_track"): {"renderer"},
    ("planning/spatial/plotting.py", "plot_cost_field"): {"renderer"},
    ("planning/spatial/plotting.py", "plot_cost_field_3d"): {"renderer"},
    ("planning/spatial/plotting.py", "plot_cost_field_exports"): {"renderer", "ax"},
    ("planning/trajectory_optimization/mpc/viz.py", "plot_sampled_futures"): {
        "renderer"
    },
}


def plot_functions():
    """Every ``plot_*`` function and method of the library: ``(path, name, keywords)``."""
    found = []
    for path in sorted(ROOT.rglob("*.py")):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef):
                continue
            if not node.name.startswith("plot_") or node.name in NOT_FIGURES:
                continue
            args = node.args.args + node.args.kwonlyargs
            keywords = {arg.arg for arg in args}
            found.append((path.relative_to(ROOT).as_posix(), node.name, keywords))
    return found


@pytest.mark.parametrize(
    "path, name, keywords",
    [(path, name, sorted(keywords)) for path, name, keywords in plot_functions()],
    ids=[f"{path}::{name}" for path, name, _ in plot_functions()],
)
def test_a_plot_takes_renderer_show_and_ax(path, name, keywords):
    missing = {"renderer", "show", "ax"} - set(keywords)
    assert missing == NOT_YET.get((path, name), set()), (
        f"{path}::{name} lacks {sorted(missing)}; a plot takes renderer=, show= "
        "and ax= (a name that now takes them all leaves NOT_YET)"
    )
