"""Every public plot takes renderer=, show= and ax=: an AST walk over the library."""

import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2] / "minilink"

# The plot_ name that draws nothing: a choice of plot space
NOT_FIGURES = {"plot_space_of"}

# The block diagram is outside the contract: Graphviz lays out and draws the
# whole graph itself, so it takes no axes to draw into and no other renderer
DIAGRAM_EXEMPTION = {"renderer", "ax"}
EXEMPT = {
    ("core/facades/plotting.py", "plot_diagram"): DIAGRAM_EXEMPTION,
    ("graphical/diagrams/dot.py", "plot_diagram"): DIAGRAM_EXEMPTION,
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
    assert missing == EXEMPT.get((path, name), set()), (
        f"{path}::{name} lacks {sorted(missing)}; a plot takes renderer=, show= and ax="
    )


# The modules that draw with a renderer's own library: the renderers, the show
# and save steps, and the matplotlib-only animations and live figures
RENDERER_MODULES = {
    "graphical/figure_renderers/matplotlib.py",
    "graphical/figure_renderers/plotly.py",
    "graphical/common/environment.py",
    "graphical/common/show.py",
    "graphical/common/plot_result.py",
    "graphical/colormaps.py",
    "graphical/live_plots.py",
    "graphical/animation/grid_animations.py",
    "graphical/animation/search_animations.py",
    "graphical/animation/renderers/matplotlib_renderer.py",
    "graphical/animation/renderers/plotly_renderer.py",
    "graphical/animation/renderers/meshcat_renderer.py",
    "graphical/animation/renderers/pygame_renderer.py",
    "graphical/signals/matplotlib_backend.py",
    "graphical/signals/plotly_backend.py",
}

DRAWING_LIBRARIES = ("matplotlib", "plotly", "mpl_toolkits")


def drawing_imports():
    """``(path, module)`` of every import of a drawing library under the library."""
    found = []
    for path in sorted(ROOT.rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            else:
                continue
            for name in names:
                if name.split(".")[0] in DRAWING_LIBRARIES:
                    found.append((path.relative_to(ROOT).as_posix(), name))
    return found


def test_only_the_renderers_import_a_drawing_library():
    outside = sorted(
        {
            (path, name)
            for path, name in drawing_imports()
            if path not in RENDERER_MODULES
        }
    )
    assert outside == [], (
        "a plot builds a FigureSpec and draws it with minilink.graphical.draw; "
        f"these modules import a drawing library themselves: {outside}"
    )


def test_every_renderer_module_still_draws():
    importing = {path for path, _ in drawing_imports()}
    assert RENDERER_MODULES <= importing, sorted(RENDERER_MODULES - importing)
