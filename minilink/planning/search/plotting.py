"""Search-tree plots: the RRT family's tree and solution path in a state projection."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from minilink.graphical.common.animation_result import require_matplotlib_animation
from minilink.graphical.common.labels import state_axis_label
from minilink.graphical.figure import FigureSpec, Panel, Trace, draw

if TYPE_CHECKING:
    from minilink.graphical.common.plot_result import PlotResult

# The tree's edges and the solution path
TREE_COLOR = "#999999"
PATH_COLOR = "#1f77b4"


###############################################################################
# Public API
###############################################################################


def plot_tree(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    title: str | None = None,
    path_style: str = "dense",
    figsize=None,
    renderer="matplotlib",
    show=True,
    ax=None,
) -> PlotResult:
    """
    Plot the final tree and solution in the ``(x_axis, y_axis)`` state projection.

    Tree edges are grey, the solution path is highlighted, and the start, goal,
    and goal region are marked. ``path_style="waypoints"`` draws the path
    through the tree's nodes, ``"dense"`` along its trajectory. Reached through
    ``RRTPlanner.plot_tree``; pass ``ax`` (e.g. the ``axes`` of
    ``scene.plot(...)``'s record) to overlay the tree on a scene.
    """
    spec = tree_figure(
        planner,
        planner.last_solution,
        x_axis=x_axis,
        y_axis=y_axis,
        title=title,
        path_style=path_style,
        figsize=figsize or (6.0, 5.0),
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def animate_search(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    step: int = 5,
    interval: int = 30,
    figsize=(6.0, 5.0),
    renderer="matplotlib",
    show=True,
    ax=None,
):
    """
    Replay the tree growth as an animation in the ``(x_axis, y_axis)`` projection.

    ``step`` edges are revealed per frame, in node insertion order; the solution
    path is drawn at the end. Drawn with matplotlib, on a new figure or on ``ax``, whose figure is then
    its owner's to show; ``show=False`` builds it without playing it. Returns the
    :class:`AnimationResult`; ``.save("tree.gif")`` writes it.
    """
    # Lazy import: the graphics load on first use, not with the planners
    from minilink.graphical.animation.line_animations import animate_lines

    require_matplotlib_animation(renderer)

    tree = require_tree(planner)
    solution = planner.last_solution

    # The edges in insertion order, and the solution path once every edge is shown
    edges = [node.edge.states for node in tree.nodes if node.parent is not None]
    path = None if solution is None else np.asarray(solution.trajectory.x).T

    def frame_of(frame):
        shown = min(len(edges), (frame + 1) * step)
        under = (edge_trace(edges[:shown], x_axis, y_axis, width=0.4),)
        done = shown >= len(edges) and path is not None
        over = (path_trace(path, x_axis, y_axis),) if done else ()
        return under, over, "RRT search"

    # One frame per batch of step edges, and a last one for the path
    n_frames = (len(edges) + step - 1) // step + 1

    return animate_lines(
        search_frame(planner, x_axis, y_axis, title="RRT search"),
        frame_of,
        n_frames,
        interval=interval,
        repeat=False,
        show=show,
        ax=ax,
        figsize=figsize,
    )


def animate_convergence(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    interval: int = 50,
    figsize=(7.0, 6.0),
    title_prefix: str = "RRT* convergence",
    renderer="matplotlib",
    show=True,
    ax=None,
):
    """
    Animate recorded RRT* history showing tree growth and best-path refinement.

    Requires the latest solution's ``record.history``, from a solve with
    ``record_history=True``.
    Each frame redraws the full tree (including rewiring) and highlights the
    current best goal path. Drawn with matplotlib, on a new figure or on ``ax``, whose figure is then
    its owner's to show; ``show=False`` builds it without playing it. Returns the
    :class:`AnimationResult`.
    """
    # Lazy import: the graphics load on first use, not with the planners
    from minilink.graphical.animation.line_animations import animate_lines

    require_matplotlib_animation(renderer)

    history = require_history(planner)

    # Each snapshot: its tree as recorded (rewired edges included), its best path,
    # and the iteration with the best cost once the goal is reached
    def frame_of(frame):
        snapshot = history[frame]
        under = (edge_trace(snapshot.tree_edges, x_axis, y_axis, width=0.35),)
        over = ()
        if snapshot.path_states is not None:
            over = (path_trace(snapshot.path_states, x_axis, y_axis),)
        progress = "searching..."
        if snapshot.reached_goal and snapshot.best_cost_to_come is not None:
            progress = f"best cost={snapshot.best_cost_to_come:.2f}"
        return under, over, f"{title_prefix}  iter={snapshot.iteration}  {progress}"

    return animate_lines(
        search_frame(planner, x_axis, y_axis, title=title_prefix),
        frame_of,
        len(history),
        interval=interval,
        repeat=True,
        show=show,
        ax=ax,
        figsize=figsize,
    )


def tree_figure(planner, solution, *, x_axis, y_axis, title, path_style, figsize):
    """The search tree, the solution path and the endpoints, as a description."""
    tree = require_tree(planner)

    # The tree: each edge along the states its extension swept, one NaN-broken line
    edges = [node.edge.states for node in tree.nodes if node.parent is not None]
    layers = [edge_trace(edges, x_axis, y_axis, width=0.4)]

    # The goal region, then the solution path: through the tree's nodes, or along the trajectory
    layers += goal_region_layers(planner, x_axis, y_axis)
    if solution is not None:
        path = solution_path(planner, solution, path_style)
        if path is not None:
            layers.append(
                Trace(
                    path[:, x_axis],
                    path[:, y_axis],
                    color=PATH_COLOR,
                    width=2.5,
                    label="path",
                )
            )

    # The start and the goal on top
    layers += endpoint_layers(planner, x_axis, y_axis)
    panel = Panel(
        layers=tuple(layers),
        title=f"RRT tree ({len(tree.nodes)} nodes)" if title is None else title,
        x_label=state_axis_label(planner.problem.sys, x_axis),
        y_label=state_axis_label(planner.problem.sys, y_axis),
    )

    return FigureSpec(panels=(panel,), figsize=figsize)


def edge_trace(edges, x_axis, y_axis, *, width, color=TREE_COLOR):
    """Every edge's states in the ``(x_axis, y_axis)`` projection, as one line broken by NaN."""
    if not edges:
        return Trace(np.array([]), np.array([]), color=color, width=width)
    gap = np.full(1, np.nan)
    x = np.concatenate([np.r_[np.asarray(s, float)[:, x_axis], gap] for s in edges])
    y = np.concatenate([np.r_[np.asarray(s, float)[:, y_axis], gap] for s in edges])

    return Trace(x[:-1], y[:-1], color=color, width=width)


def edge_traces(edges, x_axis, y_axis, *, width, color=TREE_COLOR):
    """Each edge's states in the ``(x_axis, y_axis)`` projection, one line per edge."""
    return tuple(
        Trace(states[:, x_axis], states[:, y_axis], color=color, width=width)
        for states in edges
    )


def path_trace(states, x_axis, y_axis):
    """A path's states in the ``(x_axis, y_axis)`` projection, as one wide line."""
    return Trace(states[:, x_axis], states[:, y_axis], color=PATH_COLOR, width=2.5)


def search_frame(planner, x_axis, y_axis, *, title=""):
    """The fixed frame of a search drawing, as a panel: the title, the limits, the labels, the goal region and the endpoints."""
    x_lim, y_lim = tree_limits(planner, x_axis, y_axis)
    layers = goal_region_layers(planner, x_axis, y_axis)
    layers += endpoint_layers(planner, x_axis, y_axis)

    return Panel(
        layers=tuple(layers),
        title=title,
        x_label=state_axis_label(planner.problem.sys, x_axis),
        y_label=state_axis_label(planner.problem.sys, y_axis),
        x_lim=x_lim,
        y_lim=y_lim,
    )


def endpoint_layers(planner, x_axis, y_axis):
    """The start (a black square) and the goal (a red star), as layers."""
    problem = planner.problem
    x0 = np.asarray(problem.x0, dtype=float)
    layers = [
        Trace(
            [x0[x_axis]],
            [x0[y_axis]],
            color="black",
            mode="markers",
            marker="s",
            label="start",
        )
    ]
    if problem.Xf is not None:
        xg = problem.goal_point()
        layers.append(
            Trace(
                [xg[x_axis]],
                [xg[y_axis]],
                color="red",
                mode="markers",
                marker="*",
                size=13.0,
                label="goal",
            )
        )

    return layers


def goal_region_layers(planner, x_axis, y_axis):
    """The dashed circle of a ball goal region, as layers; none for another goal."""
    problem = planner.problem
    radius = goal_radius(planner)
    if problem.Xf is None or radius is None:
        return []
    xg = problem.goal_point()

    # The circle c + r (cos φ, sin φ)
    phi = np.linspace(0.0, 2.0 * np.pi, 101)
    x = xg[x_axis] + radius * np.cos(phi)
    y = xg[y_axis] + radius * np.sin(phi)

    return [Trace(x, y, color="red", width=1.0, dash="dash")]


def tree_limits(planner, x_axis, y_axis):
    """The ``(x_lim, y_lim)`` around the tree's nodes and the goal, with a 5 % margin."""
    pts = np.array([node.x for node in planner.tree.nodes], dtype=float)
    if planner.problem.Xf is not None:
        pts = np.vstack([pts, planner.problem.goal_point()])

    limits = []
    for axis in (x_axis, y_axis):
        lo = float(pts[:, axis].min())
        hi = float(pts[:, axis].max())
        pad = 0.05 * (hi - lo or 1.0)
        limits.append((lo - pad, hi + pad))

    return tuple(limits)


def search_title(step):
    """The live plot's title for one search step."""
    if step.reached_goal and step.best_cost_to_come is not None:
        return (
            f"RRT search  phase={step.phase}  iter={step.iteration}  "
            f"nodes={step.n_nodes}  best cost={step.best_cost_to_come:.2f}"
        )

    return (
        f"RRT search  phase={step.phase}  iter={step.iteration}  "
        f"nodes={step.n_nodes}  searching..."
    )


def best_path_states(planner, reached_goal):
    """The states along the path to the planner's goal node, or ``None`` before the goal."""
    node = getattr(planner, "solution_node", None)
    if node is None or not reached_goal:
        return None

    if hasattr(planner, "path_states"):
        return planner.path_states(node)

    return None


def require_tree(planner):
    """Refuse a planner with no search tree yet; return its tree."""
    if getattr(planner, "tree", None) is None:
        raise ValueError("No search tree yet; call solve() first")

    return planner.tree


def require_history(planner):
    """Refuse a planner whose latest solution recorded no search history; return that history."""
    # Lazy import: the planner base loads with the planners, the plots after them
    from minilink.planning.planner import require_solution

    history = require_solution(planner).record.history
    if not history:
        raise ValueError(
            "No search history recorded; run solve with record_history=True"
        )

    return history


###############################################################################
# Internal machinery
###############################################################################


def solution_path(planner, solution, path_style):
    """The solution's states, one row per point: the tree's nodes, or the dense trajectory."""
    if path_style == "waypoints":
        return solution_waypoints(planner)

    return np.asarray(solution.trajectory.x, dtype=float).T


def solution_waypoints(planner):
    """Tree-node polyline from root to ``solution_node`` (no edge subsampling)."""
    node = getattr(planner, "solution_node", None)
    tree = getattr(planner, "tree", None)
    if node is None or tree is None:
        return None

    # The chain of nodes from the goal node back to the root, then reversed
    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
    chain.reverse()

    # The root, then each node's state along the chain
    points = [np.asarray(tree.root.x, dtype=float)]
    for nd in chain:
        points.append(np.asarray(nd.x, dtype=float))
    waypoints = np.asarray(points)

    return waypoints


def goal_radius(planner):
    """The radius of a ball goal region, else ``None``."""
    # Lazy import: the core sets load on first use, not with the plots
    from minilink.core.sets import BallSet

    Xf = planner.problem.Xf
    if isinstance(Xf, BallSet):
        return float(Xf.radius)

    return None
