"""Search animations in matplotlib: the RRT family's tree growing, and RRT*'s recorded convergence."""

from minilink.graphical.common.animation_result import animation_result_of
from minilink.planning.search.plotting import (
    PATH_COLOR,
    TREE_COLOR,
    axis_label,
    endpoint_layers,
    goal_region_layers,
    require_history,
    require_tree,
    tree_limits,
)

# The grey of the tree while it rewires
LIGHT_TREE_COLOR = "#bfbfbf"


###############################################################################
# Public API
###############################################################################


def animate_search(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    ax=None,
    step: int = 5,
    interval: int = 30,
    show: bool = True,
    figsize=(6.0, 5.0),
):
    """
    Replay the tree growth as an animation in the ``(x_axis, y_axis)`` projection.

    ``step`` edges are revealed per frame, in node insertion order; the solution
    path is drawn at the end. Returns the
    :class:`AnimationResult`; ``.save("tree.gif")`` writes it.
    """
    # Lazy import: matplotlib loads with the first animation
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    tree = require_tree(planner)
    solution = planner.last_solution

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    # The frame: the limits, labels, endpoints and title stay fixed while the tree grows
    frame_search_axes(ax, planner, x_axis, y_axis)
    ax.set_title("RRT search")

    # The edges in insertion order, and the lines drawn so far
    edges = [node for node in tree.nodes if node.parent is not None]
    drawn: list = []

    def update(frame):
        # Reveal up to step more edges, in insertion order
        target = min(len(edges), (frame + 1) * step)
        while len(drawn) < target:
            states = edges[len(drawn)].edge.states
            (line,) = ax.plot(
                states[:, x_axis], states[:, y_axis], color=TREE_COLOR, lw=0.4, zorder=1
            )
            drawn.append(line)

        # Once every edge is shown, the solution path
        if target >= len(edges) and solution is not None:
            traj = solution.trajectory
            ax.plot(traj.x[x_axis], traj.x[y_axis], color=PATH_COLOR, lw=2.5, zorder=3)

        return drawn

    # One frame per batch of step edges, and a last one for the path
    n_frames = (len(edges) + step - 1) // step + 1
    anim = FuncAnimation(
        fig, update, frames=n_frames, interval=interval, blit=False, repeat=False
    )
    return animation_result_of(fig, anim, fps=max(1, int(1000 / interval)), show=show)


def animate_convergence(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    ax=None,
    interval: int = 50,
    show: bool = True,
    figsize=(7.0, 6.0),
    title_prefix: str = "RRT* convergence",
):
    """
    Animate recorded RRT* history showing tree growth and best-path refinement.

    Requires the latest solution's ``record.history``, from a solve with
    ``record_history=True``.
    Each frame redraws the full tree (including rewiring) and highlights the
    current best goal path. Returns the :class:`AnimationResult`.
    """
    # Lazy import: matplotlib loads with the first animation
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    history = require_history(planner)

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    # The frame: the limits, labels and endpoints stay fixed while the tree rewires
    frame_search_axes(ax, planner, x_axis, y_axis)

    # The lines of the current frame, and the title that reports its progress
    tree_lines: list = []
    path_line = None
    title = ax.set_title(title_prefix)

    def update(frame):
        nonlocal path_line
        snapshot = history[frame]

        # Clear the previous frame's tree and path
        clear_search_artists(tree_lines, path_line)
        path_line = None

        # The tree as recorded, rewired edges included
        tree_lines.extend(
            draw_edge_lines(ax, snapshot.tree_edges, x_axis, y_axis, lw=0.35)
        )

        # The best path to the goal so far
        if snapshot.path_states is not None:
            path_line = draw_path_line(ax, snapshot.path_states, x_axis, y_axis)

        # The iteration, and the best cost once the goal is reached
        if snapshot.reached_goal and snapshot.best_cost_to_come is not None:
            title.set_text(
                f"{title_prefix}  iter={snapshot.iteration}  "
                f"best cost={snapshot.best_cost_to_come:.2f}"
            )
        else:
            title.set_text(f"{title_prefix}  iter={snapshot.iteration}  searching...")

        return tree_lines + ([path_line] if path_line is not None else [])

    # One frame per recorded snapshot, looping
    anim = FuncAnimation(
        fig,
        update,
        frames=len(history),
        interval=interval,
        blit=False,
        repeat=True,
    )
    return animation_result_of(fig, anim, fps=max(1, int(1000 / interval)), show=show)


###############################################################################
# Internal machinery
###############################################################################


def frame_search_axes(ax, planner, x_axis, y_axis):
    """The fixed frame of a search drawing: the limits, the labels, the goal region and the endpoints."""
    # Lazy import: the matplotlib renderer loads with the first drawing
    from minilink.graphical.figure_renderers.matplotlib import draw_trace

    x_lim, y_lim = tree_limits(planner, x_axis, y_axis)
    ax.set_xlim(*x_lim)
    ax.set_ylim(*y_lim)
    ax.set_xlabel(axis_label(planner.problem.sys, x_axis))
    ax.set_ylabel(axis_label(planner.problem.sys, y_axis))
    for layer in goal_region_layers(planner, x_axis, y_axis):
        draw_trace(ax, layer)
    for layer in endpoint_layers(planner, x_axis, y_axis):
        draw_trace(ax, layer)


def draw_edge_lines(ax, edges, x_axis, y_axis, *, lw):
    """One light grey line per edge's states; the lines, for a later redraw to clear."""
    lines = []
    for states in edges:
        (line,) = ax.plot(
            states[:, x_axis],
            states[:, y_axis],
            color=LIGHT_TREE_COLOR,
            lw=lw,
            zorder=1,
        )
        lines.append(line)

    return lines


def draw_path_line(ax, path_states, x_axis, y_axis):
    """The best path's states as one wide line; the line."""
    (line,) = ax.plot(
        path_states[:, x_axis],
        path_states[:, y_axis],
        color=PATH_COLOR,
        lw=2.5,
        zorder=3,
    )

    return line


def clear_search_artists(tree_lines, path_line):
    """Remove the tree and path lines of the previous redraw."""
    for line in list(tree_lines):
        line.remove()
    tree_lines.clear()

    if path_line is not None:
        path_line.remove()
