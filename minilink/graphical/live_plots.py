"""Live plots in matplotlib: figures redrawn while a planner or a solver runs."""

from minilink.graphical.animation.search_animations import (
    clear_search_artists,
    draw_edge_lines,
    draw_path_line,
    frame_search_axes,
)
from minilink.planning.search.plotting import best_path_states, search_title

###############################################################################
# Public API
###############################################################################


def redraw_search(plot, step) -> None:
    """Redraw the tree and the best path of ``step`` on ``plot``'s axes, a new figure on the first call."""
    # Lazy import: matplotlib loads with the first redraw
    import matplotlib.pyplot as plt

    # A new figure on the first update, or the one that holds the given axes
    if plot.ax is None:
        plot.fig, plot.ax = plt.subplots(figsize=(7.0, 6.0), frameon=True)
    else:
        plot.fig = plot.ax.figure

    # The frame, set once: the limits, labels and endpoints stay fixed
    planner = step.planner
    if not plot.framed:
        frame_search_axes(plot.ax, planner, plot.x_axis, plot.y_axis)
        plot.framed = True

    # The tree and the best path, redrawn in place of the previous ones
    clear_search_artists(plot.tree_lines, plot.path_line)
    plot.path_line = None
    edges = [node.edge.states for node in planner.tree.nodes if node.parent is not None]
    plot.tree_lines = draw_edge_lines(plot.ax, edges, plot.x_axis, plot.y_axis, lw=0.35)
    path_states = best_path_states(planner, step.reached_goal)
    if path_states is not None:
        plot.path_line = draw_path_line(plot.ax, path_states, plot.x_axis, plot.y_axis)

    # The title reports the search's progress
    title = search_title(step)
    if plot.title is None:
        plot.title = plot.ax.set_title(title)
    else:
        plot.title.set_text(title)

    pause_figures(plot.pause)


def pause_figures(seconds):
    """Let the open matplotlib windows redraw for ``seconds``; nothing for 0."""
    # Lazy import: matplotlib loads with the first live figure
    import matplotlib.pyplot as plt

    if seconds > 0.0:
        plt.pause(seconds)
