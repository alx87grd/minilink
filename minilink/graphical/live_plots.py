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


class LiveSearchPlot:
    """
    Matplotlib live figure of an RRT-family search, a callback of the planner's ``solve``.

    Redraws the current tree (RRT* rewiring included) and the best path as the
    search grows::

        planner.solve(callback=LiveSearchPlot(every=10, after_goal_only=True))

    It filters the extensions for itself: a callback beside it in the same
    ``solve(callback=[...])`` still sees every extension.

    Parameters
    ----------
    every : int
        Redraw every ``every`` successful extensions.
    pause : float
        ``plt.pause`` duration after each redraw (matplotlib only).
    after_goal_only : bool
        When ``True``, skip redraws until the first goal connection (RRT*
        convergence phase only if ``optimize_after_goal`` is also set).
    ax : matplotlib Axes, optional
        Axes to draw on. When omitted, a new figure is opened on the first redraw.
    x_axis, y_axis : int
        State indices for the phase-plane projection.
    """

    def __init__(
        self,
        every: int = 1,
        pause: float = 0.001,
        after_goal_only: bool = False,
        ax=None,
        *,
        x_axis: int = 0,
        y_axis: int = 1,
    ) -> None:
        # When to redraw
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.after_goal_only = bool(after_goal_only)

        # The axes to draw on, and the state projection
        self.ax = ax
        self.x_axis = int(x_axis)
        self.y_axis = int(y_axis)

        # Memory: the figure, and the artists of the latest redraw
        self.fig = None
        self.tree_lines: list = []
        self.path_line = None
        self.title = None
        self.framed = False

    @property
    def lines(self):
        """Tree edge line artists from the latest redraw."""
        return list(self.tree_lines)

    def __call__(self, step) -> None:
        # One redraw every few extensions, and none before the goal when asked
        if step.iteration % self.every != 0:
            return
        if self.after_goal_only and not step.reached_goal:
            return

        # Lazy import: matplotlib loads with the first redraw
        import matplotlib.pyplot as plt

        # A new figure on the first update, or the one that holds the given axes
        if self.ax is None:
            self.fig, self.ax = plt.subplots(figsize=(7.0, 6.0), frameon=True)
        else:
            self.fig = self.ax.figure

        # The frame, set once: the limits, labels and endpoints stay fixed
        planner = step.planner
        if not self.framed:
            frame_search_axes(self.ax, planner, self.x_axis, self.y_axis)
            self.framed = True

        # The tree and the best path, redrawn in place of the previous ones
        clear_search_artists(self.tree_lines, self.path_line)
        self.path_line = None
        edges = [
            node.edge.states for node in planner.tree.nodes if node.parent is not None
        ]
        self.tree_lines = draw_edge_lines(
            self.ax, edges, self.x_axis, self.y_axis, lw=0.35
        )
        path_states = best_path_states(planner, step.reached_goal)
        if path_states is not None:
            self.path_line = draw_path_line(
                self.ax, path_states, self.x_axis, self.y_axis
            )

        # The title reports the search's progress
        title = search_title(step)
        if self.title is None:
            self.title = self.ax.set_title(title)
        else:
            self.title.set_text(title)

        pause_figures(self.pause)


def pause_figures(seconds):
    """Let the open matplotlib windows redraw for ``seconds``; nothing for 0."""
    # Lazy import: matplotlib loads with the first live figure
    import matplotlib.pyplot as plt

    if seconds > 0.0:
        plt.pause(seconds)
