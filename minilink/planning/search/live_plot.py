"""Live search-tree plot: the RRT family's tree and best path, redrawn as the search grows."""

from dataclasses import dataclass

###############################################################################
# Public API
###############################################################################


@dataclass
class TreeSearchIteration:
    """
    One successful tree extension during a search solve.

    Parameters
    ----------
    iteration : int
        Number of successful extensions so far.
    phase : {"explore", "optimize"}
        ``explore`` before / without post-goal refinement; ``optimize`` during
        RRT* convergence after the first goal connection.
    reached_goal : bool
        Whether a goal-region node exists in the tree.
    best_cost : float or None
        Best goal cost-to-come when ``reached_goal``; otherwise ``None``.
    n_nodes : int
        Current tree size.
    planner : RRTPlanner
        Planner whose ``tree`` and ``solution_node`` reflect the live state.
    """

    iteration: int
    phase: str
    reached_goal: bool
    best_cost: float | None
    n_nodes: int
    planner: object


class LiveSearchPlot:
    """
    Matplotlib live figure of an RRT-family search, a callback of the planner's ``solve``.

    Redraws the current tree (RRT* rewiring included) and the best path as the
    search grows, mirroring pyro's live RRT display and
    :class:`~minilink.planning.trajectory_optimization.live_plot.LiveTrajectoryPlotCallback`::

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
    tree_color, path_color : str
        Edge and best-path colours.
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
        tree_color="0.75",
        path_color="tab:blue",
    ) -> None:
        # When to redraw
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.after_goal_only = bool(after_goal_only)

        # The axes to draw on, and the state projection
        self.ax = ax
        self.x_axis = int(x_axis)
        self.y_axis = int(y_axis)

        # Graphics
        self.tree_color = tree_color
        self.path_color = path_color

        # Memory: the figure, and the artists of the latest redraw
        self.fig = None
        self._tree_lines: list = []
        self._path_line = None
        self._title = None
        self._limits_set = False

    @property
    def lines(self):
        """Tree edge line artists from the latest redraw."""
        return list(self._tree_lines)

    def __call__(self, step: TreeSearchIteration) -> None:
        # One redraw every few extensions, and none before the goal when asked
        if step.iteration % self.every != 0:
            return
        if self.after_goal_only and not step.reached_goal:
            return

        import matplotlib.pyplot as plt

        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.plotting import (
            clear_search_artists,
            draw_search_state,
            label_axes,
            mark_endpoints,
            search_title,
            set_limits,
        )

        # A new figure on the first update, or the one that holds the given axes
        if self.ax is None:
            self.fig, self.ax = plt.subplots(figsize=(7.0, 6.0), frameon=True)
        else:
            self.fig = self.ax.figure

        # The frame, set once: the limits, labels and endpoints stay fixed
        planner = step.planner
        if not self._limits_set:
            set_limits(self.ax, planner, self.x_axis, self.y_axis)
            label_axes(self.ax, planner.problem.sys, self.x_axis, self.y_axis)
            mark_endpoints(self.ax, planner, self.x_axis, self.y_axis)
            self._limits_set = True

        # The tree and the best path, redrawn in place of the previous ones
        clear_search_artists(self._tree_lines, self._path_line)
        self._path_line = None
        self._tree_lines, self._path_line = draw_search_state(
            self.ax,
            planner,
            x_axis=self.x_axis,
            y_axis=self.y_axis,
            tree_color=self.tree_color,
            path_color=self.path_color,
        )

        # The title reports the search's progress
        title = search_title(step)
        if self._title is None:
            self._title = self.ax.set_title(title)
        else:
            self._title.set_text(title)

        # A short pause lets the figure redraw
        if self.pause > 0.0:
            plt.pause(self.pause)
