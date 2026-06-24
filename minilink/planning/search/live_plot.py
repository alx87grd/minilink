"""
Live search-tree visualization during RRT / RRT* planning.

:class:`LiveSearchPlotCallback` redraws the current tree (including RRT*
rewiring) on each successful extension, mirroring pyro's live RRT display and
:class:`~minilink.planning.trajectory_optimization.live_plot.LiveTrajectoryPlotCallback`.
"""

from dataclasses import dataclass

# Public API


@dataclass
class RRSearchIteration:
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
    tree_nodes : int
        Current tree size.
    planner : RRTPlanner
        Planner whose ``tree`` and ``solution_node`` reflect the live state.
    """

    iteration: int
    phase: str
    reached_goal: bool
    best_cost: float | None
    tree_nodes: int
    planner: object


class LiveSearchPlotCallback:
    """
    Matplotlib live updater for the RRT family search loop.

    Parameters
    ----------
    planner : RRTPlanner
        Planner being solved; read on each callback invocation.
    ax : matplotlib Axes, optional
        Axes to draw on. When omitted, a new figure is opened on the first update.
    x_axis, y_axis : int
        State indices for the phase-plane projection.
    every : int
        Update every ``every`` successful extensions.
    pause : float
        ``plt.pause`` duration after each redraw (matplotlib only).
    after_goal_only : bool
        When ``True``, skip updates until the first goal connection (RRT*
        convergence phase only if ``optimize_after_goal`` is also set).
    tree_color, path_color : str
        Edge and best-path colours.
    """

    def __init__(
        self,
        planner,
        *,
        ax=None,
        x_axis: int = 0,
        y_axis: int = 1,
        every: int = 1,
        pause: float = 0.001,
        after_goal_only: bool = False,
        tree_color="0.75",
        path_color="tab:blue",
    ) -> None:
        self.planner = planner
        self.ax = ax
        self.x_axis = int(x_axis)
        self.y_axis = int(y_axis)
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.after_goal_only = bool(after_goal_only)
        self.tree_color = tree_color
        self.path_color = path_color
        self.fig = None
        self._tree_lines: list = []
        self._path_line = None
        self._title = None
        self._limits_set = False

    @property
    def lines(self):
        """Tree edge line artists from the latest redraw."""
        return list(self._tree_lines)

    def __call__(self, step: RRSearchIteration) -> None:
        if step.iteration % self.every != 0:
            return
        if self.after_goal_only and not step.reached_goal:
            return

        import matplotlib.pyplot as plt

        from minilink.planning.search.plotting import (
            _clear_search_artists,
            _draw_search_state,
            _label_axes,
            _mark_endpoints,
            _search_title,
            _set_limits,
        )

        if self.ax is None:
            self.fig, self.ax = plt.subplots(figsize=(7.0, 6.0), frameon=True)
        else:
            self.fig = self.ax.figure

        if not self._limits_set:
            _set_limits(self.ax, self.planner, self.x_axis, self.y_axis)
            _label_axes(self.ax, self.planner.problem.sys, self.x_axis, self.y_axis)
            _mark_endpoints(self.ax, self.planner, self.x_axis, self.y_axis)
            self._limits_set = True

        _clear_search_artists(self._tree_lines, self._path_line)
        self._path_line = None
        self._tree_lines, self._path_line = _draw_search_state(
            self.ax,
            self.planner,
            x_axis=self.x_axis,
            y_axis=self.y_axis,
            tree_color=self.tree_color,
            path_color=self.path_color,
        )

        title = _search_title(step)
        if self._title is None:
            self._title = self.ax.set_title(title)
        else:
            self._title.set_text(title)

        if self.pause > 0.0:
            plt.pause(self.pause)
