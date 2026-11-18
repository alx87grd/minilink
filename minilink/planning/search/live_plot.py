"""Live search-tree plot: the RRT family's tree and best path, redrawn as the search grows."""

from dataclasses import dataclass

__all__ = ["LiveSearchPlot", "TreeSearchIteration"]

###############################################################################
# Public API
###############################################################################


class LiveSearchPlot:
    """
    Live figure of an RRT-family search, a callback of the planner's ``solve``.

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

        # Lazy import: the graphics load with the first redraw
        from minilink.graphical.live_plots import redraw_search

        redraw_search(self, step)


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
    best_cost_to_come : float or None
        Best goal cost-to-come when ``reached_goal``; otherwise ``None``.
    n_nodes : int
        Current tree size.
    planner : RRTPlanner
        Planner whose ``tree`` and ``solution_node`` reflect the live state.
    """

    iteration: int
    phase: str
    reached_goal: bool
    best_cost_to_come: float | None
    n_nodes: int
    planner: object
