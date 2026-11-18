"""Live search-tree plot: the RRT family's tree and best path, redrawn as the search grows."""

from dataclasses import dataclass

from minilink.graphical.live_plots import LiveSearchPlot

# The live figure itself is matplotlib's, under graphical/
__all__ = ["LiveSearchPlot", "TreeSearchIteration"]

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
