"""
Base class for trajectory-optimization planners.

Trajectory-optimization planners compute nominal state-input trajectories
from optimal-control problems. Plotting and animation delegate to the
problem system once a trajectory result exists.
"""

from __future__ import annotations

from minilink.planning.base import Planner
from minilink.planning.trajectory_optimization.results import (
    TrajectoryOptimizationResult,
)


class TrajectoryOptimizationPlanner(Planner[TrajectoryOptimizationResult]):
    """
    Base class for planners that optimize nominal trajectories.

    Examples include direct collocation, shooting methods, and future
    trajectory generators that consume a planning problem.
    """

    def plot_solution(self, *, plot: str = "xu"):
        """Plot the latest trajectory result with the problem system."""
        result = self.require_result()
        return self.problem.sys.plot_trajectory(result.require_traj(), plot=plot)

    def animate_solution(self, **kwargs):
        """Animate the latest trajectory result with the problem system."""
        result = self.require_result()
        return self.problem.sys.animate(result.require_traj(), **kwargs)
