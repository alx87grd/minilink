"""
Shared orchestration base for deterministic planners.

Concrete planners live in family subpackages (``search/``,
``trajectory_optimization/``, ``policy_synthesis/``). Result typing is
left to concrete planners and call sites for now.

The :class:`~minilink.planning.problems.PlanningProblem` remains
declarative and does not solve itself.
"""

from abc import ABC, abstractmethod

from minilink.core.costs import CostFunction
from minilink.core.sets import Set
from minilink.planning.problems import PlanningProblem


class Planner(ABC):
    """
    Base class for deterministic planners.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.
    """

    def __init__(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.last_result = None

    @abstractmethod
    def compute_solution(self):
        """Compute and return a planning result (shape defined by the concrete planner)."""
        ...

    def _store_result(self, result):
        self.last_result = result
        return result

    def require_result(self):
        """Return the latest result or raise a clear error."""
        if self.last_result is None:
            raise ValueError("No planning result has been computed yet")
        return self.last_result

    def require_cost(self) -> CostFunction:
        """Return ``problem.cost`` or raise a solver-facing error."""
        return self.problem.require_cost()

    def require_goal(self) -> Set:
        """Return ``problem.Xf`` or raise a solver-facing error."""
        return self.problem.require_goal()

    def plot_solution(self, *, plot: str = "xu"):
        """
        Plot the latest result as a trajectory with the problem system.

        Only valid when :meth:`require_result` returns a
        :class:`~minilink.core.trajectory.Trajectory`.
        """
        return self.problem.sys.plot_trajectory(self.require_result(), plot=plot)

    def animate_solution(self, **kwargs):
        """
        Animate the latest result with the problem system.

        Only valid when :meth:`require_result` returns a
        :class:`~minilink.core.trajectory.Trajectory`.
        """
        return self.problem.sys.animate(self.require_result(), **kwargs)
