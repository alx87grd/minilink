"""
Shared orchestration base for deterministic planners.

Concrete planners live in family subpackages (``search/``,
``trajectory_optimization/``, ``policy_synthesis/``). Each family picks a
result type—typically :class:`~minilink.core.trajectory.Trajectory` for
path and trajectory optimization, or :class:`~minilink.core.framework.StaticSystem`
for synthesized feedback policies. The
:class:`~minilink.planning.problems.PlanningProblem` remains declarative
and does not solve itself.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from minilink.core.trajectory import Trajectory
from minilink.planning.costs import CostFunction
from minilink.planning.problems import PlanningProblem
from minilink.planning.sets import Set

ResultT = TypeVar("ResultT")


class Planner(ABC, Generic[ResultT]):
    """
    Base class for deterministic planners.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.
    """

    def __init__(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.last_result: ResultT | None = None

    @abstractmethod
    def compute_solution(self) -> ResultT:
        """Compute and return a family-specific planning result."""
        ...

    def _store_result(self, result: ResultT) -> ResultT:
        self.last_result = result
        return result

    def require_result(self) -> ResultT:
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


class TrajectoryPlanner(Planner[Trajectory]):
    """
    Planner whose primary artifact is a state-input trajectory.

    Adds plotting and animation helpers that delegate to ``problem.sys``.
    """

    def plot_solution(self, *, plot: str = "xu"):
        """Plot the latest trajectory with the problem system."""
        return self.problem.sys.plot_trajectory(self.require_result(), plot=plot)

    def animate_solution(self, **kwargs):
        """Animate the latest trajectory with the problem system."""
        return self.problem.sys.animate(self.require_result(), **kwargs)
