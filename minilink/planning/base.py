"""
Base classes shared by deterministic planning families.

Planner classes own the numerical method and return a family-specific
result object. The :class:`~minilink.planning.problems.PlanningProblem`
remains declarative and does not solve itself.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from minilink.planning.costs import CostFunction
from minilink.planning.problems import PlanningProblem
from minilink.planning.sets import Set

ResultT = TypeVar("ResultT")


class Planner(ABC, Generic[ResultT]):
    """
    Mother class for deterministic planners.

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
