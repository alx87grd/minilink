"""
Base classes for deterministic planners.

Planner classes own the numerical method and return a
:class:`~minilink.planning.solutions.PlanningSolution`. The
:class:`~minilink.planning.problems.PlanningProblem` remains declarative
and does not solve itself.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from minilink.planning.problems import PlanningProblem
from minilink.planning.solutions import PlanningSolution


class Planner(ABC):
    """
    Mother class for deterministic planners.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.
    """

    def __init__(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.last_solution: PlanningSolution | None = None

    @abstractmethod
    def compute_solution(self) -> PlanningSolution:
        """Compute and return a planning solution."""
        ...

    def _store_solution(self, solution: PlanningSolution) -> PlanningSolution:
        self.last_solution = solution
        return solution

    def require_solution(self) -> PlanningSolution:
        """Return the latest solution or raise a clear error."""
        if self.last_solution is None:
            raise ValueError("No solution has been computed yet")
        return self.last_solution

    def require_cost(self):
        """Return ``problem.cost`` or raise a solver-facing error."""
        return self.problem.require_cost()

    def require_goal(self):
        """Return ``problem.Xf`` or raise a solver-facing error."""
        return self.problem.require_goal()


class TrajectoryPlanner(Planner):
    """
    Base class for planners that can produce nominal trajectories.

    Examples include direct collocation, shooting methods, and future
    trajectory generators that consume a :class:`PlanningProblem`.
    """

    def plot_solution(self, *, plot: str = "xu"):
        """Plot the latest trajectory solution with the problem system."""
        solution = self.require_solution()
        return self.problem.sys.plot_trajectory(solution.require_traj(), plot=plot)

    def animate_solution(self, **kwargs):
        """Animate the latest trajectory solution with the problem system."""
        solution = self.require_solution()
        return self.problem.sys.animate(solution.require_traj(), **kwargs)


class SearchPlanner(Planner):
    """
    Base class for deterministic search-style planners.

    Search planners use feasibility, sampling, and rollout/steering
    settings. They may omit a cost and still return a path or trajectory.
    """


class PolicyPlanner(Planner):
    """
    Base class for planners that compute policies or value functions.

    Dynamic programming is the first expected consumer. Policy planners
    typically require a cost, but concrete subclasses decide when to
    validate that requirement.
    """

    def require_policy_solution(self) -> PlanningSolution:
        """Return the latest solution and ensure it contains a policy."""
        solution = self.require_solution()
        if solution.policy is None:
            raise ValueError("The latest solution does not contain a policy")
        return solution
