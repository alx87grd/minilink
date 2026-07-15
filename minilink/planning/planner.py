"""
Shared orchestration base for deterministic planners.

Concrete planners live in family subpackages — ``trajectory_optimization/``,
``planning/search/`` (RRT family), and ``policy_synthesis/`` (dynamic
programming). Traj-family results use :class:`~minilink.planning.results.TrajectoryPlan`;
policy-family results use :class:`~minilink.planning.results.PolicyPlan`.

The :class:`~minilink.planning.problems.PlanningProblem` remains
declarative and does not solve itself.
"""

from abc import ABC

from minilink.core.costs import CostFunction
from minilink.core.sets import Set
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PolicyPlan, TrajectoryPlan


class Planner(ABC):
    """
    Base class for deterministic planners.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.

    Notes
    -----
    Offline entry is :meth:`solve`. Traj-family solvers store
    :attr:`last_trajectory_plan`; policy-family solvers store
    :attr:`last_policy_plan`.
    """

    def __init__(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.last_trajectory_plan: TrajectoryPlan | None = None
        self.last_policy_plan: PolicyPlan | None = None

    def solve(self, **kwargs):
        """
        Offline solve entry.

        Subclasses typically override this to call
        :meth:`solve_trajectory` or :meth:`solve_policy`.
        """
        raise NotImplementedError(f"{type(self).__name__}.solve is not implemented")

    def solve_trajectory(self, **kwargs) -> TrajectoryPlan:
        """Fixed (offline) traj-family solve."""
        raise NotImplementedError(f"{type(self).__name__} has no trajectory solve")

    def solve_trajectory_from(self, x0, **kwargs) -> TrajectoryPlan:
        """Online traj-family solve from measured ``x0``."""
        raise NotImplementedError(f"{type(self).__name__} has no solve_trajectory_from")

    def solve_policy(self, **kwargs) -> PolicyPlan:
        """Fixed (offline) policy-family solve."""
        raise NotImplementedError(f"{type(self).__name__} has no policy solve")

    def solve_policy_from(self, x0, **kwargs) -> PolicyPlan:
        """Online policy-family solve from measured ``x0``."""
        raise NotImplementedError(f"{type(self).__name__} has no solve_policy_from")

    def _store_trajectory_plan(self, plan: TrajectoryPlan) -> TrajectoryPlan:
        self.last_trajectory_plan = plan
        return plan

    def _store_policy_plan(self, plan: PolicyPlan) -> PolicyPlan:
        self.last_policy_plan = plan
        return plan

    def require_trajectory_plan(self) -> TrajectoryPlan:
        """Return the latest traj plan or raise a clear error."""
        if self.last_trajectory_plan is None:
            raise ValueError("No trajectory plan has been computed yet")
        return self.last_trajectory_plan

    def require_policy_plan(self) -> PolicyPlan:
        """Return the latest policy plan or raise a clear error."""
        if self.last_policy_plan is None:
            raise ValueError("No policy plan has been computed yet")
        return self.last_policy_plan

    def require_cost(self) -> CostFunction:
        """Return ``problem.cost`` or raise a solver-facing error."""
        return self.problem.require_cost()

    def require_goal(self) -> Set:
        """Return ``problem.Xf`` or raise a solver-facing error."""
        return self.problem.require_goal()

    def plot_solution(self, *, signals=("x", "u"), backend="matplotlib"):
        """
        Plot the latest traj-family result with the problem system.
        """
        return self.problem.sys.plot_trajectory(
            self.require_trajectory_plan().trajectory,
            signals=signals,
            backend=backend,
        )

    def animate_solution(self, **kwargs):
        """
        Animate the latest traj-family result with the problem system.
        """
        return self.problem.sys.animate(
            self.require_trajectory_plan().trajectory, **kwargs
        )
