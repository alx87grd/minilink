"""
Shared orchestration base for planners.

Concrete planners live in family subpackages — ``trajectory_optimization/``,
``planning/search/`` (RRT family), ``policy_synthesis/`` (dynamic
programming) and ``reinforcement_learning/``. Every ``solve`` returns a
:class:`~minilink.planning.results.PlanningSolution` and stores it as
:attr:`Planner.last_solution`.

The :class:`~minilink.planning.problems.PlanningProblem` remains
declarative and does not solve itself.
"""

import warnings
from abc import ABC

from minilink.blocks.sources import TrajectorySource
from minilink.control.neural import action_port_of
from minilink.core.costs import CostFunction
from minilink.core.sets import Set
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation, MonteCarloEvaluator
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution


class Planner(ABC):
    """
    Base class for planners.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.

    Notes
    -----
    Offline entry is :meth:`solve`, which returns and stores the
    :class:`~minilink.planning.results.PlanningSolution`; :meth:`get_controller`
    is its policy, :meth:`plot_solution` and :meth:`animate_solution` its
    trajectory.
    """

    #: Planners that read the uncertainty of a stochastic problem set this;
    #: the others plan on the nominal start and warn.
    accepts_stochastic = False

    def __init__(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.last_solution: PlanningSolution | None = None
        if problem.is_stochastic and not self.accepts_stochastic:
            warnings.warn(
                f"{type(self).__name__} is a deterministic planner: it plans from the "
                "mean start of this StochasticPlanningProblem and ignores its "
                "distributions. Pass problem.nominal() to say so explicitly.",
                stacklevel=2,
            )

    def solve(self, **kwargs):
        """
        Offline solve entry.

        Subclasses typically override this to call
        :meth:`solve_trajectory` or :meth:`solve_policy`.
        """
        raise NotImplementedError(f"{type(self).__name__}.solve is not implemented")

    def solve_trajectory(self, **kwargs) -> PlanningSolution:
        """Fixed (offline) traj-family solve."""
        raise NotImplementedError(f"{type(self).__name__} has no trajectory solve")

    def solve_trajectory_from(self, x0, **kwargs) -> PlanningSolution:
        """Online traj-family solve from measured ``x0``."""
        raise NotImplementedError(f"{type(self).__name__} has no solve_trajectory_from")

    def solve_policy(self, **kwargs) -> PlanningSolution:
        """Fixed (offline) policy-family solve."""
        raise NotImplementedError(f"{type(self).__name__} has no policy solve")

    def require_solution(self) -> PlanningSolution:
        """Return the latest solution or raise a clear error."""
        if self.last_solution is None:
            raise ValueError("No solution has been computed yet")
        return self.last_solution

    def get_controller(self):
        """The solution's policy: a feedback block, or the open-loop source of a trajectory planner."""
        return self.require_solution().policy

    def nominal_trajectory(self, tf=None) -> Trajectory:
        """The solution's policy from the problem's start on the planner's grid (feedback planners)."""
        raise NotImplementedError(f"{type(self).__name__} has no nominal_trajectory")

    def require_cost(self) -> CostFunction:
        """Return ``problem.cost`` or raise a solver-facing error."""
        return self.problem.require_cost()

    def require_goal(self) -> Set:
        """Return ``problem.Xf`` or raise a solver-facing error."""
        return self.problem.require_goal()

    def plot_solution(self, *, signals=("x", "u"), backend="matplotlib"):
        """Plot the latest solution's trajectory with the problem system."""
        return self.problem.sys.plot_trajectory(
            self.solution_trajectory(), signals=signals, backend=backend
        )

    def animate_solution(self, **kwargs):
        """Animate the latest solution's trajectory with the problem system."""
        return self.problem.sys.animate(self.solution_trajectory(), **kwargs)

    # Internal machinery

    def store_solution(self, solution: PlanningSolution) -> PlanningSolution:
        self.last_solution = solution
        return solution

    def open_loop_policy(
        self, trajectory, *, interpolation="linear"
    ) -> TrajectorySource:
        """
        The trajectory's input as a source block ``u = pi(t)``: ``policy >> plant``.

        The source carries the plant's action port (``u``, or its single
        input) so that series wiring and the evaluator feed it as the plant
        expects; a plant with several named ports and no ``u`` gets the
        stacked input vector, to wire by hand.
        """
        sys = self.problem.sys
        u = trajectory.u
        if "u" in sys.inputs or len(sys.inputs) == 1:
            u = sys.get_port_values_from_u(u, action_port_of(sys))
        return TrajectorySource(trajectory.t, u, interpolation=interpolation)

    def solution_trajectory(self) -> Trajectory:
        """The latest solution's trajectory, rolled out on demand for a feedback law."""
        solution = self.require_solution()
        if solution.trajectory is not None:
            return solution.trajectory
        return self.nominal_trajectory()

    def evaluate(
        self, policy, *, dt, n_trials=50, tf=None, backend="numpy"
    ) -> Evaluation | None:
        """
        The standalone evaluator on this planner's problem.

        One trial when nothing is random; ``None`` when the problem declares no
        cost (a search without one).
        """
        if self.problem.cost is None:
            return None
        trials = int(n_trials) if self.problem.is_stochastic else 1
        evaluator = MonteCarloEvaluator(
            self.problem,
            dt=dt,
            n_trials=trials,
            episode_length=tf,
            backend=backend,
            seed=0,
        )
        return evaluator.evaluate(policy)
