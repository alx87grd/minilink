"""The planner base: a planner solves a declarative problem into a stored solution."""

import warnings
from abc import ABC

from minilink.blocks.sources import TrajectorySource
from minilink.core.costs import CostFunction
from minilink.core.roles import command_port
from minilink.core.sets import Set
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.evaluator import evaluate
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution

###############################################################################
# Public API
###############################################################################


class Planner(ABC):
    """
    Base class for planners.

    Concrete planners live in family subpackages: ``trajectory_optimization/``,
    ``search/`` (RRT family), ``dynamic_programming/`` (dynamic programming) and
    ``reinforcement_learning/``. Every ``solve`` returns a
    :class:`~minilink.planning.results.PlanningSolution` and stores it as
    :attr:`last_solution`; the :class:`~minilink.planning.problems.PlanningProblem`
    stays declarative and does not solve itself.

    Parameters
    ----------
    problem : PlanningProblem
        Declarative planning problem consumed by this planner.

    Notes
    -----
    Offline entry is :meth:`solve`, which returns and stores the
    :class:`~minilink.planning.results.PlanningSolution`; :meth:`get_controller`
    is its policy, :meth:`plot_solution` and :meth:`animate_solution` its
    trajectory, :meth:`plot_control_law` and :meth:`plot_cost_to_go` its law
    and cost-to-go.
    """

    #: Planners that read the uncertainty of a stochastic problem set this;
    #: the others read its start set ``X0`` and its ``x_start``, and warn.
    accepts_stochastic = False

    def __init__(self, problem: PlanningProblem) -> None:
        # The declarative problem
        self.problem = problem

        # Memory
        self.last_solution: PlanningSolution | None = None

        # A deterministic planner reads a stochastic problem's start set, not its draws
        if problem.is_stochastic and not self.accepts_stochastic:
            warnings.warn(
                f"{type(self).__name__} is a deterministic planner: it reads the "
                "start set X0 of this stochastic problem (the support of its start "
                "law) and its x_start, and ignores its laws and random sources. "
                "Pass problem.nominal() to plan from the mean start alone.",
                stacklevel=2,
            )

    # Solve entries: each family overrides its own

    def solve(self, **kwargs):
        """
        Offline solve entry: the planner's verb, each planner's own solve.

        A trajectory planner's ``solve`` runs :meth:`solve_trajectory`. A policy
        planner's ``solve`` is the solve itself, and :meth:`solve_policy`, the
        family name every policy planner answers, calls it.
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

    # The latest solution, and the parts of the problem a solver needs

    def require_solution(self) -> PlanningSolution:
        """Refuse a planner with no solution yet; return the latest one."""
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
        """Refuse a problem without a cost; return ``problem.cost``."""
        return self.problem.require_cost()

    def require_goal(self) -> Set:
        """Refuse a problem without a goal set; return ``problem.Xf``."""
        return self.problem.require_goal()

    # Views of the latest solution

    def plot_solution(self, *, signals=("x", "u"), backend="matplotlib"):
        """Plot the latest solution's trajectory with the problem system."""
        return self.problem.sys.plot_trajectory(
            self.solution_trajectory(), signals=signals, backend=backend
        )

    def animate_solution(self, **kwargs):
        """Animate the latest solution's trajectory with the problem system."""
        return self.problem.sys.animate(self.solution_trajectory(), **kwargs)

    def plot_control_law(self, **kwargs):
        """The latest solution's law over the problem's box (:meth:`PlanningSolution.plot_control_law`)."""
        return self.require_solution().plot_control_law(**kwargs)

    def plot_cost_to_go(self, **kwargs):
        """The latest solution's cost-to-go field (:meth:`PlanningSolution.plot_cost_to_go`)."""
        return self.require_solution().plot_cost_to_go(**kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def solution_trajectory(self) -> Trajectory:
        """The latest solution's trajectory, rolled out on demand for a feedback law."""
        solution = self.require_solution()

        # The planned trajectory, else the feedback law rolled out from the start
        if solution.trajectory is not None:
            return solution.trajectory

        return self.nominal_trajectory()

    # What a family's solve builds its solution from

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

        # The command port's values, when the plant has a u port or a single input
        if "u" in sys.inputs or len(sys.inputs) == 1:
            u = sys.get_port_values_from_u(u, command_port(sys))

        return TrajectorySource(trajectory.t, u, interpolation=interpolation)

    def evaluate(
        self, policy, *, dt, n_trials=50, tf=None, backend="numpy"
    ) -> Evaluation | None:
        """
        The :func:`~minilink.planning.evaluator.evaluate` of ``policy`` on this planner's problem.

        One run when nothing is random, ``n_trials`` draws from key 0 otherwise;
        ``None`` when the problem declares no cost (a search without one).
        """
        # A problem without a cost has nothing to score
        if self.problem.cost is None:
            return None

        # One run when nothing is random, n_trials draws otherwise
        trials = int(n_trials) if self.problem.is_stochastic else None

        return evaluate(
            self.problem,
            policy,
            dt=dt,
            n_trials=trials,
            key=0,
            backend=backend,
            episode_length=tf,
        )

    def store_solution(self, solution: PlanningSolution) -> PlanningSolution:
        """Keep ``solution`` as :attr:`last_solution`, and return it."""
        self.last_solution = solution
        return solution
