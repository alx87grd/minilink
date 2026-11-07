"""The planner base: a planner solves a declarative problem into a stored solution."""

import warnings
from abc import ABC
from copy import deepcopy

import numpy as np

from minilink.blocks.sources import TrajectorySource
from minilink.core.params import checked, merge_known
from minilink.core.roles import command_port
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.evaluator import controller_of, evaluate
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
    :class:`~minilink.planning.results.PlanningSolution`, and mirrors it:
    :attr:`controller` is its block, :meth:`policy` its law ``u = pi(x, t)``,
    :meth:`cost_to_go` its cost-to-go at a state, :meth:`plot_trajectory` and
    :meth:`animate` its trajectory, :meth:`plot_control_law` and
    :meth:`plot_cost_to_go` its law and cost-to-go.
    """

    #: Planners that read the uncertainty of a stochastic problem set this;
    #: the others read its start set ``X0`` and its ``x0``, and warn.
    accepts_stochastic = False

    #: Planners whose feedback law runs on the grid of its ``dt`` (value
    #: iteration, the learners) score it held over each period; a planner whose
    #: law is continuous (the LQR) clears this and scores the continuous loop.
    holds_policy = True

    def __init__(self, problem: PlanningProblem) -> None:
        # The declarative problem, and the seed of a planner that draws (None: none)
        self.problem = problem
        self.seed = None

        # Memory
        self.last_solution: PlanningSolution | None = None

        # A deterministic planner reads a stochastic problem's start set, not its draws
        if problem.is_stochastic and not self.accepts_stochastic:
            warnings.warn(
                f"{type(self).__name__} is a deterministic planner: it reads the "
                "start set X0 of this stochastic problem (the support of its start "
                "law) and its x0, and plans on its parameters' means with its "
                "random sources at their mean. Pass problem.nominal() to plan from "
                "the mean start alone.",
                stacklevel=2,
            )

    #: The constructor keywords that are the method's numbers, ``params["method"]``
    number_names: tuple = ()

    @property
    def params(self) -> dict:
        """
        The planner's tree: the problem's leaves by role, ``"method"`` its numbers, ``"seed"`` its seed.

        Its problem parts are the problem's own dicts (live); ``solve(params=...)``
        takes the complete tree, a copy edited (``copy.deepcopy(planner.params)``),
        and solves on the planner rebuilt there (:func:`planner_at`); the seed is
        ``None`` for a planner that draws nothing.
        """
        method = {name: getattr(self, name) for name in self.number_names}

        return {**self.problem.parts_view(), "method": method, "seed": self.seed}

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: its structure and the numbers it was given, never a display (the workflow is the verb's)."""
        raise NotImplementedError(f"{type(self).__name__} has no build_kwargs")

    # Solve entries: each family overrides its own

    def solve(self, **kwargs):
        """
        Offline solve entry: the planner's verb, each planner's own solve.

        Every planner answers ``solve``, the one offline entry of the family.
        """
        raise NotImplementedError(f"{type(self).__name__}.solve is not implemented")

    # The latest solution's law and cost-to-go

    @property
    def controller(self):
        """The latest solution's controller: a feedback block, or the open-loop source of a trajectory planner."""
        return require_solution(self).controller

    def policy(self, x=None, t=0.0):
        """The latest solution's command ``u = pi(x, t)`` (:meth:`PlanningSolution.policy`)."""
        return require_solution(self).policy(x, t)

    def cost_to_go(self, x, t=None) -> float:
        """The latest solution's cost-to-go at ``x`` (at ``t`` when given): its ``cost_to_go`` field, read."""
        J = require_solution(self).cost_to_go
        value = J(x) if t is None else J(x, t=t)

        return float(value)

    def nominal_trajectory(self, tf=None, *, solution=None) -> Trajectory:
        """``solution``'s controller from the problem's start on the planner's grid (feedback planners); the latest solution when none is given."""
        raise NotImplementedError(f"{type(self).__name__} has no nominal_trajectory")

    # Views of the latest solution

    def plot_trajectory(self, *, signals=("x", "u"), backend="matplotlib"):
        """Plot the latest solution's trajectory with the problem system."""
        return self.problem.sys.plot_trajectory(
            solution_trajectory_of(self), signals=signals, backend=backend
        )

    def animate(self, **kwargs):
        """Animate the latest solution's trajectory with the problem system."""
        return self.problem.sys.animate(solution_trajectory_of(self), **kwargs)

    def plot_control_law(self, **kwargs):
        """The latest solution's law over the problem's box (:meth:`PlanningSolution.plot_control_law`)."""
        return require_solution(self).plot_control_law(**kwargs)

    def plot_cost_to_go(self, **kwargs):
        """The latest solution's cost-to-go field (:meth:`PlanningSolution.plot_cost_to_go`)."""
        return require_solution(self).plot_cost_to_go(**kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def store_solution(self, solution: PlanningSolution) -> PlanningSolution:
        """Keep ``solution`` as :attr:`last_solution`, and return it."""
        self.last_solution = solution
        return solution


###############################################################################
# Internal machinery
###############################################################################


def require_solution(planner) -> PlanningSolution:
    """Refuse a planner with no solution yet; return the latest one."""
    if planner.last_solution is None:
        raise ValueError("No solution has been computed yet")

    return planner.last_solution


def solution_trajectory_of(planner) -> Trajectory:
    """The latest solution's trajectory, rolled out on demand for a feedback law."""
    solution = require_solution(planner)

    # The planned trajectory, else the feedback law rolled out from the start
    if solution.trajectory is not None:
        return solution.trajectory

    return planner.nominal_trajectory(solution=solution)


def open_loop_controller_of(
    trajectory, sys, *, interpolation="linear"
) -> TrajectorySource:
    """
    The trajectory's input as a source block ``u = pi(t)`` for the plant ``sys``: ``controller >> sys``.

    The source carries the plant's action port (``u``, or its single
    input) so that series wiring and the evaluator feed it as the plant
    expects; a plant with several named ports and no ``u`` gets the
    stacked input vector, to wire by hand.
    """
    u = trajectory.u

    # The command port's values, when the plant has a u port or a single input
    if "u" in sys.inputs or len(sys.inputs) == 1:
        u = sys.get_port_values_from_u(u, command_port(sys))

    return TrajectorySource(trajectory.t, u, interpolation=interpolation)


def planner_evidence(
    planner, controller, *, dt, n_trials=50, tf=None, backend="numpy", problem=None
) -> Evaluation | None:
    """
    The evidence a planner's ``solve(evaluate=True)`` records: its law scored on the real plant.

    The :func:`~minilink.planning.evaluator.evaluate` of ``controller`` on the
    planner's problem (or ``problem``): one run when nothing is random,
    ``n_trials`` draws from key 0 otherwise; ``None`` when the problem declares
    no cost (a search without one). A feedback law with no clock of its own
    is held over each period of ``dt`` (``controller % dt``), the loop its planner
    designs for, and the plant is integrated at the automatic step; an
    open-loop source keeps the interpolation its planner states. The public
    way to score a law is the problem's evaluator,
    ``problem.compile(ctl % dt).evaluate()``, with ``substeps=`` for the
    planner's own physics.
    """
    problem = planner.problem if problem is None else problem

    # A problem without a cost has nothing to score
    if problem.cost is None:
        return None

    # One run when nothing is random, n_trials draws otherwise
    trials = int(n_trials) if problem.is_stochastic else None

    # The law the run holds: a feedback law on the clock of dt, when the
    # planner designs it on that grid
    law = held_feedback(controller, dt) if planner.holds_policy else controller

    return evaluate(
        problem,
        law,
        dt=dt,
        n_trials=trials,
        key=0,
        backend=backend,
        episode_length=tf,
    )


def held_feedback(controller, dt):
    """A feedback controller with no clock of its own on the clock of ``dt`` (``controller % dt``); a source, a tick law or a clocked law as given."""
    # Lazy import: the step system loads with the compile chain
    from minilink.core.discrete.step_system import StepSystem

    controller = controller_of(controller)
    reads_input = int(controller.m) > 0
    tick_or_clocked = isinstance(controller, StepSystem)
    if reads_input and not tick_or_clocked:
        return controller % float(dt)

    return controller


def planner_at(planner, params) -> "Planner":
    """
    A planner of the same kind at the complete tree ``params`` of ``planner.params``: what ``solve(params=)`` solves with.

    A new planner, built from the planner's keywords (the numbers it was
    given, never the defaults it derived, so a changed problem derives its
    own) on a copy of its problem holding the tree; the planner is unchanged.
    It inherits no display: the caller's ``solve`` hands its own workflow
    keywords (``verbose``, ``callback``, ...) to the rebuilt planner's.
    A method number the tree leaves at the planner's value is not given: it
    is derived again on the new problem (the discount factor follows a new
    rate); a number the tree changes is given. The host planners solve
    through it (value iteration, tabular learning, LQR, RRT, RRT*); a planner
    with a compiled face (trajectory optimization, MPPI) solves at a tree
    through the face.
    """
    require_known_numbers(planner, (params or {}).get("method"))
    params = checked(params, planner.params)
    problem_leaves, method, seed = planner_tree_parts(params)
    problem = problem_with(planner.problem, host_tree(problem_leaves))

    # The numbers the tree changes: the others stay derived (or given) as they were
    own = planner.params["method"]
    changed = {
        name: value
        for name, value in (method or {}).items()
        if not np.array_equal(np.asarray(value), np.asarray(own[name]))
    }
    kwargs = {**planner.build_kwargs(), **changed}
    if seed is not None:
        kwargs["seed"] = seed

    return type(planner)(problem, **kwargs)


def given_numbers(**numbers) -> dict:
    """The numbers the caller gave (``None``: derived from the problem, left out)."""
    return {name: value for name, value in numbers.items() if value is not None}


def planner_tree_parts(params):
    """``(problem, method, seed)``: a planner's tree split into the problem's leaves, the method's numbers and the seed."""
    if not params:
        return None, None, None
    problem = {
        key: value for key, value in params.items() if key not in ("method", "seed")
    }

    return problem, params.get("method"), params.get("seed")


def require_known_numbers(planner, method):
    """Refuse a number the planner's method does not have."""
    unknown = sorted(set(method or {}) - set(planner.number_names))
    if unknown:
        raise ValueError(
            f"{type(planner).__name__} has no number(s) {unknown} under "
            f'params["method"]; its numbers are {list(planner.number_names)}'
        )


def host_tree(tree):
    """A tree with every leaf a NumPy value (``None`` kept, an empty subtree kept empty), or ``None`` for none."""
    if not tree:
        return None
    if not isinstance(tree, dict):
        return tree

    return host_subtree(tree)


def host_subtree(tree):
    """Every leaf of ``tree`` a NumPy value, its subtrees kept, the empty ones too."""
    return {
        key: host_subtree(value) if isinstance(value, dict) else host_leaf(value)
        for key, value in tree.items()
    }


def host_leaf(value):
    """One leaf on the host: a NumPy value, ``None`` kept."""
    return None if value is None else np.asarray(value)


def problem_with(problem, params):
    """The problem itself for no values, else a copy holding them: its parts copied, so the caller's stay as they are."""
    if not params:
        return problem

    merged = merge_known(problem.parts_view(), params)
    copy = deepcopy(problem)
    copy.params = merged

    return copy
