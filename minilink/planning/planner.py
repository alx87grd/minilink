"""The planner base: a planner solves a declarative problem into a stored solution."""

import warnings
from abc import ABC
from copy import deepcopy

import numpy as np

from minilink.blocks.sources import TrajectorySource
from minilink.core.params import checked, flatten, lookup, merge_known, with_leaf
from minilink.core.roles import command_port
from minilink.planning.evaluation import Evaluation
from minilink.planning.evaluator import evaluate
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution, episode_ticks_of
from minilink.planning.step_model import is_continuous, step_model_of

###############################################################################
# Public API
###############################################################################


class Planner(ABC):
    """
    Base class for planners.

    Concrete planners live in family subpackages: ``trajectory_optimization/``,
    ``search/`` (RRT family) and ``dynamic_programming/`` (dynamic programming)
    here, and ``minilink.learning``'s ``reinforcement_learning/`` (the neural
    and the tabular learners). Every ``solve`` returns a
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
    :attr:`controller` is its block, :meth:`cost_to_go` its cost-to-go at a
    state, :meth:`plot_trajectory`, :meth:`animate` and :meth:`plot_cost` its
    plan or its controller's run (integrated on the planner's :attr:`dt`, the
    step of its evidence), :meth:`plot_control_law` and
    :meth:`plot_cost_to_go` its law and cost-to-go.
    """

    #: Planners that read the uncertainty of a stochastic problem set this;
    #: the others read its start set ``X0`` and its ``x0``, and warn.
    accepts_stochastic = False

    #: The constructor keywords that are the method's numbers, ``params["method"]``
    number_names: tuple = ()

    #: The planner's own step: the integration step of its evidence and of its
    #: views' run of a feedback law (``None``: none, the evaluator's automatic step)
    dt = None

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

    def rebuilt_on(self, problem, kwargs) -> "Planner":
        """A planner of this kind on ``problem`` from ``kwargs`` (:meth:`build_kwargs`, the changed numbers in): what :func:`planner_at` solves with."""
        return type(self)(problem, **kwargs)

    # Solve entries: each family overrides its own

    def solve(self, **kwargs):
        """
        Offline solve entry: the planner's verb, each planner's own solve.

        Every planner answers ``solve``, the one offline entry of the family.
        """
        raise NotImplementedError(f"{type(self).__name__}.solve is not implemented")

    # The latest solution's controller and cost-to-go

    @property
    def controller(self):
        """The latest solution's controller: a feedback block, or the open-loop source of a trajectory planner."""
        return require_solution(self).controller

    def cost_to_go(self, x, t=None) -> float:
        """The latest solution's cost-to-go field read at ``x`` (at ``t`` when given), a ``float``."""
        solution = require_solution(self)
        J = require_cost_to_go(solution, type(self).__name__)
        x = np.asarray(x, dtype=float)

        # J(x), or J(x, t) on a field that reads the time
        if t is None:
            value = J(x)
        else:
            value = J(x, t=t)

        return float(value)

    # Views of the latest solution

    def plot_trajectory(self, **kwargs):
        """The latest solution's plan, else its controller's run on the planner's ``dt`` (:meth:`PlanningSolution.plot_trajectory`)."""
        return require_solution(self).plot_trajectory(**{"dt": self.dt, **kwargs})

    def animate(self, **kwargs):
        """The latest solution's plan, else its controller's run on the planner's ``dt``, animated (:meth:`PlanningSolution.animate`)."""
        return require_solution(self).animate(**{"dt": self.dt, **kwargs})

    def plot_cost(self, **kwargs):
        """The running cost along the latest solution's plan, else its controller's run on the planner's ``dt`` (:meth:`PlanningSolution.plot_cost`)."""
        return require_solution(self).plot_cost(**{"dt": self.dt, **kwargs})

    def plot_control_law(
        self,
        *,
        x_axis=None,
        y_axis=None,
        u_axis=0,
        anchor=None,
        bounds=None,
        grid_shape=(101, 101),
        t=0.0,
        is_3d=False,
        trajectory=None,
        title=None,
        ax=None,
        renderer="matplotlib",
        show=True,
    ):
        """The latest solution's law (:meth:`PlanningSolution.plot_control_law`, the same keywords): a ``PlotResult``."""
        return require_solution(self).plot_control_law(
            x_axis=x_axis,
            y_axis=y_axis,
            u_axis=u_axis,
            anchor=anchor,
            bounds=bounds,
            grid_shape=grid_shape,
            t=t,
            is_3d=is_3d,
            trajectory=trajectory,
            title=title,
            ax=ax,
            renderer=renderer,
            show=show,
        )

    def plot_cost_to_go(
        self,
        *,
        x_axis=0,
        y_axis=1,
        anchor=None,
        bounds=None,
        grid_shape=(51, 51),
        t=0.0,
        jmax=None,
        is_3d=False,
        trajectory=None,
        title=None,
        ax=None,
        renderer="matplotlib",
        show=True,
    ):
        """The latest solution's cost-to-go field (:meth:`PlanningSolution.plot_cost_to_go`, the same keywords): a ``PlotResult``."""
        return require_solution(self).plot_cost_to_go(
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            bounds=bounds,
            grid_shape=grid_shape,
            t=t,
            jmax=jmax,
            is_3d=is_3d,
            trajectory=trajectory,
            title=title,
            ax=ax,
            renderer=renderer,
            show=show,
        )

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


def require_cost_to_go(solution, owner=None):
    """Refuse a solution whose method gives no cost-to-go field; return its ``cost_to_go``. ``owner`` names the solution's planner in the refusal (its family when ``None``)."""
    if solution.cost_to_go is None:
        owner = f"this {solution.family}" if owner is None else f"{owner}'s"
        raise ValueError(
            f"{owner} solution carries no cost-to-go field (its cost_to_go is None)"
        )

    return solution.cost_to_go


def open_loop_controller_of(trajectory, sys, *, hold="linear") -> TrajectorySource:
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
        u = sys.ports(u, command_port(sys))

    return TrajectorySource(trajectory.t, u, hold=hold)


def planner_evidence(
    planner,
    controller,
    *,
    dt=None,
    n_trials=50,
    episode_length=None,
    backend="numpy",
    problem=None,
) -> Evaluation | None:
    """
    The evidence a planner's ``solve(evaluate=True)`` records: its controller, as it is, scored on the real plant.

    The :func:`~minilink.planning.evaluator.evaluate` of ``controller`` on the
    planner's problem (or ``problem``): one run when nothing is random,
    ``n_trials`` draws from key 0 otherwise; ``None`` when the problem declares
    no cost (a search without one). A continuous controller runs on ``dt``, the
    planner's own step (ruled 2026-11-08: the grid's for value iteration and
    tabular learning, the environment's for deep learning, the LQR's, an
    open-loop plan's own grid), the law continuous, ``dt`` only how finely its
    loop is integrated; a clocked one on its own clock; ``episode_length`` is the
    run of an infinite horizon. On a step problem, the model's rollouts
    (:func:`step_evaluation`).
    """
    problem = planner.problem if problem is None else problem

    # A problem without a cost has nothing to score
    if problem.cost is None:
        return None

    # One run when nothing is random, n_trials draws otherwise
    trials = int(n_trials) if problem.is_stochastic else None

    # A step problem is scored on its model
    if not is_continuous(problem):
        return step_evaluation(
            problem, controller, trials, episode_length=episode_length
        )

    return evaluate(
        problem,
        controller,
        dt=dt,
        n_trials=trials,
        key=0,
        backend=backend,
        episode_length=episode_length,
    )


def step_evaluation(
    problem, controller, n_trials=None, *, key=0, episode_length=None
) -> Evaluation:
    """
    The :class:`~minilink.planning.evaluation.Evaluation` of a controller on a step problem: its model's rollouts, scored.

    One run on a deterministic problem, ``n_trials`` draws of
    ``problem.realize(key, n_trials)`` otherwise, each the model's
    ``J = Σ γᵏ c_k + γᴺ h`` over its horizon (an infinite one over an episode
    of ``episode_length`` ticks, 10 by default), with the judge's facts: the
    least margin of ``x_k`` to ``X`` and of ``u_k`` to ``U``, the margin of
    ``x_N`` to ``Xf``, and with ``penalize_failure`` the model's failure cost
    (a run not finite: the model's cost bound and it).
    """
    model = step_model_of(problem)
    N = episode_ticks_of(model, episode_length)
    n = 1 if n_trials is None else int(n_trials)
    draws = problem.realize(key if problem.is_stochastic else None, n)
    drawn = [("x0",), *(path for path, _ in flatten(problem.distributions))]
    failure_cost = model.failure_cost
    bound = model.cost_bound

    rows = []
    for i in range(n):
        params = trial_of(draws, drawn, i)

        # The model's run: x_0 … x_N, u_0 … u_{N−1}, cost = Σ γᵏ c_k + γᴺ h, the
        # running cost's last value
        traj = model.rollout(controller, params=params, n_steps=N)
        xs = traj.x.T
        us = traj.u[:, :N].T
        cost = traj.signals["cost"][0, -1]
        t = [model.time_of(k, params) for k in range(N + 1)]

        # The judge's facts: finite, the least margin to X and U, the margin to Xf
        finite = bool(np.all(np.isfinite(xs)) and np.all(np.isfinite(us)))
        margin_X = min(float(np.min(problem.X.margin(x, t_k))) for x, t_k in zip(xs, t))
        margin_U = min(
            float(np.min(problem.U.margin(u, x, t_k))) for u, x, t_k in zip(us, xs, t)
        )
        feasibility_margin = min(margin_X, margin_U)
        failed = not finite or feasibility_margin < 0.0
        reachability_margin = np.inf
        if problem.Xf is not None:
            reachability_margin = float(np.min(problem.Xf.margin(xs[-1], t[-1])))
        reached = reachability_margin >= 0.0

        # J = cost, or with the failure charged cost + failure_cost · (failed or not
        # reached), and the cost bound + failure_cost for a run not finite
        cost = float(cost) if finite else np.inf
        J = cost
        if problem.penalize_failure:
            charged = failure_cost if failed or not reached else 0.0
            J = (cost if finite else bound) + charged

        rows.append(
            (cost, J, feasibility_margin, failed, reachability_margin, reached, xs[0])
        )

    return Evaluation(
        cost=np.array([row[0] for row in rows]),
        J=np.array([row[1] for row in rows]),
        feasibility_margin=np.array([row[2] for row in rows]),
        failed=np.array([row[3] for row in rows]),
        reachability_margin=np.array([row[4] for row in rows]),
        reached=np.array([row[5] for row in rows]),
        x0=np.stack([row[6] for row in rows]),
        criterion=problem.criterion,
    )


def trial_of(draws, drawn, i):
    """Trial ``i`` of a step problem's realization: each drawn leaf (``drawn``, its paths) indexed, the shared ones as they are."""
    params = draws
    for path in drawn:
        params = with_leaf(params, path, lookup(draws, path)[i])

    return params


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

    return planner.rebuilt_on(problem, kwargs)


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
