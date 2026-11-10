"""The planning solution: the controller, the plan it predicts, and the evidence for them."""

from __future__ import annotations

import copy
import re
from dataclasses import dataclass

import numpy as np

from minilink.core.fields import Field
from minilink.core.inspect import body_line, format_inspect, repr_pretty
from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.evaluator import EPISODE_LENGTH
from minilink.planning.problems import PlanningProblem, require_cost
from minilink.planning.step_model import is_continuous, step_model_of

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True, kw_only=True)
class PlanningSolution:
    """
    What every planner returns: the controller, the plan it predicts, and the evidence for them.

    One rule: the controller handed out is the block scored in ``evaluation``
    and drawn by the views, as it is. Everything else here is the solver's
    own claim under its own model — the plan, the table on its grid, the
    Riccati form on the linear model, the critic's fit. A measurement of the
    controller comes from the evaluators
    (:func:`~minilink.planning.evaluator.evaluate`, value iteration's
    ``evaluate_policy``) as its
    own object.

    Parameters
    ----------
    problem : PlanningProblem
        What was solved: the system, its sets, the cost and the horizon. A
        frozen record; its ``sys`` is a live reference, not a snapshot.
    controller : System
        The block you run, as it is. A feedback law (LQR, value iteration,
        tabular and deep learning) reads the state: ``controller @ plant``
        closes the loop, continuous. An open-loop plan (trajectory
        optimization, MPPI, RRT) is a source such as a
        :class:`~minilink.blocks.sources.TrajectorySource`: ``controller >>
        plant`` drives the plant. On a step problem, the law ticks with the plant.
    record : record
        The planner's account of the solve, one dataclass per planner
        (``TrajectoryOptimizationRecord``, ``TreeSearchRecord``,
        ``ValueIterationRecord``, ``TabularLearningRecord``,
        ``ReinforcementLearningRecord``, ``RiccatiRecord``, ``MPPIRecord``),
        each with a ``success`` flag and a one-line ``str``.
    trajectory : Trajectory, optional
        The plan an open-loop planner computed, on its own grid and model;
        ``None`` for a feedback law (its views run the controller).
    cost_to_go : Field, optional
        ``J(x)``, the method's own estimate under its own model, where the
        method produces one: a :class:`~minilink.core.fields.GridField` for
        dynamic programming and tabular learning (the failure cost past its
        grid), a :class:`~minilink.core.fields.QuadraticField` for a
        linear-quadratic design (``J(x, t=t)`` on a finite horizon), the
        critic's field for a learned law (when it estimates the problem's own
        discount). On a stochastic problem it is the expected cost-to-go when
        the method takes the distributions.
    evaluation : Evaluation, optional
        The score of ``controller``, as it is, over the problem's draws under
        the one scoring contract (one trial on a deterministic problem), when
        the solve was asked to evaluate (``solve(evaluate=True)``); ``None``
        otherwise, or when the problem declares no cost.
    params : dict, optional
        The numbers the solve ran on: the problem's tree as it was when the
        solution was made, a copy. ``None`` (the default) takes it from
        ``problem`` at construction; the live problem may move afterwards, the
        evidence keeps its numbers.
    """

    problem: PlanningProblem
    controller: System
    record: object
    trajectory: Trajectory | None = None
    cost_to_go: Field | None = None
    evaluation: Evaluation | None = None
    params: dict | None = None

    def __post_init__(self) -> None:
        # The numbers solved on, a copy: the evidence keeps them when the problem moves
        if self.params is None:
            object.__setattr__(self, "params", copy.deepcopy(self.problem.params))

    @property
    def success(self) -> bool:
        """The record's own success: feasible, converged, goal reached, finite weights, stable."""
        return bool(self.record.success)

    @property
    def open_loop(self) -> bool:
        """``True`` for an open-loop controller, a source with no input port: ``u = pi(t)``."""
        return int(self.controller.m) == 0

    @property
    def family(self) -> str:
        """The planning family, read from the record (``"value iteration"``, ``"riccati"``, ``"mppi"``, ...)."""
        # The record's class name without its suffix, one lowercase word per capitalized
        # word (an acronym stays one word)
        name = type(self.record).__name__.removesuffix("Record")

        return re.sub(r"(?<=[a-z])(?=[A-Z])", " ", name).lower()

    # Views, run on demand: the plan, else the controller run from the problem's start

    def plot_trajectory(
        self, *, dt=None, signals=("x", "u"), renderer="matplotlib", show=True
    ):
        """
        Draw the plan, else the controller's run from the problem's start, on the problem's system.

        The run is the loop the score runs, ``problem.compile(controller, dt)``
        (a continuous law integrated on ``dt``, ``None`` the evaluator's
        automatic step; a clocked one on its own clock; a step problem's
        model), once from the start with nominal parameters. The planner's
        mirror passes its own ``dt``, the step of its evidence. The figure is
        titled plan or run. ``renderer`` is the drawing library,
        ``"matplotlib"`` or ``"plotly"``.
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import plot_solution_trajectory

        return plot_solution_trajectory(
            self,
            drawn_trajectory_of(self, dt),
            signals=signals,
            renderer=renderer,
            show=show,
        )

    def animate(self, *, dt=None, **kwargs):
        """Animate the plan, else the controller's run from the problem's start on ``dt`` (:meth:`plot_trajectory`), on the problem's system."""
        return self.problem.sys.animate(drawn_trajectory_of(self, dt), **kwargs)

    def plot_cost(self, *, dt=None, renderer="matplotlib", show=True):
        """Draw the running cost and ``J(t)`` along the plan, else the controller's run on ``dt`` (:meth:`plot_trajectory`), with the drawing library ``renderer``."""
        return self.problem.sys.plot_cost(
            require_cost(self.problem),
            traj=drawn_trajectory_of(self, dt),
            renderer=renderer,
            show=show,
        )

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
        """
        Draw the feedback law, input ``u_axis`` over the state axes ``x_axis`` and ``y_axis``: a :class:`~minilink.graphical.common.plot_result.PlotResult`.

        A grid planner's law (value iteration, tabular learning) is its table
        on the grid's own nodes; any other law is the controller's own
        :meth:`~minilink.core.facades.plotting.SystemPlotting.plot_control_law`
        swept over ``bounds``, the problem's constraint set (or the plant's
        state box) by default, on a ``grid_shape`` mesh at the time ``t``,
        its colour limits from the input set, so laws of one problem share one
        scale (``bounds``, ``grid_shape``, ``t`` or ``is_3d`` given, a grid
        law is swept the same way); ``anchor`` pins a grid law's other axes
        (a controller's are at their nominal values). ``trajectory`` overlays a run, ``title``
        replaces the title, ``ax`` draws into existing axes, ``show=False``
        builds only; ``renderer`` is ``"matplotlib"``.
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import plot_control_law

        return plot_control_law(
            self,
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
        """
        Draw the method's cost-to-go over the state axes ``x_axis`` and ``y_axis``: a :class:`~minilink.graphical.common.plot_result.PlotResult`.

        A grid planner's table is drawn on the grid's own nodes; any other
        field is sampled on a ``grid_shape`` mesh over ``bounds`` (the
        problem's state box) at the time ``t``, the other axes pinned at
        ``anchor`` (the goal, else the start). ``jmax`` tops the colour scale,
        by default the field's value past its grid (the failure cost) when it
        has one, ``np.inf`` for Matplotlib's own scale; ``is_3d`` adds the
        surface, ``trajectory`` overlays a run, ``title`` replaces the title,
        ``ax`` draws into existing axes, ``show=False`` builds only;
        ``renderer`` is ``"matplotlib"``.
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import plot_cost_to_go

        return plot_cost_to_go(
            self,
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

    # Display

    def __str__(self) -> str:
        return solution_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_feedback_law(self) -> None:
        """Refuse to draw the control law of an open-loop controller ``u = pi(t)``."""
        if self.open_loop:
            raise ValueError(
                "an open-loop controller u = pi(t) has no control law to draw; "
                "plot_trajectory draws the plan"
            )


###############################################################################
# Internal machinery
###############################################################################


def drawn_trajectory_of(solution, dt=None) -> Trajectory:
    """
    What a solution's views draw: its plan, else its controller run once from the problem's start.

    The run is the loop the score runs, ``problem.compile(controller, dt)`` (a
    continuous law integrated on ``dt``, ``None`` the automatic step), with
    nominal parameters (a stochastic problem at its mean start), the plant's
    own run read back (``traj.of("plant")``); on a step problem, its model's
    rollout (:func:`step_run_of`).
    """
    if solution.trajectory is not None:
        return solution.trajectory

    problem = solution.problem
    if not is_continuous(problem):
        return step_run_of(problem, solution.controller)

    run = problem.compile(solution.controller, dt).trajectory().of("plant")
    plant_run = Trajectory(
        t=np.asarray(run.t), x=np.asarray(run.x), u=np.asarray(run.u)
    )

    return plant_run


def step_run_of(problem, controller, params=None, *, episode_length=None) -> Trajectory:
    """
    A step problem's rollout under ``controller`` from its start: the model's :meth:`~minilink.planning.step_model.StepModel.rollout`.

    ``N`` ticks: its own horizon, else an episode of ``episode_length`` (10
    by default) in its time. ``params`` is the model's tree (``None``: its
    own). The last input is held at the final state, so ``x`` and ``u`` share
    the ``N + 1`` times.
    """
    model = step_model_of(problem)
    params = model.params if params is None else params
    N = episode_ticks_of(model, episode_length)

    # x_0 … x_N and u_0 … u_N on the ticks t0 + k, the last input held
    traj = model.rollout(controller, params=params, n_steps=N)

    return traj


def episode_ticks_of(model, episode_length=None) -> int:
    """The ticks of a model's rollout: its horizon ``N``, else an episode of ``episode_length`` (10 by default) in its time."""
    if model.n_steps is not None:
        return int(model.n_steps)

    length = EPISODE_LENGTH if episode_length is None else float(episode_length)

    return max(1, int(round(length / model.period)))


def solution_text(solution):
    """The print text of a solution: its method, law, plan, evaluation and record."""
    law = "u = pi(t), open loop" if solution.open_loop else "u = pi(x), feedback"
    if solution.trajectory is None:
        trajectory = "none (a feedback law: the views run the controller)"
    else:
        trajectory = (
            f"plan of {solution.trajectory.n_samples} samples over "
            f"{solution.trajectory.tf - solution.trajectory.t0:.3g} s"
        )
    evaluation = "none" if solution.evaluation is None else str(solution.evaluation)
    success = "yes" if solution.success else "no"
    headline = f"PlanningSolution, family={solution.family}, success={success}"
    body = [
        body_line("controller", f"{solution.controller.name} ({law})"),
        body_line("trajectory", trajectory),
        body_line("evaluation", evaluation),
        body_line(
            "cost_to_go",
            "yes" if solution.cost_to_go is not None else "no",
        ),
        body_line("record", str(solution.record)),
    ]

    return format_inspect(headline, body)


def solve_time_text(solve_time_s) -> str:
    """A record's printed solve time, ``"; solved in 0.123 s"``, or nothing when the solve has no wall clock (``None``)."""
    if solve_time_s is None:
        return ""

    return f"; solved in {solve_time_s:.3g} s"
