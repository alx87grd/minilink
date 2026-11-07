"""The planning solution: the controller, its policy, its cost-to-go, and the evidence for them."""

from __future__ import annotations

import copy
import re
from collections.abc import Callable
from dataclasses import dataclass

from minilink.core.inspect import body_line, format_inspect, repr_pretty
from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.problems import PlanningProblem, require_cost

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class PlanningSolution:
    """
    What every planner returns: the controller, its cost-to-go and the evidence for them.

    Everything here is the solver's own claim under its own model — the table
    on its grid, the Riccati form on the linear model, the critic's fit. A
    measurement of the law comes from the evaluators (:func:`~minilink.planning.evaluator.evaluate`,
    ``PolicyEvaluator``) as its own object and is never written back here.

    Parameters
    ----------
    problem : PlanningProblem
        What was solved: the system, its sets, the cost and the horizon. A
        frozen record; its ``sys`` is a live reference, not a snapshot.
    controller : System
        The block a loop closes on. A feedback controller reads the state,
        ``controller @ plant`` closes the loop; an open-loop controller is a
        source such as a :class:`~minilink.blocks.sources.TrajectorySource`,
        ``controller >> plant`` drives the plant. Its policy, the law
        ``u = pi(x, t)`` as a function, is :meth:`policy`; a finite-horizon
        design is feedback in time.
    record : record
        The planner's account of the solve, one dataclass per planner
        (``TrajectoryOptimizationRecord``, ``TreeSearchRecord``,
        ``ValueIterationRecord``, ``TabularLearningRecord``,
        ``ReinforcementLearningRecord``, ``RiccatiRecord``), each with a
        ``success`` flag and a one-line ``str``.
    trajectory : Trajectory or None
        What the controller produces from the problem's start on the planner's
        grid with nominal parameters: the schedule of an open-loop planner,
        the rollout of a feedback law when the solve was asked to evaluate.
        It is the planner's evidence, not a closed-loop simulation: close the
        loop in the script, ``controller @ plant``, and simulate that.
    evaluation : Evaluation or None
        The cost over the problem's draws under the one scoring contract
        (one trial on a deterministic problem), when the solve was asked to
        evaluate; ``None`` otherwise, or when the problem declares no cost.
    cost_to_go : Field, callable or None
        ``J(x)``, the method's own estimate under its own model, where the
        method produces one: a :class:`~minilink.core.fields.GridField` for
        dynamic programming (the failure cost past its grid), a
        :class:`~minilink.core.fields.QuadraticField` for a linear-quadratic
        design (``J(x, t=t)`` on a finite horizon); the interpolated table of
        tabular learning and the critic of a learned law (when it estimates
        the problem's own discount) are plain callables of ``x``. On a
        stochastic problem it is the expected cost-to-go when the method
        takes the distributions.
    params : dict, optional
        The numbers the solve ran on: the problem's tree as it was when the
        solution was made, a copy. ``None`` (the default) takes it from
        ``problem`` at construction; the live problem may move afterwards, the
        evidence keeps its numbers.
    """

    problem: PlanningProblem
    controller: System
    record: object
    trajectory: Trajectory | None
    evaluation: Evaluation | None
    cost_to_go: Callable | None = None
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
    def method(self) -> str:
        """The planning method, read from the record (``"value iteration"``, ``"riccati"``, ...)."""
        # The record's class name without its suffix, one lowercase word per capital
        name = type(self.record).__name__.removesuffix("Record")

        return re.sub(r"(?<!^)(?=[A-Z])", " ", name).lower()

    def policy(self, x=None, t=0.0):
        """
        The command ``u = pi(x, t)`` of the controller, a 1-D array: its policy as a function.

        A feedback controller reads ``x`` on its measurement port, its other
        inputs (a reference) at their nominal values and its own state at its
        ``x0``; an open-loop controller reads the time ``t`` alone and
        ignores ``x``.
        """
        # A real cycle: the policy adapters import the planners, which import this module
        from minilink.planning.dynamic_programming.policy_eval import (
            policy_from_block,
            policy_from_source,
        )

        # The law of the block: the time alone for a source, the state and the time
        # for a feedback controller
        if self.open_loop:
            pi = policy_from_source(self.controller)
        else:
            self.require_state(x)
            pi = policy_from_block(self.controller)

        u = pi(x, t)

        return u

    # Views of what the planner produced — never a closed-loop simulation

    def plot_control_law(self, **kwargs):
        """
        Draw the feedback law over the problem's state box.

        The controller's own :meth:`~minilink.core.facades.plotting.SystemPlotting.plot_control_law`,
        with ``bounds`` filled from the problem's constraint set (or the
        plant's state box) and the colour limits from its input set, so that
        laws of one problem share one scale. Every keyword passes through
        (``x_axis``, ``y_axis``, ``u_axis``, ``ax``, ``show``, ...).
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import control_law_kwargs

        self.require_feedback_law()

        return self.controller.plot_control_law(**control_law_kwargs(self, kwargs))

    def plot_cost_to_go(self, **kwargs):
        """
        Draw the method's cost-to-go over two state axes, the others pinned at the goal.

        A heatmap of ``cost_to_go`` sampled on the problem's state box; see
        :func:`~minilink.planning.comparison.plot_cost_to_go` for ``axes``,
        ``anchor``, ``bounds``, ``jmax``, ``ax`` and ``show``.
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import plot_cost_to_go

        return plot_cost_to_go(self, **kwargs)

    def plot_trajectory(self, *, signals=("x", "u"), backend="matplotlib", show=True):
        """
        Draw ``trajectory`` on the problem's system: the plan, or the nominal rollout.

        The figure says which. A closed-loop simulation is not this: wire
        ``controller @ plant`` and plot that diagram's trajectory.
        """
        # A real cycle: comparison imports PlanningSolution from this module
        from minilink.planning.comparison import plot_solution_trajectory

        return plot_solution_trajectory(
            self, signals=signals, backend=backend, show=show
        )

    def plot_cost(self, *, backend="matplotlib", show=True):
        """Draw the running cost and ``J(t)`` along ``trajectory`` under the problem's cost."""
        return self.problem.sys.plot_cost(
            require_cost(self.problem),
            traj=self.require_trajectory(),
            backend=backend,
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

    def require_state(self, x) -> None:
        """Refuse a feedback policy read without a state."""
        if x is None:
            raise ValueError(
                "a feedback controller's policy u = pi(x, t) reads the state: "
                "pass x, policy(x, t)"
            )

    def require_feedback_law(self) -> None:
        """Refuse to draw the control law of an open-loop controller ``u = pi(t)``."""
        if self.open_loop:
            raise ValueError(
                "an open-loop policy u = pi(t) has no control law to draw; "
                "plot_trajectory draws the plan"
            )

    def require_trajectory(self) -> Trajectory:
        """Refuse a solution that carries no trajectory, naming ``solve(evaluate=True)``; return it."""
        if self.trajectory is None:
            raise ValueError(
                f"this {self.method} solution carries no trajectory; "
                "solve(evaluate=True) rolls the law out from the problem's start"
            )

        return self.trajectory


###############################################################################
# Internal machinery
###############################################################################


def solution_text(solution):
    law = "u = pi(t), open loop" if solution.open_loop else "u = pi(x), feedback"
    if solution.trajectory is None:
        trajectory = "not rolled out (solve with evaluate=True)"
    else:
        trajectory = (
            f"{solution.trajectory.n_samples} samples over "
            f"{solution.trajectory.tf - solution.trajectory.t0:.3g} s"
        )
    evaluation = "none" if solution.evaluation is None else str(solution.evaluation)
    success = "yes" if solution.success else "no"
    headline = f"PlanningSolution, method={solution.method}, success={success}"
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
