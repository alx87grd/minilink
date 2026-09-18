"""The planning solution: the policy, its cost-to-go, and the evidence for them."""

from __future__ import annotations

import re
from collections.abc import Callable
from dataclasses import dataclass

from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.problems import PlanningProblem

# Public API


@dataclass(frozen=True)
class PlanningSolution:
    """
    What every planner returns: the optimal-control pair and the evidence for it.

    Everything here is the solver's own claim under its own model — the table
    on its grid, the Riccati form on the linear model, the critic's fit. A
    measurement of the law comes from the evaluators (``MonteCarloEvaluator``,
    ``PolicyEvaluator``) as its own object and is never written back here.

    Parameters
    ----------
    problem : PlanningProblem
        What was solved: the system, its sets, the cost and the horizon. A
        frozen record; its ``sys`` is a live reference, not a snapshot.
    policy : System
        The law. Feedback ``u = pi(x)`` is a controller block, ``policy @ plant``
        closes the loop; open loop ``u = pi(t)`` is a
        :class:`~minilink.blocks.sources.TrajectorySource`, ``policy >> plant``
        drives the plant; a finite-horizon design is feedback in time,
        ``u = pi(x, t)``.
    solver : record
        The planner's account of the solve, one dataclass per planner
        (``TrajectoryOptimizationRecord``, ``TreeSearchRecord``,
        ``ValueIterationRecord``, ``TabularLearningRecord``,
        ``ReinforcementLearningRecord``, ``RiccatiRecord``), each with a
        ``success`` flag and a one-line ``str``.
    trajectory : Trajectory or None
        What the policy produces from the problem's start on the planner's
        grid with nominal parameters: the schedule of an open-loop planner,
        the rollout of a feedback law when the solve was asked to evaluate.
        It is the planner's evidence, not a closed-loop simulation: close the
        loop in the script, ``policy @ plant``, and simulate that.
    evaluation : Evaluation or None
        The cost over the problem's draws under the one scoring contract
        (one trial on a deterministic problem), when the solve was asked to
        evaluate; ``None`` otherwise, or when the problem declares no cost.
    cost_to_go : callable or None
        ``J(x)``, the method's own estimate under its own model, where the
        method produces one: the interpolated table of dynamic programming
        and tabular learning, the Riccati form of a linear-quadratic design,
        the critic of a learned law when it estimates the problem's own
        discount. On a stochastic problem it is the expected cost-to-go
        when the method takes the distributions.
    """

    problem: PlanningProblem
    policy: System
    solver: object
    trajectory: Trajectory | None
    evaluation: Evaluation | None
    cost_to_go: Callable | None = None

    @property
    def success(self) -> bool:
        """The solver record's own success: feasible, converged, goal reached, finite weights, stable."""
        return bool(self.solver.success)

    @property
    def open_loop(self) -> bool:
        """``True`` for a time-based policy (a source with no input port)."""
        return int(self.policy.m) == 0

    @property
    def method(self) -> str:
        """The planning method, read from the solver record (``"value iteration"``, ``"riccati"``, ...)."""
        name = type(self.solver).__name__.removesuffix("Record")
        return re.sub(r"(?<!^)(?=[A-Z])", " ", name).lower()

    def __str__(self) -> str:
        law = "u = pi(t), open loop" if self.open_loop else "u = pi(x), feedback"
        if self.trajectory is None:
            trajectory = "not rolled out (solve with evaluate=True)"
        else:
            trajectory = (
                f"{self.trajectory.n_samples} samples over "
                f"{self.trajectory.time_duration:.3g} s"
            )
        evaluation = "none" if self.evaluation is None else str(self.evaluation)
        return "\n".join(
            [
                f"policy: {self.policy.name} ({law})",
                f"trajectory: {trajectory}",
                f"evaluation: {evaluation}",
                f"solver: {self.solver}",
            ]
        )

    def _repr_pretty_(self, p, cycle):
        p.text("..." if cycle else str(self))

    # Views of what the planner produced — never a closed-loop simulation

    def plot_control_law(self, **kwargs):
        """
        Draw the feedback law over the problem's state box.

        The policy's own :meth:`~minilink.core.feedback.Controller.plot_control_law`,
        with ``bounds`` filled from the problem's constraint set (or the
        plant's state box) and the colour limits from its input set, so that
        laws of one problem share one scale. Every keyword passes through
        (``x_axis``, ``y_axis``, ``u_axis``, ``ax``, ``show``, ...).
        """
        from minilink.planning.comparison import control_law_kwargs

        if self.open_loop:
            raise ValueError(
                "an open-loop policy u = pi(t) has no control law to draw; "
                "plot_trajectory draws the plan"
            )
        return self.policy.plot_control_law(**control_law_kwargs(self, kwargs))

    def plot_cost_to_go(self, **kwargs):
        """
        Draw the method's cost-to-go over two state axes, the others pinned at the goal.

        A heatmap of ``cost_to_go`` sampled on the problem's state box; see
        :func:`~minilink.planning.comparison.plot_cost_to_go` for ``axes``,
        ``anchor``, ``bounds``, ``jmax``, ``ax`` and ``show``.
        """
        from minilink.planning.comparison import plot_cost_to_go

        return plot_cost_to_go(self, **kwargs)

    def plot_trajectory(self, *, signals=("x", "u"), backend="matplotlib", show=True):
        """
        Draw ``trajectory`` on the problem's system: the plan, or the nominal rollout.

        The figure says which. A closed-loop simulation is not this: wire
        ``policy @ plant`` and plot that diagram's trajectory.
        """
        from minilink.planning.comparison import plot_solution_trajectory

        return plot_solution_trajectory(
            self, signals=signals, backend=backend, show=show
        )

    def plot_cost(self, *, backend="matplotlib", show=True):
        """Draw the running cost and ``J(t)`` along ``trajectory`` under the problem's cost."""
        return self.problem.sys.plot_cost(
            self.problem.require_cost(),
            traj=self.require_trajectory(),
            backend=backend,
            show=show,
        )

    def require_trajectory(self) -> Trajectory:
        """``trajectory``, or a clear error naming ``solve(evaluate=True)``."""
        if self.trajectory is None:
            raise ValueError(
                f"this {self.method} solution carries no trajectory; "
                "solve(evaluate=True) rolls the law out from the problem's start"
            )
        return self.trajectory
