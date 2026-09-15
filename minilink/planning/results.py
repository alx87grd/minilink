"""The planning solution: the policy, its cost-to-go, and the evidence for them."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation

# Public API


@dataclass(frozen=True)
class PlanningSolution:
    """
    What every planner returns: the optimal-control pair and the evidence for it.

    Parameters
    ----------
    policy : System
        The law. Feedback ``u = pi(x)`` is a controller block, ``policy @ plant``
        closes the loop; open loop ``u = pi(t)`` is a
        :class:`~minilink.blocks.sources.TrajectorySource`, ``policy >> plant``
        drives the plant.
    solver : record
        The planner's account of the solve, one dataclass per planner
        (``TrajectoryOptimizationRecord``, ``TreeSearchRecord``,
        ``ValueIterationRecord``, ``TabularLearningRecord``,
        ``ReinforcementLearningRecord``), each with a ``success`` flag.
    trajectory : Trajectory or None
        What the policy produces from the problem's start on the planner's
        grid with nominal parameters: the schedule of an open-loop planner,
        the rollout of a feedback law when the solve was asked to evaluate.
    evaluation : Evaluation or None
        The cost over the problem's draws under the one scoring contract
        (one trial on a deterministic problem), when the solve was asked to
        evaluate; ``None`` otherwise, or when the problem declares no cost.
    cost_to_go : callable or None
        ``J(x)`` where the method produces it: the interpolated table of
        dynamic programming and tabular learning, the critic of a learned law
        when it estimates the problem's own discount.
    """

    policy: System
    solver: object
    trajectory: Trajectory | None
    evaluation: Evaluation | None
    cost_to_go: Callable | None = None

    @property
    def success(self) -> bool:
        """The solver record's own success: feasible, converged, goal reached, finite weights."""
        return bool(self.solver.success)

    @property
    def open_loop(self) -> bool:
        """``True`` for a time-based policy (a source with no input port)."""
        return int(self.policy.m) == 0

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
