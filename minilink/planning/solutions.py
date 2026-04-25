"""
Standard solution envelope for deterministic planning.

Different planners can produce different artifacts: a nominal trajectory,
an open-loop control law, a search tree, a value function, or a policy.
The :class:`PlanningSolution` object keeps those outputs under one
reviewable contract without forcing every solver to return the same data.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from minilink.core.trajectory import Trajectory


@dataclass(frozen=True)
class PlanningSolution:
    """
    Result returned by deterministic planners.

    Parameters
    ----------
    problem : PlanningProblem, optional
        Problem that generated this solution.
    traj : Trajectory, optional
        Nominal state-input trajectory.
    controller : object, optional
        Open-loop control law, lookup-table policy, or Minilink controller.
    path, tree, value, policy : object, optional
        Planner-family-specific artifacts.
    cost : float, optional
        Final objective value or start-state cost-to-go.
    success : bool
        Whether the planner found a solution.
    message : str
        Human-readable solver status.
    stats : dict
        Solver statistics and debug metadata.
    """

    problem: Any | None = None
    traj: Trajectory | None = None
    controller: Any | None = None
    path: Any | None = None
    tree: Any | None = None
    value: Any | None = None
    policy: Any | None = None
    cost: float | None = None
    success: bool = False
    message: str = ""
    stats: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_trajectory(
        cls,
        traj: Trajectory,
        *,
        problem: Any | None = None,
        controller: Any | None = None,
        cost: float | None = None,
        success: bool = True,
        message: str = "",
        stats: dict[str, Any] | None = None,
    ) -> PlanningSolution:
        """Create a solution whose primary artifact is a trajectory."""
        return cls(
            problem=problem,
            traj=traj,
            controller=controller,
            cost=cost,
            success=success,
            message=message,
            stats={} if stats is None else dict(stats),
        )

    @classmethod
    def from_path(
        cls,
        path: Any,
        *,
        problem: Any | None = None,
        traj: Trajectory | None = None,
        controller: Any | None = None,
        tree: Any | None = None,
        cost: float | None = None,
        success: bool = True,
        message: str = "",
        stats: dict[str, Any] | None = None,
    ) -> PlanningSolution:
        """Create a solution whose primary artifact is a search path."""
        return cls(
            problem=problem,
            traj=traj,
            controller=controller,
            path=path,
            tree=tree,
            cost=cost,
            success=success,
            message=message,
            stats={} if stats is None else dict(stats),
        )

    @classmethod
    def from_policy(
        cls,
        policy: Any,
        *,
        problem: Any | None = None,
        value: Any | None = None,
        controller: Any | None = None,
        cost: float | None = None,
        success: bool = True,
        message: str = "",
        stats: dict[str, Any] | None = None,
    ) -> PlanningSolution:
        """Create a solution whose primary artifact is a policy."""
        return cls(
            problem=problem,
            controller=controller,
            value=value,
            policy=policy,
            cost=cost,
            success=success,
            message=message,
            stats={} if stats is None else dict(stats),
        )

    @property
    def has_traj(self) -> bool:
        """Return ``True`` when a trajectory is available."""
        return self.traj is not None

    @property
    def has_controller(self) -> bool:
        """Return ``True`` when a controller or control law is available."""
        return self.controller is not None

    def require_traj(self) -> Trajectory:
        """Return the trajectory or raise a clear error."""
        if self.traj is None:
            raise ValueError("This planning solution does not contain a trajectory")
        return self.traj

    def as_controller(self) -> Any:
        """Return the controller/control law or raise a clear error."""
        if self.controller is None:
            raise ValueError("This planning solution does not contain a controller")
        return self.controller
