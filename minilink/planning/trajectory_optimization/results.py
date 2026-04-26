"""
Result objects for trajectory-optimization planners.

Trajectory optimization primarily returns a state-input trajectory, with
cost, optimizer status, feasibility diagnostics, and warm-start data kept
close to that trajectory-specific contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from minilink.core.trajectory import Trajectory
from minilink.optimization.problems import OptimizationResult


@dataclass(frozen=True)
class TrajectoryOptimizationResult:
    """
    Result returned by trajectory-optimization planners.

    Parameters
    ----------
    traj : Trajectory, optional
        Nominal state-input trajectory.
    cost : float, optional
        Final objective value.
    optimizer_result : OptimizationResult, optional
        Raw finite-dimensional optimizer result after transcription.
    feasibility : dict
        Constraint and set-feasibility diagnostics.
    warm_start : object, optional
        Backend/transcription-specific data for future solves.
    """

    problem: Any | None = None
    traj: Trajectory | None = None
    cost: float | None = None
    optimizer_result: OptimizationResult | None = None
    feasibility: dict[str, Any] = field(default_factory=dict)
    warm_start: Any | None = None
    success: bool = False
    message: str = ""
    stats: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_trajectory(
        cls,
        traj: Trajectory,
        *,
        problem: Any | None = None,
        cost: float | None = None,
        success: bool = True,
        message: str = "",
        stats: dict[str, Any] | None = None,
    ) -> TrajectoryOptimizationResult:
        """Create a successful trajectory-optimization result."""
        return cls(
            problem=problem,
            traj=traj,
            cost=cost,
            success=success,
            message=message,
            stats={} if stats is None else dict(stats),
        )

    @property
    def has_traj(self) -> bool:
        """Return ``True`` when a trajectory is available."""
        return self.traj is not None

    def require_traj(self) -> Trajectory:
        """Return the trajectory or raise a clear error."""
        if self.traj is None:
            raise ValueError(
                "This trajectory-optimization result does not contain a trajectory"
            )
        return self.traj
