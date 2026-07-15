"""Deploy / RAS command payload from :class:`ModelPredictiveController`."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.planning.results import TrajectoryPlan


@dataclass(frozen=True)
class Command:
    """
    One replan tick result for deploy or debugging.

    ``plan.trajectory.t`` is plan-local time with zero at the solve instant.
    Absolute solve time is ``t_solve``.
    """

    plan: TrajectoryPlan
    k: int
    t_solve: float
    u_ff: np.ndarray
    x_ff: np.ndarray
    z: np.ndarray
    success: bool

    @property
    def plan_flat(self) -> np.ndarray:
        """Flattened ``(t, x, u)`` from :meth:`TrajectoryPlan.to_flat`."""
        return self.plan.to_flat()
