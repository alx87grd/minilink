"""
Dynamic-programming planner skeletons.

Dynamic programming consumes a deterministic :class:`PlanningProblem`
and solver-owned discretization options, then returns policy/value
artifacts through :class:`~minilink.planning.solutions.PlanningSolution`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.planning.planner import PolicyPlanner
from minilink.planning.problems import PlanningProblem
from minilink.planning.solutions import PlanningSolution


@dataclass(frozen=True)
class DynamicProgrammingOptions:
    """
    Grid options for deterministic dynamic programming.
    """

    x_grid_shape: tuple[int, ...]
    u_grid_shape: tuple[int, ...]
    dt: float

    def __post_init__(self) -> None:
        x_grid_shape = tuple(int(v) for v in self.x_grid_shape)
        u_grid_shape = tuple(int(v) for v in self.u_grid_shape)
        if any(v < 2 for v in x_grid_shape):
            raise ValueError("Each state grid dimension must be at least 2")
        if any(v < 1 for v in u_grid_shape):
            raise ValueError("Each input grid dimension must be at least 1")
        dt = float(self.dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite scalar")
        object.__setattr__(self, "x_grid_shape", x_grid_shape)
        object.__setattr__(self, "u_grid_shape", u_grid_shape)
        object.__setattr__(self, "dt", dt)


class DynamicProgrammingPlanner(PolicyPlanner):
    """
    Skeleton for deterministic dynamic programming.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        x_grid_shape: tuple[int, ...],
        u_grid_shape: tuple[int, ...],
        dt: float,
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.options = DynamicProgrammingOptions(
            x_grid_shape=x_grid_shape,
            u_grid_shape=u_grid_shape,
            dt=dt,
        )

    def compute_solution(self) -> PlanningSolution:
        """
        Compute a dynamic-programming policy solution.

        TODO: User Architectural Review - implement grid construction,
        Bellman updates, and controller reconstruction after the high-level
        planning architecture is reviewed.
        """
        raise NotImplementedError("DP internals are deferred until architecture review")
