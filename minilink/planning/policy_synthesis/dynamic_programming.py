"""
Dynamic-programming policy-synthesis skeletons.

Dynamic programming consumes a deterministic planning problem and
solver-owned discretization options. The intended primary artifact is a
feedback policy packaged as a :class:`~minilink.core.system.StaticSystem`
(state in, control out).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.system import StaticSystem
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem


@dataclass(frozen=True)
class GridSpec:
    """
    Lightweight grid specification for DP-style solvers.
    """

    shape: tuple[int, ...]

    def __post_init__(self) -> None:
        shape = tuple(int(v) for v in self.shape)
        if any(v < 1 for v in shape):
            raise ValueError("Grid dimensions must be positive")
        object.__setattr__(self, "shape", shape)


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


class DynamicProgrammingPlanner(Planner):
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

    def compute_solution(self) -> StaticSystem:
        """
        Compute a closed-loop policy as a static system.

        TODO: User Architectural Review - implement grid construction,
        Bellman updates, and controller reconstruction after the high-level
        planning architecture is reviewed.
        """
        raise NotImplementedError("DP internals are deferred until architecture review")
