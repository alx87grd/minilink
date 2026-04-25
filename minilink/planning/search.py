"""
Search-style deterministic planner skeletons.

RRT and related planners belong in this family. They may use random
sampling internally, but the problem statement remains deterministic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from minilink.planning.planner import SearchPlanner
from minilink.planning.problems import PlanningProblem
from minilink.planning.solutions import PlanningSolution


@dataclass(frozen=True)
class SearchOptions:
    """
    Common high-level options for search planners.
    """

    dt: float
    max_nodes: int = 2_000
    seed: int | None = None

    def __post_init__(self) -> None:
        dt = float(self.dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite scalar")
        if isinstance(self.max_nodes, bool) or int(self.max_nodes) < 1:
            raise ValueError("max_nodes must be a positive integer")
        object.__setattr__(self, "dt", dt)
        object.__setattr__(self, "max_nodes", int(self.max_nodes))


class RRTPlanner(SearchPlanner):
    """
    Skeleton for a deterministic-problem RRT planner.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        dt: float,
        max_nodes: int = 2_000,
        seed: int | None = None,
        sampler: Any | None = None,
    ) -> None:
        super().__init__(problem)
        self.require_goal()
        self.options = SearchOptions(dt=dt, max_nodes=max_nodes, seed=seed)
        self.sampler = sampler

    def compute_solution(self) -> PlanningSolution:
        """
        Compute an RRT solution.

        TODO: User Architectural Review - implement sampling, steering,
        nearest-neighbor search, and trajectory reconstruction after the
        high-level deterministic planning architecture is reviewed.
        """
        raise NotImplementedError("RRT internals are deferred until architecture review")
