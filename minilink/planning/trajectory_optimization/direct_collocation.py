"""
Direct-collocation trajectory-optimization skeletons.

This module reserves the direct-collocation API shape without implementing
the nonlinear program. It records where grid/transcription settings and
optimization backends live, while :class:`PlanningProblem` remains a pure
deterministic problem description.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from minilink.optimization.backends import OptimizationBackend
from minilink.optimization.scipy import ScipyMinimizeBackend
from minilink.planning.costs import CostFunction
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.base import TrajectoryOptimizationPlanner
from minilink.planning.trajectory_optimization.results import (
    TrajectoryOptimizationResult,
)


@dataclass(frozen=True)
class DirectCollocationOptions:
    """
    Grid and backend options for direct-collocation transcriptions.

    These are solver options, not fields on :class:`PlanningProblem`.
    """

    tf: float
    n_steps: int
    compile_backend: str = "numpy"
    optimizer_backend: OptimizationBackend = field(
        default_factory=ScipyMinimizeBackend
    )

    def __post_init__(self) -> None:
        tf = float(self.tf)
        if not np.isfinite(tf) or tf <= 0.0:
            raise ValueError("tf must be a positive finite scalar")
        if isinstance(self.n_steps, bool) or int(self.n_steps) < 2:
            raise ValueError("n_steps must be an integer greater than or equal to 2")
        object.__setattr__(self, "tf", tf)
        object.__setattr__(self, "n_steps", int(self.n_steps))

    @property
    def t(self) -> np.ndarray:
        """Uniform transcription time grid."""
        return np.linspace(0.0, self.tf, self.n_steps)


class DirectCollocationTrajectoryOptimization(TrajectoryOptimizationPlanner):
    """
    High-level direct-collocation planner skeleton.

    The concrete nonlinear programming implementation is intentionally
    deferred so the planning architecture can be reviewed first.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        tf: float,
        n_steps: int,
        optimizer_backend: OptimizationBackend | None = None,
        compile_backend: str = "numpy",
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.options = DirectCollocationOptions(
            tf=tf,
            n_steps=n_steps,
            compile_backend=compile_backend,
            optimizer_backend=(
                ScipyMinimizeBackend()
                if optimizer_backend is None
                else optimizer_backend
            ),
        )

    @classmethod
    def from_system(
        cls,
        sys: Any,
        *,
        x_start: np.ndarray,
        x_goal: np.ndarray,
        cost: CostFunction,
        tf: float,
        n_steps: int,
        optimizer_backend: OptimizationBackend | None = None,
        compile_backend: str = "numpy",
    ) -> DirectCollocationTrajectoryOptimization:
        """Convenience constructor building a :class:`PlanningProblem`."""
        problem = PlanningProblem(
            sys=sys,
            x_start=x_start,
            x_goal=x_goal,
            cost=cost,
        )
        return cls(
            problem,
            tf=tf,
            n_steps=n_steps,
            optimizer_backend=optimizer_backend,
            compile_backend=compile_backend,
        )

    def compute_solution(self) -> TrajectoryOptimizationResult:
        """
        Compute a direct-collocation result.

        TODO: User Architectural Review - implement decision packing,
        dynamics residuals, set-to-constraint wrappers, and backend solve
        through a transcription object after reviewing this architecture.
        """
        raise NotImplementedError(
            "Direct collocation internals are deferred until architecture review"
        )
