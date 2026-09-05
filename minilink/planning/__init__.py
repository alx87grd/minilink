"""Planning: problems, trajectory optimization, dynamic programming, search.

Band facade for short teaching imports::

    from minilink.planning import PlanningProblem, TrajectoryOptimizationPlanner
    from minilink.planning import StateSpaceGrid, DynamicProgrammingPlanner

Defining modules stay importable (``minilink.planning.problems``, ...).
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    # problem and results
    "PlanningProblem": ("minilink.planning.problems", "PlanningProblem"),
    "ProblemParameters": ("minilink.planning.problems", "ProblemParameters"),
    "TrajectoryPlan": ("minilink.planning.results", "TrajectoryPlan"),
    "PolicyPlan": ("minilink.planning.results", "PolicyPlan"),
    "SolveMetadata": ("minilink.planning.results", "SolveMetadata"),
    # trajectory optimization
    "TrajectoryOptimizationPlanner": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationPlanner",
    ),
    "TrajectoryOptimizationOptions": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationOptions",
    ),
    # dynamic programming (policy synthesis)
    "StateSpaceGrid": (
        "minilink.planning.policy_synthesis.discretizer",
        "StateSpaceGrid",
    ),
    "DynamicProgrammingPlanner": (
        "minilink.planning.policy_synthesis.dp",
        "DynamicProgrammingPlanner",
    ),
    "DynamicProgrammingOptions": (
        "minilink.planning.policy_synthesis.dp",
        "DynamicProgrammingOptions",
    ),
    "LookupTableController": (
        "minilink.planning.policy_synthesis.lookup_policy",
        "LookupTableController",
    ),
    "PolicyEvaluator": (
        "minilink.planning.policy_synthesis.policy_eval",
        "PolicyEvaluator",
    ),
    # search
    "RRTPlanner": ("minilink.planning.search.rrt", "RRTPlanner"),
    "RRTStarPlanner": ("minilink.planning.search.rrt_star", "RRTStarPlanner"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    try:
        module_path, attr = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    value = getattr(import_module(module_path), attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
