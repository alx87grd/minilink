"""Planning: problems, trajectory optimization, dynamic programming, search.

Band facade for short teaching imports::

    from minilink.planning import PlanningProblem, TrajectoryOptimizationPlanner
    from minilink.planning import StateSpaceGrid, DynamicProgrammingPlanner

Defining modules stay importable (``minilink.planning.problems``, ...).
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

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

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
