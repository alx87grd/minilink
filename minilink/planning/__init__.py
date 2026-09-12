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
    "StochasticPlanningProblem": (
        "minilink.planning.problems",
        "StochasticPlanningProblem",
    ),
    "as_stochastic": ("minilink.planning.problems", "as_stochastic"),
    "Gaussian": ("minilink.planning.distributions", "Gaussian"),
    "Uniform": ("minilink.planning.distributions", "Uniform"),
    "Particles": ("minilink.planning.distributions", "Particles"),
    "Sampler": ("minilink.planning.distributions", "Sampler"),
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
    # reinforcement learning (policy synthesis) and Monte Carlo evaluation
    "ReinforcementLearningPlanner": (
        "minilink.planning.reinforcement_learning.planner",
        "ReinforcementLearningPlanner",
    ),
    "MonteCarloEvaluator": ("minilink.planning.evaluation", "MonteCarloEvaluator"),
    "MonteCarloReport": ("minilink.planning.evaluation", "MonteCarloReport"),
    # spatial scene: tracks, collision geometry, cost shaping
    "ReferenceTrack": ("minilink.planning.spatial.track", "ReferenceTrack"),
    "from_waypoints": ("minilink.planning.spatial.paths", "from_waypoints"),
    "Scene": ("minilink.planning.spatial.scene", "Scene"),
    "bind": ("minilink.planning.spatial.collision", "bind"),
    "car_outline": ("minilink.planning.spatial.collision", "car_outline"),
    "point_probe": ("minilink.planning.spatial.collision", "point_probe"),
    "quadratic_excess": ("minilink.planning.spatial.shaping", "quadratic_excess"),
    "quadratic_hinge": ("minilink.planning.spatial.shaping", "quadratic_hinge"),
    "inverse_barrier": ("minilink.planning.spatial.shaping", "inverse_barrier"),
    "TrackCorridorOverlay": (
        "minilink.planning.spatial.overlays",
        "TrackCorridorOverlay",
    ),
    "plot_track": ("minilink.planning.spatial.plotting", "plot_track"),
    # search
    "RRTPlanner": ("minilink.planning.search.rrt", "RRTPlanner"),
    "RRTStarPlanner": ("minilink.planning.search.rrt_star", "RRTStarPlanner"),
    "RRTOptions": ("minilink.planning.search.rrt", "RRTOptions"),
    "KinodynamicExtender": (
        "minilink.planning.search.extenders",
        "KinodynamicExtender",
    ),
    "SteeringExtender": ("minilink.planning.search.extenders", "SteeringExtender"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
