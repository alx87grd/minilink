"""Planning: problems and solutions, dynamic programming, trajectory optimization, search."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# The band facade for short teaching imports, e.g.
#
#     from minilink.planning import PlanningProblem, TrajectoryOptimizationPlanner
#     from minilink.planning import StateSpaceGrid, DynamicProgrammingPlanner, LQRPlanner
#     from minilink.planning import compare
#     from minilink.planning import QuadraticFeatures, LinearApproximator
#
# The defining modules stay importable (``minilink.planning.problems``, ...). Each
# exported name maps to its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Problem and results
    "PlanningProblem": ("minilink.planning.problems", "PlanningProblem"),
    "ProblemParameters": ("minilink.planning.problems", "ProblemParameters"),
    "StochasticPlanningProblem": (
        "minilink.planning.problems",
        "StochasticPlanningProblem",
    ),
    "as_stochastic": ("minilink.planning.problems", "as_stochastic"),
    "Gaussian": ("minilink.core.distributions", "Gaussian"),
    "Uniform": ("minilink.core.distributions", "Uniform"),
    "Particles": ("minilink.core.distributions", "Particles"),
    "PlanningSolution": ("minilink.planning.results", "PlanningSolution"),
    "compare": ("minilink.planning.comparison", "compare"),
    "Comparison": ("minilink.planning.comparison", "Comparison"),
    # Linear-quadratic regulator
    "LQRPlanner": ("minilink.planning.dynamic_programming.lqr", "LQRPlanner"),
    # Trajectory optimization
    "TrajectoryOptimizationPlanner": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationPlanner",
    ),
    "TrajectoryOptimizationOptions": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationOptions",
    ),
    # Dynamic programming
    "StateSpaceGrid": (
        "minilink.planning.dynamic_programming.discretizer",
        "StateSpaceGrid",
    ),
    "DynamicProgrammingPlanner": (
        "minilink.planning.dynamic_programming.dp",
        "DynamicProgrammingPlanner",
    ),
    "DynamicProgrammingOptions": (
        "minilink.planning.dynamic_programming.dp",
        "DynamicProgrammingOptions",
    ),
    "LookupTableController": (
        "minilink.planning.dynamic_programming.lookup_policy",
        "LookupTableController",
    ),
    "PolicyEvaluator": (
        "minilink.planning.dynamic_programming.policy_eval",
        "PolicyEvaluator",
    ),
    # Function approximation (cost-to-go fits, approximate dynamic programming)
    "QuadraticFeatures": (
        "minilink.planning.dynamic_programming.approximation",
        "QuadraticFeatures",
    ),
    "RadialBasisFeatures": (
        "minilink.planning.dynamic_programming.approximation",
        "RadialBasisFeatures",
    ),
    "LinearApproximator": (
        "minilink.planning.dynamic_programming.approximation",
        "LinearApproximator",
    ),
    # Monte Carlo evaluation
    "MonteCarloEvaluator": ("minilink.planning.evaluation", "MonteCarloEvaluator"),
    "Evaluation": ("minilink.planning.evaluation", "Evaluation"),
    # Spatial scene: tracks, collision geometry, cost shaping
    "ReferenceTrack": ("minilink.planning.spatial.track", "ReferenceTrack"),
    "circuit_waypoints": (
        "minilink.planning.spatial.paths",
        "circuit_waypoints",
    ),
    "from_waypoints": ("minilink.planning.spatial.paths", "from_waypoints"),
    "Sphere": ("minilink.core.geometry", "Sphere"),
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
    # Search
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
