"""Planning: problems, trajectory optimization, dynamic programming, reinforcement learning, search."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# The band facade for short teaching imports, e.g.
#
#     from minilink.planning import PlanningProblem, TrajectoryOptimizationPlanner
#     from minilink.planning import StateSpaceGrid, DynamicProgrammingPlanner, LQRPlanner
#     from minilink.planning import compare
#     from minilink.planning import ReinforcementLearningPlanner, TabularLearningPlanner
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
    # Linear-quadratic regulator (policy synthesis)
    "LQRPlanner": ("minilink.planning.policy_synthesis.lqr", "LQRPlanner"),
    # Trajectory optimization
    "TrajectoryOptimizationPlanner": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationPlanner",
    ),
    "TrajectoryOptimizationOptions": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationOptions",
    ),
    # Dynamic programming (policy synthesis)
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
    # Function approximation (cost-to-go fits, approximate dynamic programming)
    "QuadraticFeatures": (
        "minilink.planning.policy_synthesis.approximation",
        "QuadraticFeatures",
    ),
    "RadialBasisFeatures": (
        "minilink.planning.policy_synthesis.approximation",
        "RadialBasisFeatures",
    ),
    "LinearApproximator": (
        "minilink.planning.policy_synthesis.approximation",
        "LinearApproximator",
    ),
    # Reinforcement learning (policy synthesis) and Monte Carlo evaluation
    "ReinforcementLearningPlanner": (
        "minilink.planning.reinforcement_learning.planner",
        "ReinforcementLearningPlanner",
    ),
    "TabularLearningPlanner": (
        "minilink.planning.reinforcement_learning.tabular",
        "TabularLearningPlanner",
    ),
    "EpsilonGreedy": (
        "minilink.planning.reinforcement_learning.tabular",
        "EpsilonGreedy",
    ),
    "UCB": ("minilink.planning.reinforcement_learning.tabular", "UCB"),
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
