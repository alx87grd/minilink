"""Model predictive control: a planner's compiled face closed on the loop."""

# The entry is ModelPredictiveController(planner) on a TrajectoryOptimizationPlanner,
# closed as mpc % dt @ plant; PlanFollower applies its plan at a faster rate.
from minilink.planning.trajectory_optimization.mpc.predictive import (
    ModelPredictiveController,
    PlanFollower,
)
from minilink.planning.trajectory_optimization.mpc.viz import (
    mpc_animation_overlays,
    mpc_plans_from_rollout,
)

__all__ = [
    "ModelPredictiveController",
    "PlanFollower",
    "mpc_animation_overlays",
    "mpc_plans_from_rollout",
]
