"""Model predictive control: receding-horizon control blocks on trajectory optimization."""

# The product entry is :func:`~minilink.control.mpc.controller.ModelPredictiveController`
# with a :class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`.
# ``compile_backend='jax'`` auto-compiles a parametric NLP once (bind + solve per tick);
# ``compile_backend='numpy'`` or ``'direct'`` rebuilds the NLP each replan tick (no JAX
# required). Terminal boundaries, non-singleton ``X0`` and shooting transcriptions are
# not supported.
from minilink.control.mpc.controller import Command, ModelPredictiveController
from minilink.control.mpc.viz import (
    mpc_animation_overlays,
    mpc_plans_from_rollout,
)

__all__ = [
    "Command",
    "ModelPredictiveController",
    "mpc_animation_overlays",
    "mpc_plans_from_rollout",
]
