"""
Beta receding-horizon MPC control blocks on trajectory optimization.

Product entry:
:func:`~minilink.planning.mpc.model_predictive_controller.ModelPredictiveController`
with a
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
(auto-compiles the parametric NLP when needed). Terminal boundaries,
non-singleton ``X0``, and shooting transcriptions are not yet supported.
"""

from minilink.planning.mpc.animation_overlays import mpc_animation_overlays
from minilink.planning.mpc.command import Command
from minilink.planning.mpc.model_predictive_controller import (
    ModelPredictiveController,
)
from minilink.planning.mpc.plan_reconstruct import mpc_plans_from_rollout
from minilink.planning.mpc.warm_start import (
    mpc_default_computer_x0,
    mpc_warm_start_guess,
    warm_start_guess_from_prev_plan,
)

__all__ = [
    "Command",
    "ModelPredictiveController",
    "mpc_animation_overlays",
    "mpc_default_computer_x0",
    "mpc_plans_from_rollout",
    "mpc_warm_start_guess",
    "warm_start_guess_from_prev_plan",
]
