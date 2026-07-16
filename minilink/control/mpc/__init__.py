"""
Receding-horizon MPC control blocks on trajectory optimization.

Product entry:
:func:`~minilink.control.mpc.controller.ModelPredictiveController`
with a
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
(auto-compiles the parametric NLP when needed). Terminal boundaries,
non-singleton ``X0``, and shooting transcriptions are not yet supported.
"""

from minilink.control.mpc.controller import Command, ModelPredictiveController
from minilink.control.mpc.utilities import (
    mpc_default_computer_x0,
    mpc_warm_start_guess,
)
from minilink.control.mpc.viz import (
    mpc_animation_overlays,
    mpc_plans_from_rollout,
)

__all__ = [
    "Command",
    "ModelPredictiveController",
    "mpc_animation_overlays",
    "mpc_default_computer_x0",
    "mpc_plans_from_rollout",
    "mpc_warm_start_guess",
]
