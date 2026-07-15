"""
Beta receding-horizon MPC control blocks on trajectory optimization.

Controllers accept a
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
(or duck-typed compatible planner). Call
:meth:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner.compile_parametric_program`
or let the controller auto-compile so ticks bind + solve only. Terminal
boundaries, non-singleton ``X0``, and shooting transcriptions are not yet
supported.
"""

from minilink.planning.mpc.animation_overlays import mpc_animation_overlays
from minilink.planning.mpc.command import Command
from minilink.planning.mpc.controller import (
    MPCStatelessController,
    mpc_stateless_controller,
)
from minilink.planning.mpc.model_predictive_controller import (
    ModelPredictiveController,
)
from minilink.planning.mpc.plan_reconstruct import mpc_plans_from_rollout
from minilink.planning.mpc.step_block import (
    MPCStatefulController,
    mpc_stateful_controller,
)
from minilink.planning.mpc.transcription import MPCDirectCollocationTranscription
from minilink.planning.mpc.warm_start import (
    mpc_default_computer_x0,
    mpc_warm_start_guess,
    warm_start_guess_from_prev_plan,
)

__all__ = [
    "Command",
    "ModelPredictiveController",
    "mpc_animation_overlays",
    "MPCStatelessController",
    "MPCDirectCollocationTranscription",
    "MPCStatefulController",
    "mpc_stateless_controller",
    "mpc_default_computer_x0",
    "mpc_plans_from_rollout",
    "mpc_stateful_controller",
    "mpc_warm_start_guess",
    "warm_start_guess_from_prev_plan",
]
