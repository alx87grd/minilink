"""
Beta compile-once MPC planning pipeline.

This package is experimental. It currently supports JAX direct collocation with
a singleton initial boundary ``X0`` whose point is supplied at runtime via
:meth:`~minilink.planning.mpc.planner.MPCPlanner.step`. Terminal boundaries,
non-singleton ``X0``, and shooting transcriptions are not yet supported.
"""

from minilink.planning.mpc.animation_overlays import mpc_animation_overlays
from minilink.planning.mpc.controller import (
    MPCStatelessController,
    mpc_stateless_controller,
)
from minilink.planning.mpc.options import MPCOptions
from minilink.planning.mpc.plan_reconstruct import mpc_plans_from_rollout
from minilink.planning.mpc.planner import MPCPlanner
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
    "mpc_animation_overlays",
    "MPCStatelessController",
    "MPCDirectCollocationTranscription",
    "MPCOptions",
    "MPCPlanner",
    "MPCStatefulController",
    "mpc_stateless_controller",
    "mpc_default_computer_x0",
    "mpc_plans_from_rollout",
    "mpc_stateful_controller",
    "mpc_warm_start_guess",
    "warm_start_guess_from_prev_plan",
]
