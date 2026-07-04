"""
Beta compile-once MPC planning pipeline.

This package is experimental. It currently supports JAX direct collocation with
a singleton initial boundary ``X0`` whose point is supplied at runtime via
:meth:`~minilink.planning.mpc.planner.MPCPlanner.step`. Terminal boundaries,
non-singleton ``X0``, and shooting transcriptions are not yet supported.
"""

from minilink.planning.mpc.options import MPCOptions
from minilink.planning.mpc.planner import MPCPlanner
from minilink.planning.mpc.transcription import MPCDirectCollocationTranscription

__all__ = [
    "MPCDirectCollocationTranscription",
    "MPCOptions",
    "MPCPlanner",
]
