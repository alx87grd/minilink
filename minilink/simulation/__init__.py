"""Simulation: the simulators that run a ``System`` through time, continuous, hybrid, static or stepped."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.simulation import Simulator``; the defining modules stay importable
# (``minilink.simulation.simulator``, ...). Each exported name maps to its
# (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    "Computer": ("minilink.simulation.discrete.computer", "Computer"),
    "HybridSimResult": (
        "minilink.simulation.discrete.hybrid_simulator",
        "HybridSimResult",
    ),
    "HybridSimulator": (
        "minilink.simulation.discrete.hybrid_simulator",
        "HybridSimulator",
    ),
    "RealtimeSimulator": ("minilink.simulation.realtime", "RealtimeSimulator"),
    "Simulator": ("minilink.simulation.simulator", "Simulator"),
    "StaticSimulator": ("minilink.simulation.static_simulator", "StaticSimulator"),
    "StepSchedule": ("minilink.simulation.discrete.computer", "StepSchedule"),
    "hybrid_closed_loop": (
        "minilink.core.discrete.hybrid_composition",
        "hybrid_closed_loop",
    ),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
