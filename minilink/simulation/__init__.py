"""Time integration, static forcing, and scheduled step simulation."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    "Computer": ("minilink.simulation.computer", "Computer"),
    "HybridSimResult": ("minilink.simulation.hybrid_simulator", "HybridSimResult"),
    "HybridSimulator": ("minilink.simulation.hybrid_simulator", "HybridSimulator"),
    "RealtimeSimulator": ("minilink.simulation.realtime", "RealtimeSimulator"),
    "Simulator": ("minilink.simulation.simulator", "Simulator"),
    "StaticSimulator": ("minilink.simulation.static_simulator", "StaticSimulator"),
    "StepSchedule": ("minilink.simulation.computer", "StepSchedule"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
