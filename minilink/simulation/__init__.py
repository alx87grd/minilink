"""Simulation: the simulators that run a ``System`` through time, continuous, static or live."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.simulation import Simulator``; the defining modules stay importable
# (``minilink.simulation.simulator``, ...). Each exported name maps to its
# (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    "RealtimeSimulator": ("minilink.simulation.realtime", "RealtimeSimulator"),
    "Simulator": ("minilink.simulation.simulator", "Simulator"),
    "StaticSimulator": ("minilink.simulation.static_simulator", "StaticSimulator"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
