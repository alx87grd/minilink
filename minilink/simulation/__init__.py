"""Time integration, static forcing, and scheduled step simulation."""

from __future__ import annotations

from importlib import import_module
from typing import Any

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

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    try:
        module_path, attr = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    value = getattr(import_module(module_path), attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
