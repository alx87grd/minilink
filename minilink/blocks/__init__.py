"""Plant-agnostic wiring blocks — sources, routing, filters, nonlinearities.

Band facade for short teaching imports::

    from minilink.blocks import Step, Integrator, Sum, Saturation
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    # basic
    "Integrator": ("minilink.blocks.basic", "Integrator"),
    # sources
    "Source": ("minilink.blocks.sources", "Source"),
    "Step": ("minilink.blocks.sources", "Step"),
    "TrajectorySource": ("minilink.blocks.sources", "TrajectorySource"),
    "WhiteNoise": ("minilink.blocks.sources", "WhiteNoise"),
    # routing
    "Demux": ("minilink.blocks.routing", "Demux"),
    "Gain": ("minilink.blocks.routing", "Gain"),
    "Mux": ("minilink.blocks.routing", "Mux"),
    "Sum": ("minilink.blocks.routing", "Sum"),
    # nonlinear
    "DeadZone": ("minilink.blocks.nonlinear", "DeadZone"),
    "Relay": ("minilink.blocks.nonlinear", "Relay"),
    "Saturation": ("minilink.blocks.nonlinear", "Saturation"),
    # transfer functions and filters
    "LowPassFilter": ("minilink.blocks.filters", "LowPassFilter"),
    "NotchFilter": ("minilink.blocks.filters", "NotchFilter"),
    "TransferFunction": ("minilink.blocks.transfer_function", "TransferFunction"),
    "Washout": ("minilink.blocks.filters", "Washout"),
    # neural
    "NeuralNetwork": ("minilink.blocks.neural", "NeuralNetwork"),
    # discrete
    "ZOHHold": ("minilink.blocks.step", "ZOHHold"),
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
