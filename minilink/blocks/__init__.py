"""Blocks: the plant-agnostic wiring blocks, sources, routing, filters and nonlinearities."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.blocks import Step, Integrator, Error, Sum, Saturation``; the defining
# modules stay importable (``minilink.blocks.sources``, ...). Each exported name maps to
# its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Basic
    "Integrator": ("minilink.blocks.basic", "Integrator"),
    # Sources
    "Source": ("minilink.blocks.sources", "Source"),
    "Step": ("minilink.blocks.sources", "Step"),
    "Sine": ("minilink.blocks.sources", "Sine"),
    "TrajectorySource": ("minilink.blocks.sources", "TrajectorySource"),
    "WhiteNoise": ("minilink.blocks.sources", "WhiteNoise"),
    # Routing
    "Demux": ("minilink.blocks.routing", "Demux"),
    "Error": ("minilink.blocks.routing", "Error"),
    "Gain": ("minilink.blocks.routing", "Gain"),
    "Mux": ("minilink.blocks.routing", "Mux"),
    "Sum": ("minilink.blocks.routing", "Sum"),
    # Nonlinear
    "Saturation": ("minilink.blocks.nonlinear", "Saturation"),
    # Transfer functions and filters
    "LowPassFilter": ("minilink.blocks.filters", "LowPassFilter"),
    "TransferFunction": ("minilink.blocks.transfer_function", "TransferFunction"),
    "Lead": ("minilink.blocks.transfer_function", "Lead"),
    # Neural
    "NeuralNetwork": ("minilink.blocks.neural", "NeuralNetwork"),
    "MLP": ("minilink.blocks.neural", "MLP"),
    # Discrete
    "ZOHHold": ("minilink.blocks.step", "ZOHHold"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
