"""Optimization: generic nonlinear programs and the optimizer that solves them."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.optimization import MathematicalProgram, Optimizer``; the defining
# modules stay importable (``minilink.optimization.optimizer``, ...). Each exported
# name maps to its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    "MathematicalProgram": (
        "minilink.optimization.mathematical_program",
        "MathematicalProgram",
    ),
    "OptimizationResult": (
        "minilink.optimization.mathematical_program",
        "OptimizationResult",
    ),
    "Optimizer": ("minilink.optimization.optimizer", "Optimizer"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
