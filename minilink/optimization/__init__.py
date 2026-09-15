"""Generic nonlinear programs: ``MathematicalProgram`` + ``Optimizer``.

Band facade for short teaching imports::

    from minilink.optimization import MathematicalProgram, Optimizer

Defining modules stay importable (``minilink.optimization.optimizer``, ...).
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# name -> (module path, attribute)
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
