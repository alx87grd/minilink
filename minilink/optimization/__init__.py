"""Generic nonlinear programs: ``MathematicalProgram`` + ``Optimizer``.

Band facade for short teaching imports::

    from minilink.optimization import MathematicalProgram, Optimizer

Defining modules stay importable (``minilink.optimization.optimizer``, ...).
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

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
