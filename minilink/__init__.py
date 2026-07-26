"""minilink — Python/JAX block-diagram toolbox for dynamical systems.

Teaching imports (shortest → deepest)::

    from minilink import Pendulum, ImpedanceController
    from minilink.catalog import CartPole
    from minilink.control.lqr import lqr
    from minilink.analysis.linearize import linearize

See DESIGN.md §2 (Public imports). Root is a selective prelude, not the full API.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS: dict[str, tuple[str, str]] = {
    # core
    "DiagramSystem": ("minilink.core.diagram", "DiagramSystem"),
    "DynamicSystem": ("minilink.core.system", "DynamicSystem"),
    "StepSystem": ("minilink.core.system", "StepSystem"),
    "System": ("minilink.core.system", "System"),
    # plants (teaching set — grow slowly)
    "CartPole": ("minilink.catalog", "CartPole"),
    "InvertedPendulum": ("minilink.catalog", "InvertedPendulum"),
    "Pendulum": ("minilink.catalog", "Pendulum"),
    # control
    "ImpedanceController": ("minilink.control", "ImpedanceController"),
    "lqr": ("minilink.control.lqr", "lqr"),
    # sources
    "Step": ("minilink.blocks.sources", "Step"),
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
