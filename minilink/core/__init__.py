"""Core modeling and data abstractions.

Band facade for short teaching imports::

    from minilink.core import DynamicSystem, DiagramSystem, Trajectory
    from minilink.core import QuadraticCost, BoxSet

Defining modules stay importable (``minilink.core.system``, ...).
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    # systems
    "System": ("minilink.core.system", "System"),
    "DynamicSystem": ("minilink.core.system", "DynamicSystem"),
    "StepSystem": ("minilink.core.system", "StepSystem"),
    "DiagramSystem": ("minilink.core.diagram", "DiagramSystem"),
    "StepDiagramSystem": ("minilink.core.diagram", "StepDiagramSystem"),
    # controller markers
    "Controller": ("minilink.core.feedback", "Controller"),
    "DynamicController": ("minilink.core.feedback", "DynamicController"),
    # data
    "Trajectory": ("minilink.core.trajectory", "Trajectory"),
    # costs
    "CostFunction": ("minilink.core.costs", "CostFunction"),
    "QuadraticCost": ("minilink.core.costs", "QuadraticCost"),
    "TimeCost": ("minilink.core.costs", "TimeCost"),
    # sets
    "Set": ("minilink.core.sets", "Set"),
    "InputSet": ("minilink.core.sets", "InputSet"),
    "BoxSet": ("minilink.core.sets", "BoxSet"),
    "BoxInputSet": ("minilink.core.sets", "BoxInputSet"),
    "BallSet": ("minilink.core.sets", "BallSet"),
    "closed_loop_qdq": ("minilink.core.composition", "closed_loop_qdq"),
    "SingletonSet": ("minilink.core.sets", "SingletonSet"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
