"""Core: the systems and the mathematical objects every band builds on."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.core import DynamicSystem, DiagramSystem, Trajectory``; the defining
# modules stay importable (``minilink.core.system``, ...). Each exported name maps to
# its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Systems
    "System": ("minilink.core.system", "System"),
    "DynamicSystem": ("minilink.core.system", "DynamicSystem"),
    "StepSystem": ("minilink.core.discrete.step_system", "StepSystem"),
    "DiagramSystem": ("minilink.core.diagram", "DiagramSystem"),
    "StepDiagramSystem": ("minilink.core.discrete.step_diagram", "StepDiagramSystem"),
    # Data
    "Trajectory": ("minilink.core.trajectory", "Trajectory"),
    # Costs
    "CostFunction": ("minilink.core.costs", "CostFunction"),
    "QuadraticCost": ("minilink.core.costs", "QuadraticCost"),
    "TimeCost": ("minilink.core.costs", "TimeCost"),
    # Sets
    "Set": ("minilink.core.sets", "Set"),
    "InputSet": ("minilink.core.sets", "InputSet"),
    "BoxSet": ("minilink.core.sets", "BoxSet"),
    "BoxInputSet": ("minilink.core.sets", "BoxInputSet"),
    "BallSet": ("minilink.core.sets", "BallSet"),
    "SingletonSet": ("minilink.core.sets", "SingletonSet"),
    # Distributions
    "Distribution": ("minilink.core.distributions", "Distribution"),
    "Gaussian": ("minilink.core.distributions", "Gaussian"),
    "Uniform": ("minilink.core.distributions", "Uniform"),
    "Particles": ("minilink.core.distributions", "Particles"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
