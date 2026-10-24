"""Dynamics: the textbook model forms a plant is written in, and the plant catalog."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The band facade of the modelling abstractions a student subclasses, e.g.
#
#     from minilink.dynamics import MechanicalSystem, Manipulator, LTISystem
#
# The catalog plants are ``minilink.catalog``. Each exported name maps to its
# (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    "MechanicalSystem": (
        "minilink.dynamics.abstraction.mechanical",
        "MechanicalSystem",
    ),
    "GeneralizedMechanicalSystem": (
        "minilink.dynamics.abstraction.generalized_mechanical",
        "GeneralizedMechanicalSystem",
    ),
    "Manipulator": ("minilink.dynamics.abstraction.manipulator", "Manipulator"),
    "StateSpaceSystem": (
        "minilink.dynamics.abstraction.state_space",
        "StateSpaceSystem",
    ),
    "LTISystem": ("minilink.dynamics.abstraction.state_space", "LTISystem"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
