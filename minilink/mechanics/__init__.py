"""
Mechanics: rigid-body style blocks for minilink.

**TRL 1** — :class:`~minilink.mechanics.mechanical.MechanicalSystem` (NumPy) and
:class:`~minilink.mechanics.mechanical.JaxMechanicalSystem` (JAX), plus optional
SymPy under ``minilink.mechanics.symbolic``.
"""

from minilink.mechanics.mechanical import JaxMechanicalSystem, MechanicalSystem

__all__ = ["MechanicalSystem", "JaxMechanicalSystem"]
