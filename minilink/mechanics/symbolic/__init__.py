"""
Symbolic multibody mechanics (SymPy).

Import convention
-----------------
* ``from minilink.mechanics import MechanicalSystem`` — NumPy simulation block.
* ``from minilink.mechanics.symbolic import MechanicalModel, MechanicalSystem`` —
  pre-EoM builder and **symbolic** EoM (SymPy). The symbolic class is not the
  same as the numeric one despite the shared name.

Dependencies: requires SymPy (``pip install minilink[symbolic]``).
"""

from .derivation import derive_kane, derive_lagrange
from .export import create_minilink_system, to_minilink
from .model import MechanicalModel
from .symbolic_system import MechanicalSystem
from .utils import (
    inertia_box,
    inertia_cylinder,
    inertia_rod,
    inertia_sphere,
    inertia_tensor,
    smart_simplify,
)

__all__ = [
    "MechanicalModel",
    "MechanicalSystem",
    "create_minilink_system",
    "derive_kane",
    "derive_lagrange",
    "inertia_box",
    "inertia_cylinder",
    "inertia_rod",
    "inertia_sphere",
    "inertia_tensor",
    "smart_simplify",
    "to_minilink",
]
