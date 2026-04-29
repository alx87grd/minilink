"""Prototype: 1-D floating mass with a force arrow.

Demonstrates the :class:`Arrow` primitive — the red arrow scales with the
applied force ``F`` via the transform matrix's scaling factor.
"""

import numpy as np

from minilink.dynamics.catalog.msd.floating_mass import FloatingMass1D

# Plant: unit mass on a frictionless rail
mass = FloatingMass1D()
mass.params["m"] = 1.0
mass.x0 = np.array([0.0, 0.0])

# mass.game()
mass.game(renderer="matplotlib")
# mass.game(renderer="meshcat")
