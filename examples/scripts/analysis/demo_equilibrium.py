"""Find a trim point where the dynamics vanish.

Run from the repo root::

    python examples/scripts/analysis/demo_equilibrium.py
"""

import numpy as np

from minilink.analysis.equilibria import find_equilibrium
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

plant = Pendulum()
x_guess = np.array([0.3, 0.0])  # near upright, zero rate

x_eq = find_equilibrium(plant, x_guess)
print("guess:      ", np.round(x_guess, 4))
print("equilibrium:", np.round(x_eq, 4))
print("f(x_eq):    ", np.round(plant.f(x_eq, plant.get_u_from_input_ports()), 6))
