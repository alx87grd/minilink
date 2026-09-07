"""Find a trim point where the dynamics vanish."""

import numpy as np

from minilink import Pendulum

plant = Pendulum()
x_guess = np.array([0.3, 0.0])  # near upright, zero rate

x_eq = plant.find_equilibrium(x_guess)
print("guess:      ", np.round(x_guess, 4))
print("equilibrium:", np.round(x_eq, 4))
print("f(x_eq):    ", np.round(plant.f(x_eq, plant.get_u_from_input_ports()), 6))
