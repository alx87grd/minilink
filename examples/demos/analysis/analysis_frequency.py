"""Frequency response of a linearized pendulum: Bode with margins, Nyquist, transfer function."""

import numpy as np

from minilink import Pendulum

plant = Pendulum()
plant.params["d"] = 1.0  # add damping to the default pendulum

x_bar = np.array([0.0, 0.0])  # upright equilibrium (unstable for this model)
channel = dict(of=("y", 1), wrt="u")  # velocity dtheta from the torque

w, magnitude_db, phase_deg = plant.bode(x_bar, w=np.logspace(-1, 2, 5), **channel)
print("Linearized Bode response: y[1] / u[0]")
print("w [rad/s]   magnitude [dB]   phase [deg]")
for omega, mag, phase in zip(w, magnitude_db, phase_deg):
    print(f"{omega:8.3g}   {mag:14.3f}   {phase:11.2f}")

print("\nmargins of the channel taken as a loop gain:", plant.margins(x_bar, **channel))

G = plant.transfer_function(x_bar, **channel)
print("\nTransfer function:", G.name)
print("num:", np.round(G.numerator, 4), " den:", np.round(G.denominator, 4))

plant.plot_bode(x_bar, **channel)
plant.plot_pzmap(x_bar, **channel)
plant.plot_nyquist(x_bar, **channel)
plant.plot_step_response(x_bar, **channel)
