"""Bode plot, pole-zero map and transfer function of a linearized pendulum."""

import numpy as np

from minilink import Pendulum

plant = Pendulum()
plant.params.update({"m": 1.0, "l": 1.0, "I": 1.0, "gravity": 9.81, "d": 1.0})

x_bar = np.array([0.0, 0.0])  # upright equilibrium (unstable for this model)
channel = dict(of=("y", 1), wrt="u")  # velocity dtheta from the torque

w, magnitude_db, phase_deg = plant.bode(x_bar, w=np.logspace(-1, 2, 5), **channel)
print("Linearized Bode response: y[1] / u[0]")
print("w [rad/s]   magnitude [dB]   phase [deg]")
for omega, mag, phase in zip(w, magnitude_db, phase_deg):
    print(f"{omega:8.3g}   {mag:14.3f}   {phase:11.2f}")

zeros, poles, gain = plant.pzmap(x_bar, **channel)
print("\nZero-pole-gain form:")
print("zeros:", np.round(zeros, 4))
print("poles:", np.round(poles, 4))
print("gain:", np.round(gain, 4))

G = plant.transfer_function(x_bar, **channel)
print("\nTransfer function:", G.name)
print("num:", np.round(G.numerator, 4), " den:", np.round(G.denominator, 4))

plant.plot_bode(x_bar, **channel)
plant.plot_pzmap(x_bar, **channel)
