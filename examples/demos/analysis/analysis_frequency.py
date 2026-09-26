"""Frequency response of a pendulum and of a PID loop around it: Bode with margins, Nyquist, step."""

import numpy as np

from minilink import PID, Pendulum
from minilink.analysis import bode, margins, plot_nyquist

plant = Pendulum()
plant.params["d"] = 1.0  # add damping to the default pendulum
plant.x0 = np.array(
    [0.0, 0.0]
)  # hanging down: the operating point every tool defaults to

# The plant alone, channel theta from the torque
w, magnitude_db, phase_deg = bode(plant, w=np.logspace(-1, 2, 5))
print("Linearized Bode response: y[0] / u[0]")
print("w [rad/s]   magnitude [dB]   phase [deg]")
for omega, mag, phase in zip(w, magnitude_db, phase_deg):
    print(f"{omega:8.3g}   {mag:14.3f}   {phase:11.2f}")

G = plant.transfer_function()
print("\nTransfer function:", G.name)
print("num:", np.round(G.numerator, 4), " den:", np.round(G.denominator, 4))

plant.plot_bode(margins=False)
plant.plot_pzmap()

# The loop gain L = C G with a PID compensator: margins read on the series diagram
C = PID(Kp=20.0, Ki=10.0, Kd=2.0, tau=0.05)
L = C >> plant
print("\nmargins of L = C G:", margins(L))
L.plot_bode()
plot_nyquist(L)

# The closed loop r -> theta: the junction e = r - theta is inserted by @
T = C @ plant
T.plot_diagram()
T.linearize().plot_step_response()  # a time response of the linear model
