"""The digital PID's closed-loop poles in z as its period grows: inside the unit circle, then out."""

import numpy as np

from minilink import PID, Pendulum
from minilink.analysis import linearize, plot_pzmap

plant = Pendulum()
pid = PID(Kp=20.0, Ki=5.0, Kd=4.0)

# The loop on the computer at each period, linearized at the hanging rest: x_{k+1} = A_d x_k
for dt in (0.01, 0.1, 0.2, 0.4):
    loop = (pid % dt) @ plant
    model = linearize(loop, x_bar=np.zeros(loop.n))
    radius = np.abs(np.linalg.eigvals(model.A())).max()
    verdict = "stable" if radius < 1.0 - 1e-9 else "not stable"
    print(f"dt = {dt:5.2f} s   max |z| = {radius:.4f}   {verdict}")

# The pole-zero map in z of the slowest computer, the unit circle drawn
plot_pzmap(model)
