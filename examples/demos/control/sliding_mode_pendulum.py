"""Pendulum regulation with Pyro sliding-mode control — continuous-time closed loop."""

import numpy as np

from minilink import Pendulum, SlidingModeController, Step, closed_loop_qdq

plant = Pendulum(length=1.0, mass=1.0)
model = Pendulum(length=1.0, mass=0.5)
plant.x0 = np.array([1.0, 0.0])

ref = Step(
    initial_value=np.array([np.pi, 0.0]),
    final_value=np.array([0.0, 0.0]),
    step_time=0.5,
)

smc = SlidingModeController(model, lam=20.0, gain=8.0, nab=0.15)
smc.plot_control_law()  # switching surface over (q, dq)

diagram = ref >> closed_loop_qdq(smc, plant)

diagram.plot_diagram()
# discontinuous loop: the automatic grid uses Euler with a small dt (see DESIGN)
traj = diagram.compute_trajectory(tf=10.0)
# traj = diagram.compute_trajectory(tf=10.0, dt=0.01, solver="euler")
diagram.plot_trajectory()
# diagram.animate()
