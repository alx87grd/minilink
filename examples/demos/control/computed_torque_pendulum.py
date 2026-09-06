"""Pendulum regulation with computed torque — Pyro ``simple_pendulum_with_computed_torque``."""

import numpy as np

from minilink import ComputedTorqueController, Pendulum, Step

plant = Pendulum()
plant.x0 = np.array([np.pi + 0.25, 0.0])

ref = Step(
    initial_value=np.array([0.0, 0.0]),
    final_value=np.array([np.pi, 0.0]),
    step_time=0.0,
)

ct = ComputedTorqueController(plant)
ct.params["Kp"] = np.array([25.0])
ct.params["Kd"] = np.array([8.0])
ct.plot_control_law()  # τ over (q, dq)

diagram = ref >> ct @ plant

diagram.plot_diagram()
diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
diagram.animate()
