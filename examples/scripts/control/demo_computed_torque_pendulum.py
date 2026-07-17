"""Pendulum regulation with computed torque — Pyro ``simple_pendulum_with_computed_torque``.

Run from repo root::

    python examples/scripts/control/demo_computed_torque_pendulum.py

Built-in outer PD + inverse dynamics in one block::

    ref.r ─────────────► ct.r              ([q_d; dq_d])
    plant.y ───────────► ct.y              ([q; dq])
    ct.u ──────────────► plant.u           (τ)
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import ComputedTorqueController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

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

diagram = ref >> ct @ plant

diagram.plot_diagram()
diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
diagram.animate()
