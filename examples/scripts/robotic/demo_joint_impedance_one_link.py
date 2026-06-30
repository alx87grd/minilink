"""One-link arm joint impedance — Pyro ``onelinkrobot_joint_impedance_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_joint_impedance_one_link.py

Wiring (``closed_loop_qdq`` inserts ``Mux(q, dq) → controller.y``)::

    ref.r ─────────────► impedance.r
    plant.q  ──┐
    plant.dq ──┴─ Mux ─► impedance.y
    impedance.u ───────► plant.u
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.robotic import JointImpedance
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.manipulators.arms import OneLinkManipulator

arm = OneLinkManipulator()
arm.x0 = np.array([0.4, 0.0])

ref = Step(
    initial_value=np.array([0.0]),
    final_value=np.array([np.pi / 2]),
    step_time=3.0,
)

impedance = JointImpedance(arm, gravity_comp=True)
impedance.params["Kp"] = np.array([60.0])
impedance.params["Kd"] = np.array([8.0])

diagram = ref >> closed_loop_qdq(impedance, arm)

diagram.plot_diagram()
diagram.compute_trajectory(tf=6.0)
diagram.plot_trajectory()
diagram.animate()
