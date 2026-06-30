"""Two-link arm with computed torque — Pyro ``twolinkrobot_computed_torque_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_computed_torque_two_link.py

Joint-space feedback with PD gains inside :class:`ComputedTorqueController`::

    ref.r ─────────────► ctl.r
    sys.y ─────────────► ctl.y              ([q; dq])
    ctl.u ──► sys.u

Same end-effector step task as :mod:`demo_task_impedance_two_link` (``p0`` → ``p1``),
with joint references from inverse kinematics.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import ComputedTorqueController
from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator

p0 = np.array([0.5, 0.5])
p1 = np.array([-0.5, 0.5])
STEP_TIME = 4.0
TF = 12.0

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

q0 = arm.inverse_kinematics(p0)
q1 = arm.inverse_kinematics(p1)

ref = Step(initial_value=q0, final_value=q1, step_time=STEP_TIME)

ctl = ComputedTorqueController(arm, tracking_ref=False)
ctl.params["Kp"] = np.array([20.0, 20.0])
ctl.params["Kd"] = np.array([10.0, 10.0])

diagram = ref >> ctl @ arm

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
