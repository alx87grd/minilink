"""Two-link arm joint impedance — Pyro ``twolinkrobot_joint_impedance_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_joint_impedance_two_link.py

Joint-space virtual spring-damper on ``[q, dq]`` via ``closed_loop_qdq``::

    ref.r ─────────────► ctl.r
    sys.q  ──┐
    sys.dq ──┴─ Mux ─► ctl.y
    ctl.u ──► sys.u          (τ = PD + optional g(q))

Same end-effector step task as :mod:`demo_task_impedance_two_link` (``p0`` → ``p1``),
with joint references from inverse kinematics.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.robotic import JointImpedance
from minilink.core.composition import closed_loop_qdq
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

ctl = JointImpedance(arm, gravity_comp=False)
ctl.params["Kp"] = np.array([20.0, 20.0])
ctl.params["Kd"] = np.array([5.0, 5.0])

diagram = ref >> closed_loop_qdq(ctl, arm)

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
