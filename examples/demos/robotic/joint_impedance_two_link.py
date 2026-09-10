"""Two-link arm joint impedance — Pyro ``twolinkrobot_joint_impedance_controller``."""

import numpy as np

from minilink import JointImpedance, Step, TwoLinkManipulator, closed_loop_qdq

p0 = np.array([0.5, 0.5])
p1 = np.array([-0.5, 0.5])
STEP_TIME = 4.0
TF = 12.0

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

q0 = arm.inverse_kinematics(p0)
q1 = arm.inverse_kinematics(p1)

ref = Step(initial_value=q0, final_value=q1, step_time=STEP_TIME)

ctl = JointImpedance(arm, gravity_comp=False, Kp=[20.0, 20.0], Kd=[5.0, 5.0])
ctl.plot_control_law()  # Kp·e + Kd·de plane (first joint)

diagram = ref >> closed_loop_qdq(ctl, arm)

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
