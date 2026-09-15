"""Two-link arm with computed torque — Pyro ``twolinkrobot_computed_torque_controller``."""

import numpy as np

from minilink import ComputedTorqueController, Step, TwoLinkManipulator

p0 = np.array([0.5, 0.5])
p1 = np.array([-0.5, 0.5])
STEP_TIME = 4.0
TF = 12.0

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

q0 = arm.inverse_kinematics(p0)
q1 = arm.inverse_kinematics(p1)

ref = Step(initial_value=q0, final_value=q1, step_time=STEP_TIME)

ctl = ComputedTorqueController(
    arm, tracking_ref=False, Kp=[20.0, 20.0], Kd=[10.0, 10.0]
)
ctl.plot_control_law()  # τ1 over (q0, dq0)

diagram = ref >> ctl @ arm

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
