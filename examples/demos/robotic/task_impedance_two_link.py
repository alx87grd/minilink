"""Two-link end-effector impedance — Pyro ``twolinkrobot_effector_impedance_controller``."""

import numpy as np

from minilink import Step, TaskImpedance, TwoLinkManipulator

p0 = np.array([0.5, 0.5])
p1 = np.array([-0.5, 0.5])
STEP_TIME = 4.0
TF = 12.0

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

ref = Step(initial_value=p0, final_value=p1, step_time=STEP_TIME)

ctl = TaskImpedance(arm, gravity_comp=False, show_task_force=True)
ctl.params["Kp"] = np.array([120.0, 120.0])
ctl.params["Kd"] = np.array([12.0, 12.0])
ctl.task_force_scale = 0.005

diagram = ref >> ctl @ arm

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
