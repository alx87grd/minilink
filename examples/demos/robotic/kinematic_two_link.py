"""Two-link kinematic end-effector control — Pyro ``twolinkrobot_kinematic_controller``."""

import numpy as np

from minilink import KinematicManipulator, Source, TwoLinkManipulator
from minilink.control import TaskKinematic

p_d = np.array([0.5, 0.5])

arm = KinematicManipulator.from_manipulator(TwoLinkManipulator())
arm.x0 = np.array([-0.5, 0.2])

ref = Source(2)
ref.params["value"] = p_d

ctl = TaskKinematic(arm, Kp=[1.0, 1.0])

diagram = ref >> ctl @ arm

diagram.plot_diagram()
diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
