"""Five-link nullspace kinematic control — Pyro ``fivelinkrobot_kinematic_nullspace_controller``."""

import numpy as np

from minilink import FiveLinkPlanarManipulator, Source, SpeedControlledManipulator, Step
from minilink.control import TaskKinematicNullspace

TF = 8.0
NULLSPACE_STEP_TIME = 1.0

p_d = np.array([1.0, 1.0])
q0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
q1 = -np.array([1.0, 1.0, 1.0, 1.0, 1.0])
q2 = np.array([1.0, 1.0, 0.0, -1.0, -1.0])

arm = SpeedControlledManipulator.from_manipulator(FiveLinkPlanarManipulator())
arm.x0 = q0.copy()

ref_p = Source(2)
ref_p.name = "End-effector ref"
ref_p.params["value"] = p_d

ref_q = Step(
    initial_value=-q2,
    final_value=q2,
    step_time=NULLSPACE_STEP_TIME,
)
ref_q.name = "Joint posture ref"

ctl = TaskKinematicNullspace(
    arm,
    Kp=[1.0, 1.0],
    K_null=[10.0, 10.0, 10.0, 10.0, 10.0],
)

diagram = (ref_q + (ref_p >> (ctl @ arm))).autowire(strict=True)

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
# diagram.plot_trajectory()
# diagram.plot_trajectory(signals=((ref_p, "y"), (ref_q, "y"), (arm, "p"), (arm, "y")))
diagram.animate()
