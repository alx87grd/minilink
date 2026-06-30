"""Two-link kinematic end-effector control — Pyro ``twolinkrobot_kinematic_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_kinematic_two_link.py

Velocity-controlled plant with task-space kinematic law::

    ref.r ─────────────► ctl.r              (p_d)
    plant.y ───────────► ctl.y              (q)
    ctl.u ─────────────► plant.u            (dq)
"""

import numpy as np

from minilink.blocks.sources import Source
from minilink.control.robotic import TaskKinematic
from minilink.dynamics.catalog.manipulators.arms import (
    SpeedControlledManipulator,
    TwoLinkManipulator,
)

p_d = np.array([0.5, 0.5])

arm = SpeedControlledManipulator.from_manipulator(TwoLinkManipulator())
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
