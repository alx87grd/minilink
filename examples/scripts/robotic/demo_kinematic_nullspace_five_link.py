"""Five-link nullspace kinematic control — Pyro ``fivelinkrobot_kinematic_nullspace_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_kinematic_nullspace_five_link.py

Two reference signals on a redundant arm (5 DOF, 2D task):

1. **End-effector ref** — constant ``p_d`` on ``ctl.r``.
2. **Nullspace joint ref** — ``ctl.r_null`` held at ``q0`` initially, then steps
   to ``q_null`` at ``NULLSPACE_STEP_TIME`` so the arm reconfigures in the
   nullspace while tracking ``p``.

Wiring::

    ref_p.y ───────────► ctl.r              (p_d)
    ref_q.y ───────────► ctl.r_null        (q_null, stepped late)
    plant.y ───────────► ctl.y              (q)
    ctl.u ─────────────► plant.u            (dq)
"""

import numpy as np

from minilink.blocks.sources import Source, Step
from minilink.control.robotic import TaskKinematicNullspace
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    SpeedControlledManipulator,
)

TF = 8.0
NULLSPACE_STEP_TIME = 5.0

p_d = np.array([1.0, 1.0])
q0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
q_null = np.array([1.0, 1.0, 0.0, -1.0, -1.0])

arm = SpeedControlledManipulator.from_manipulator(FiveLinkPlanarManipulator())
arm.x0 = q0.copy()

ref_p = Source(2)
ref_p.name = "End-effector ref"
ref_p.params["value"] = p_d

ref_q = Step(
    initial_value=-q_null,
    final_value=q_null,
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
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref_p, "y"), (ref_q, "y"), (arm, "p"), (arm, "y")))
diagram.animate()
