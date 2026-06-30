"""Two-link end-effector impedance — Pyro ``twolinkrobot_effector_impedance_controller``.

Run from repo root::

    python examples/scripts/robotic/demo_task_impedance_two_link.py

Task-space law uses the internal FK/Jacobian model; feedback is joint space::

    ref.r ─────────────► ctl.r          (desired p)
    sys.q  ──┐
    sys.dq ──┴─ Mux ─► ctl.y         ([q; dq])
    ctl.u ──► sys.u                    (τ)

End-effector step from ``p0 = (0.5, 0.5)`` to ``p1 = (-0.5, 0.5)`` at ``t = 4`` s.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.robotic import TaskImpedance
from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator

p0 = np.array([0.5, 0.5])
p1 = np.array([-0.5, 0.5])
STEP_TIME = 4.0
TF = 12.0

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

ref = Step(initial_value=p0, final_value=p1, step_time=STEP_TIME)

ctl = TaskImpedance(arm, gravity_comp=False)
ctl.params["Kp"] = np.array([120.0, 120.0])
ctl.params["Kd"] = np.array([12.0, 12.0])

diagram = ref >> ctl @ arm

diagram.plot_diagram()
diagram.compute_trajectory(tf=TF)
diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))
diagram.animate()
