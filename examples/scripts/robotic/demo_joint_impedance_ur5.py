"""UR5 joint impedance with gravity compensation.

Run from repo root::

    python examples/scripts/robotic/demo_joint_impedance_ur5.py

Joint-space virtual spring-damper on ``[q, dq]`` via ``closed_loop_qdq``::

    ref.r ─────────────► ctl.r
    arm.q  ──┐
    arm.dq ──┴─ Mux ─► ctl.y
    ctl.u ──► arm.u          (tau = PD + g(q))
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.robotic import JointImpedance
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.manipulators import UR5Manipulator

STEP_TIME = 1.0
TF = 8.0

q0 = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
q1 = np.array([1.35, -1.2, 1.55, -1.4, 0.35, 0.25])

arm = UR5Manipulator()
arm.x0 = arm.q2x(q0, np.zeros(6))
ref = Step(initial_value=q0, final_value=q1, step_time=STEP_TIME)

ctl = JointImpedance(arm, gravity_comp=True)
ctl.params["Kp"] = np.array([10.0, 60.0, 20.0, 0.5, 0.1, 0.003])
ctl.params["Kd"] = np.array([3.0, 25.0, 8.0, 0.2, 0.05, 0.003])

diagram = ref >> closed_loop_qdq(ctl, arm)
diagram.plot_diagram()
diagram.compute_trajectory(tf=TF, n_steps=240, compile_backend="jax")
diagram.plot_trajectory()
diagram.animate(renderer="meshcat", is_3d=True)
