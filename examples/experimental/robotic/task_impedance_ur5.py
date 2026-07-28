"""UR5 end-effector impedance with gravity compensation.

Run from repo root::

    python examples/experimental/robotic/task_impedance_ur5.py

Task-space spring-damper on the tool position; feedback is joint space::

    ref.r ─────────────► ctl.r          (desired p)
    arm.q  ──┐
    arm.dq ──┴─ Mux ─► ctl.y         ([q; dq])
    ctl.u ──► arm.u                    (τ = Jᵀ(Kp e + Kd ė) + g(q))

Initial and target poses are tool points ``[x, y, z]``. A ready-pose IK
guess seeds ``x0`` (6-DOF position IK is underdetermined).
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.robotic import TaskImpedance
from minilink.dynamics.catalog.manipulators import UR5Manipulator

STEP_TIME = 1.0
TF = 6.0

p0 = np.array([-0.5, -0.0, 0.5])
p1 = np.array([0.0, 0.5, 0.5])
q_guess = np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])

arm = UR5Manipulator()
q0 = arm.inverse_kinematics(p0, q_guess=q_guess)
arm.x0 = arm.q2x(q0, np.zeros(6))

ref = Step(initial_value=p0, final_value=p1, step_time=STEP_TIME)

ctl = TaskImpedance(arm, gravity_comp=True, show_task_force=True)
ctl.params["Kp"] = np.array([200.0, 200.0, 200.0])
ctl.params["Kd"] = np.array([40.0, 40.0, 40.0])
ctl.task_force_scale = 0.005

diagram = ref >> ctl @ arm
diagram.plot_diagram()
diagram.compute_trajectory(tf=TF, n_steps=180, compile_backend="jax")
# diagram.plot_trajectory()
diagram.plot_trajectory(signals=((ref, "y"), (arm, "p")))

diagram.animate(renderer="meshcat")
# diagram.animate(native=False, is_3d=True)
