"""Two-link arm: a spring and a damper at the end effector, moved from one point to another."""

import numpy as np

from minilink import Step, TaskImpedance, TwoLinkManipulator

TF = 12.0  # [s]

# The arm, at rest with both joints at zero

arm = TwoLinkManipulator()
arm.x0 = np.zeros(4)

# The reference: the end effector's target steps from p_0 to p_1 at t = 4 s

p_0 = np.array([0.5, 0.5])  # [m]
p_1 = np.array([-0.5, 0.5])  # [m]

reference = Step(initial_value=p_0, final_value=p_1, step_time=4.0)

# The law: f = Kp (r − p) − Kd ṗ at the end effector, τ = J(q)ᵀ f at the joints

controller = TaskImpedance(
    arm,
    gravity_comp=False,
    Kp=[120.0, 120.0],
    Kd=[12.0, 12.0],
    show_task_force=True,
    task_force_scale=0.005,
)

closed_loop = reference >> controller @ arm
closed_loop.plot_diagram()

closed_loop.compute_trajectory(tf=TF)
closed_loop.plot_trajectory(signals=((reference, "y"), (arm, "p")))
closed_loop.animate()
