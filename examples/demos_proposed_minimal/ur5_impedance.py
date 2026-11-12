"""UR5 task impedance: a spring and a damper at the hand of a six-axis arm, pulled through four points."""

import numpy as np

from minilink import TaskImpedance, TrajectorySource, UR5Manipulator

# The hand's target: it glides from one waypoint to the next in 1.5 s and holds 0.5 s
# at each

waypoints = np.array(
    [
        [-0.5, 0.0, 0.5],
        [0.0, 0.5, 0.5],
        [-0.3, 0.3, 0.2],
        [-0.5, -0.3, 0.4],
        [-0.5, 0.0, 0.5],
    ]
)  # [m]

t_knots = [0.0]
p_knots = [waypoints[0]]
for p_from, p_to in zip(waypoints[:-1], waypoints[1:]):
    t_knots += [t_knots[-1] + 0.5, t_knots[-1] + 2.0]
    p_knots += [p_from, p_to]

target = TrajectorySource(t_knots, np.array(p_knots).T)

# The arm: a UR5 with its rigid-body dynamics, started with its hand on the first
# waypoint

arm = UR5Manipulator()
q_start = arm.inverse_kinematics(waypoints[0], q_guess=[0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
arm.x0 = arm.q2x(q_start, np.zeros(6))

# The law: a spring and a damper between the hand and its target, f = Kp (r − p) − Kd ṗ,
# sent to the joints as τ = J(q)ᵀ f + g(q). The spring is soft, so the hand trails its
# target and the force shows

controller = TaskImpedance(
    arm,
    gravity_comp=True,
    Kp=[100.0, 100.0, 100.0],
    Kd=[20.0, 20.0, 20.0],
    show_task_force=True,
    task_force_scale=0.08,
)

closed_loop = target >> controller @ arm
closed_loop.plot_diagram()

# The run: the hand against its target, then the arm in 3-D

traj = closed_loop.compute_trajectory(tf=t_knots[-1] + 0.5, n_samples=260)
closed_loop.plot_trajectory(signals=((target, "y"), (arm, "p")))
closed_loop.animate(traj, renderer="meshcat", is_3d=True)
