"""UR5 task impedance: a spring and a damper at the hand of a six-axis arm, pulled through four points."""

import numpy as np

from minilink import TaskImpedance, TrajectorySource, UR5Manipulator

########################################################
### Tunables
########################################################

HOLD = 1.0  # [s]
DT = 3.0  # [s]

########################################################
### Hand trajectory
########################################################

waypoints = np.array(
    [
        [-0.5, 0.0, 0.5],
        [0.0, 0.5, 0.5],
        [-0.3, 0.3, 0.2],
        [-0.5, -0.3, 0.4],
        [0.5, -0.0, 0.5],
    ]
)  # [m]

t_knots = [0.0]
p_knots = [waypoints[0]]
t = 0.0
for p_from, p_to in zip(waypoints[:-1], waypoints[1:]):
    t += HOLD
    t_knots.append(t)
    p_knots.append(p_from)
    t += DT
    t_knots.append(t)
    p_knots.append(p_to)

target = TrajectorySource(t_knots, np.array(p_knots).T)

########################################################
### Arm at the first waypoint
########################################################

arm = UR5Manipulator()
q_start = arm.inverse_kinematics(waypoints[0], q_guess=[0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
arm.x0 = arm.q2x(q_start, np.zeros(6))

########################################################
### Task impedance
########################################################

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

########################################################
### Run and 3-D animation
########################################################

traj = closed_loop.compute_trajectory(tf=t_knots[-1] + HOLD, n_samples=260)
closed_loop.plot_trajectory(signals=((target, "y"), (arm, "p")))
closed_loop.animate(traj, renderer="meshcat", is_3d=True)
