"""Steer a UR5 tool setpoint with the keyboard — real-time game mode in 3D."""

import numpy as np

from minilink import TaskImpedance, UR5Manipulator
from minilink.simulation import RealtimeSimulator
from minilink.simulation.realtime import PygameInput

########################################################
### Tunables
########################################################

# GRAVITY_COMP = False
GRAVITY = True

p0 = np.array([-0.5, -0.0, 0.5])
q_guess = np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])

########################################################
### Arm and task impedance
########################################################

arm = UR5Manipulator()

# arm.animate_modal()

q0 = arm.inverse_kinematics(p0, q_guess=q_guess)
arm.x0 = arm.q2x(q0, np.zeros(6))


ctl = TaskImpedance(
    arm,
    gravity_comp=GRAVITY,
    Kp=[200.0, 200.0, 200.0],
    Kd=[40.0, 40.0, 40.0],
    show_task_force=True,
    task_force_scale=0.05,
)

diagram = ctl @ arm
diagram.plot_diagram()
diagram.inputs["r"].nominal_value = np.array(p0, dtype=float)

########################################################
### Real-time game
########################################################

rt_sim = RealtimeSimulator(
    diagram,
    frame_dt=1 / 30,
    # sim_dt=0.05 / 30,
    renderer="meshcat",
    is_3d=True,
    verbose=True,
    input=PygameInput(
        mode="rate",
        rate=0.3,
        key_axes=[("up", "down"), ("right", "left"), ("w", "s")],
    ),
)
traj = rt_sim.run()
diagram.plot_trajectory(traj)
