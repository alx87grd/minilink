"""Steer a UR5 tool setpoint with the keyboard — real-time game mode in 3D.

Run from repo root::

    python examples/demos/realtime/game_ur5_setpoint.py

The live input is the closed-loop diagram's boundary reference ``r`` (desired
tool position), not a joint torque: the same realtime loop that pushes a
cart-pole force drives a *setpoint* through a controller here, with no
special-casing. Keys slew the setpoint (``mode="rate"``) and the
``TaskImpedance`` controller tracks it::

    UP/DOWN    -> r[0]  (tool x)
    RIGHT/LEFT -> r[1]  (tool y)
    W/S        -> r[2]  (tool z)

The 3D view opens in meshcat; keep the small pygame window focused so it
receives the key events. ESC or closing that window stops the session and
returns the recorded ``Trajectory``.
"""

import numpy as np

from minilink.control.robotic import TaskImpedance
from minilink.dynamics.catalog.manipulators import UR5Manipulator
from minilink.simulation.realtime import PygameInput, RealtimeSimulator

# GRAVITY_COMP = False
GRAVITY = True

p0 = np.array([-0.5, -0.0, 0.5])
q_guess = np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])

arm = UR5Manipulator()

# arm.modal_analysis()

q0 = arm.inverse_kinematics(p0, q_guess=q_guess)
arm.x0 = arm.q2x(q0, np.zeros(6))


ctl = TaskImpedance(arm, gravity_comp=GRAVITY, show_task_force=True)
ctl.params["Kp"] = np.array([200.0, 200.0, 200.0])
ctl.params["Kd"] = np.array([40.0, 40.0, 40.0])
ctl.task_force_scale = 0.05

diagram = ctl @ arm  # boundary input r = desired tool position
diagram.plot_diagram()
diagram.inputs["r"].set_nominal_value(p0)  # rate-mode setpoint starts at the tool

rt_sim = RealtimeSimulator(
    diagram,
    frame_dt=1 / 30,
    # sim_dt=0.05 / 30,
    renderer="meshcat",
    is_3d=True,
    verbose=True,
    input=PygameInput(
        mode="rate",
        rate=0.3,  # setpoint slew in m/s while a key is held
        key_axes=[("up", "down"), ("right", "left"), ("w", "s")],
    ),
)
traj = rt_sim.run()

diagram.traj = traj
diagram.plot_trajectory()
