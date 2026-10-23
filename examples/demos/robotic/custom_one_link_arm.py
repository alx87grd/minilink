"""A one-link arm written in the manipulator form, then held up by a task-space impedance."""

import numpy as np

from minilink import TaskImpedance
from minilink.dynamics import Manipulator

M = 1.0  # [kg]  tip mass
L = 0.5  # [m]   link length
D = 0.1  # [N m s] joint damping
G = 9.81  # [m/s²]


class OneLinkArm(Manipulator):
    def __init__(self):
        super().__init__(dof=1, task_dim=2)

    def H(self, q, params=None):
        # H = m l²
        return np.array([[M * L**2]])

    def g(self, q, params=None):
        # g(q) = m g l sin(q)
        return np.array([M * G * L * np.sin(q[0])])

    def d(self, q, v, u=None, t=0.0, params=None):
        # d = b q̇
        return D * v

    def forward_kinematics(self, q, params=None):
        # p = [l sin(q), −l cos(q)]
        return np.array([L * np.sin(q[0]), -L * np.cos(q[0])])

    def J(self, q, params=None):
        # J = ∂p/∂q
        return np.array([[L * np.cos(q[0])], [L * np.sin(q[0])]])


arm = OneLinkArm()
arm.x0 = np.array([1.0, 0.0])

# The unforced arm swings down
arm.compute_trajectory(tf=5.0)
arm.plot_trajectory()

# A spring-damper on the tip, plus the gravity g(q) it reads from the model, holds it
ctl = TaskImpedance(arm, gravity_comp=True)
ctl.inputs["r"].nominal_value = arm.forward_kinematics(np.array([0.5]))
loop = ctl @ arm
loop.compute_trajectory(tf=5.0)
loop.plot_trajectory()
