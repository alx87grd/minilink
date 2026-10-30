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

        # Parameters
        self.params = {"m": M, "l": L, "d": D, "gravity": G}

    def H(self, q, params=None):
        params = self.params if params is None else params

        m = params["m"]
        l = params["l"]

        H = np.array([[m * l**2]])

        return H

    def g(self, q, params=None):
        params = self.params if params is None else params

        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]

        g = np.array([m * gravity * l * np.sin(q[0])])

        return g

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params

        b = params["d"]

        # d = b q̇
        d = b * v

        return d

    def forward_kinematics(self, q, params=None):
        params = self.params if params is None else params

        l = params["l"]

        p = np.array([l * np.sin(q[0]), -l * np.cos(q[0])])

        return p

    def J(self, q, params=None):
        params = self.params if params is None else params

        l = params["l"]

        # J = ∂p/∂q
        J = np.array([[l * np.cos(q[0])], [l * np.sin(q[0])]])

        return J


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
