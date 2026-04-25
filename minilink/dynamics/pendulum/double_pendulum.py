"""
Pyro-ported double pendulum (2 actuators, 2 links).

This matches SherbyRobotics/pyro ``DoublePendulum`` in ``pyro/dynamic/pendulum.py``:
* ``q = [theta1, theta2]`` where ``theta1`` is the first joint and ``theta2`` is
  measured relative to the first link
* the same ``H``, ``C``, ``B``, ``g``, and linear joint damping in ``d``

Visualization maps Pyro's line-based torque arcs to :class:`TorqueArrow` using
the same sweep scaling as the tutorial single pendulum in ``pendulum.py``.
"""

import numpy as np

from minilink.graphical.primitives import (
    Circle,
    CustomLine,
    Rod,
    TorqueArrow,
    pose2d_matrix,
    torque_pose2d_matrix,
)
from minilink.mechanics.mechanical import MechanicalSystem


class DoublePendulum(MechanicalSystem):
    """
    Two-link actuated pendulum in Pyro's manipulator-equation convention.

    Notes
    -----
    The kinematic canvas uses a Pyro-style mapping from joint angles to rod
    orientations: world rod heading uses ``(pi/2 - theta)`` so the default
    ``Rod`` (local -Y) matches Pyro's ``forward_kinematic_lines`` convention.
    """

    def __init__(self):
        super().__init__(dof=2, actuators=2)

        self.name = "Double Pendulum"
        self.params = {
            "l1": 1.0,
            "l2": 1.0,
            "lc1": 1.0,
            "lc2": 1.0,
            "m1": 1.0,
            "m2": 1.0,
            "I1": 0.0,
            "I2": 0.0,
            "gravity": 9.81,
            "d1": 0.0,
            "d2": 0.0,
            "ground_half_width": 10.0,
        }

        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def _trig(self, q):
        c1 = np.cos(q[0])
        s1 = np.sin(q[0])
        c2 = np.cos(q[1])
        s2 = np.sin(q[1])
        c12 = np.cos(q[0] + q[1])
        s12 = np.sin(q[0] + q[1])
        return c1, s1, c2, s2, c12, s12

    def H(self, q, params=None):
        params = params or self.params
        _, _, c2, _, _, _ = self._trig(q)

        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        m1 = params["m1"]
        m2 = params["m2"]
        I1 = params["I1"]
        I2 = params["I2"]

        H = np.zeros((self.dof, self.dof))
        H[0, 0] = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        H[1, 0] = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        H[0, 1] = H[1, 0]
        H[1, 1] = m2 * lc2**2 + I2
        return H

    def C(self, q, dq, params=None):
        params = params or self.params
        _, _, _, s2, _, _ = self._trig(q)

        h = params["m2"] * params["l1"] * params["lc2"] * s2

        C = np.zeros((self.dof, self.dof))
        C[0, 0] = -h * dq[1]
        C[1, 0] = h * dq[0]
        C[0, 1] = -h * (dq[0] + dq[1])
        C[1, 1] = 0.0
        return C

    def B(self, q, params=None):
        return np.eye(self.dof)

    def g(self, q, params=None):
        params = params or self.params
        _, s1, _, _, _, s12 = self._trig(q)

        g1 = (params["m1"] * params["lc1"] + params["m2"] * params["l1"]) * params[
            "gravity"
        ]
        g2 = params["m2"] * params["lc2"] * params["gravity"]

        G = np.zeros(self.dof)
        G[0] = -g1 * s1 - g2 * s12
        G[1] = -g2 * s12
        return G

    def d(self, q, dq, params=None):
        params = params or self.params
        D = np.diag([params["d1"], params["d2"]])
        return D @ dq

    def _joint_positions(self, q, params=None):
        params = params or self.params
        l1 = params["l1"]
        l2 = params["l2"]
        _, s1, _, _, c12, s12 = self._trig(q)
        c1 = np.cos(q[0])

        p0 = np.array([0.0, 0.0])
        p1 = np.array([l1 * s1, l1 * c1])
        p2 = p1 + np.array([l2 * s12, l2 * c12])
        return p0, p1, p2

    def get_kinematic_geometry(self):
        params = self.params
        l1 = params["l1"]
        l2 = params["l2"]
        radius = 0.08 * max(l1, l2)
        torque_radius = 0.2 * max(l1, l2)

        return [
            CustomLine(
                [
                    [-params["ground_half_width"], 0.0, 0.0],
                    [params["ground_half_width"], 0.0, 0.0],
                ],
                color="black",
                style="--",
            ),
            Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2),
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2),
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            TorqueArrow(radius=torque_radius, head_ratio=0.4, color="red", linewidth=2),
            TorqueArrow(radius=torque_radius, head_ratio=0.4, color="red", linewidth=2),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = np.asarray(x[: self.dof])
        u = np.asarray(u)
        p0, p1, p2 = self._joint_positions(q)

        theta1 = np.pi - q[0]
        theta12 = np.pi - (q[0] + q[1])
        rod1_angle = np.pi / 2.0 - q[0]
        rod2_angle = np.pi / 2.0 - q[0] - q[1]
        u_lim = float(self.inputs["u"].upper_bound[0])
        torque_scale = 2.0 * np.pi / 3.0 / u_lim

        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(p0[0], p0[1], theta1),
            pose2d_matrix(p1[0], p1[1], 0.0),
            pose2d_matrix(p1[0], p1[1], theta12),
            pose2d_matrix(p2[0], p2[1], 0.0),
            torque_pose2d_matrix(p0[0], p0[1], rod1_angle, -u[0] * torque_scale),
            torque_pose2d_matrix(p1[0], p1[1], rod2_angle, -u[1] * torque_scale),
        ]


if __name__ == "__main__":

    double_pendulum = DoublePendulum()
    double_pendulum.x0 = np.array([0.2, -0.15, 0.0, 0.0])
    double_traj = double_pendulum.compute_forced(
        lambda t: np.array([0.5 * np.sin(t), 0.25 * np.cos(1.5 * t)]),
        tf=4.0,
        n_steps=160,
        show=False,
        verbose=False,
    )
    double_pendulum.animate(double_traj)
    double_pendulum.game(renderer="meshcat")
