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

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    Rod,
    TorqueArrow,
)


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
            "lc1": 1.0,
            "lc2": 1.0,
            "m1": 1.0,
            "m2": 1.0,
            "I1": 0.0,
            "I2": 0.0,
            "gravity": 9.81,
            "d1": 0.0,
            "d2": 0.0,
        }

        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.l2 = 1.0
        self.ground_half_width = 10.0
        self.camera_scale = 3.0

    def _trig(self, q):
        xp = array_module(q)
        c1 = xp.cos(q[0])
        s1 = xp.sin(q[0])
        c2 = xp.cos(q[1])
        s2 = xp.sin(q[1])
        c12 = xp.cos(q[0] + q[1])
        s12 = xp.sin(q[0] + q[1])
        return c1, s1, c2, s2, c12, s12

    def H(self, q, params=None):
        params = self.params if params is None else params
        _, _, c2, _, _, _ = self._trig(q)

        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        m1 = params["m1"]
        m2 = params["m2"]
        I1 = params["I1"]
        I2 = params["I2"]
        xp = array_module(q)

        # inertia couples the two links through the elbow angle (cos theta2)
        h00 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        h01 = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        h11 = m2 * lc2**2 + I2
        # fmt: off
        return xp.array([
            [h00, h01],
            [h01, h11],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        _, _, _, s2, _, _ = self._trig(q)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]
        xp = array_module(q)

        # Coriolis/centrifugal coupling driven by the elbow rate
        h = m2 * l1 * lc2 * s2
        # fmt: off
        return xp.array([
            [-h * dq[1], -h * (dq[0] + dq[1])],
            [ h * dq[0],                  0.0],
        ])
        # fmt: on

    def B(self, q, params=None):
        xp = array_module(q)
        return xp.eye(self.dof)

    def g(self, q, params=None):
        params = self.params if params is None else params
        _, s1, _, _, _, s12 = self._trig(q)

        m1 = params["m1"]
        m2 = params["m2"]
        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        gravity = params["gravity"]
        xp = array_module(q)

        # gravity restoring torque on each joint (sign matches the +y-down screen)
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity
        return xp.array([-g1 * s1 - g2 * s12, -g2 * s12])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1 = params["d1"]
        d2 = params["d2"]
        xp = array_module(dq)
        D = xp.diag(xp.array([d1, d2]))
        return D @ dq

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.l2
        radius = 0.08 * max(l1, l2)
        return {
            "world": [
                CustomLine(
                    [
                        [-self.ground_half_width, 0.0, 0.0],
                        [self.ground_half_width, 0.0, 0.0],
                    ],
                    color="black",
                    style="--",
                )
            ],
            "link1": [Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2)],
            "joint1": [
                Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
            ],
            "link2": [Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2)],
            "joint2": [
                Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
            ],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[: self.dof]
        l1 = self.params["l1"]
        l2 = self.l2
        _, s1, _, _, c12, s12 = self._trig(q)
        c1 = np.cos(q[0])
        p1 = np.array([l1 * s1, l1 * c1])
        p2 = p1 + np.array([l2 * s12, l2 * c12])
        theta1 = np.pi - q[0]
        theta12 = np.pi - (q[0] + q[1])
        rod1_angle = np.pi / 2.0 - q[0]
        rod2_angle = np.pi / 2.0 - q[0] - q[1]
        return {
            "link1": SE2(0.0, 0.0, theta1),
            "joint1": SE2(p1[0], p1[1], 0.0),
            "link2": SE2(p1[0], p1[1], theta12),
            "joint2": SE2(p2[0], p2[1], 0.0),
            "torque1": SE2(0.0, 0.0, rod1_angle),
            "torque2": SE2(p1[0], p1[1], rod2_angle),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        l1 = self.params["l1"]
        l2 = self.l2
        torque_radius = 0.2 * max(l1, l2)
        u_lim = float(self.inputs["u"].upper_bound[0])
        torque_scale = 2.0 * np.pi / 3.0 / u_lim
        return {
            "torque1": [
                TorqueArrow(
                    sweep=-u[0] * torque_scale,
                    radius=torque_radius,
                    head_ratio=0.4,
                    color="red",
                    linewidth=2,
                )
            ],
            "torque2": [
                TorqueArrow(
                    sweep=-u[1] * torque_scale,
                    radius=torque_radius,
                    head_ratio=0.4,
                    color="red",
                    linewidth=2,
                )
            ],
        }


class Acrobot(DoublePendulum):
    """Double pendulum actuated only at the elbow."""

    def __init__(self):
        super().__init__()
        self.name = "Acrobot"
        self.inputs.clear()
        self.add_input_port(
            "u",
            dim=1,
            labels=["tau"],
            units=["Nm"],
            lower_bound=[-10.0],
            upper_bound=[10.0],
        )

    def B(self, q, params=None):
        xp = array_module(q)
        return xp.array([[0.0], [1.0]])

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # only the elbow joint is actuated; emit a single torque arc at joint 2
        l1 = self.params["l1"]
        l2 = self.l2
        torque_radius = 0.2 * max(l1, l2)
        u_lim = float(self.inputs["u"].upper_bound[0])
        torque_scale = 2.0 * np.pi / 3.0 / u_lim
        return {
            "torque2": [
                TorqueArrow(
                    sweep=-u[0] * torque_scale,
                    radius=torque_radius,
                    head_ratio=0.4,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    # sys = DoublePendulum()
    sys = Acrobot()
    sys.x0 = np.array([0.2, -0.15, 0.0, 0.0])
    sys.inputs["u"].nominal_value[0] = 1.0
    # sys.inputs["u"].nominal_value[1] = 1.0
    sys.compute_trajectory(tf=4.0)
    sys.animate(time_factor_video=1.0)
