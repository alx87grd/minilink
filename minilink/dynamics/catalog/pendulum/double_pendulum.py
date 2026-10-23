"""Double pendulum: two links, two actuators, the second angle relative to the first link (pyro port)."""

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

###############################################################################
# Public API
###############################################################################


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

        # Parameters
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

        # Signal labels and units
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the EoM): the second link's drawn length, the
        # ground line and the camera
        self.l2 = 1.0
        self.ground_half_width = 10.0
        self.camera_scale = 3.0

    def trig(self, q):
        """Shorthands ``c1, s1, c2, s2, c12, s12`` of the joint angles and their sum."""
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
        xp = array_module(q)

        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        m1 = params["m1"]
        m2 = params["m2"]
        I1 = params["I1"]
        I2 = params["I2"]

        _, _, c2, _, _, _ = self.trig(q)

        # The inertia couples the two links through the elbow angle (cos θ₂)
        H11 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        H12 = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        H22 = m2 * lc2**2 + I2

        # fmt: off
        H = xp.array([
            [H11, H12],
            [H12, H22],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]

        _, _, _, s2, _, _ = self.trig(q)
        h = m2 * l1 * lc2 * s2

        # Coriolis and centrifugal coupling, driven by the elbow rate
        C11 = -h * dq[1]
        C12 = -h * (dq[0] + dq[1])
        C21 = h * dq[0]
        C22 = 0.0

        # fmt: off
        C = xp.array([
            [C11, C12],
            [C21, C22],
        ])
        # fmt: on

        return C

    def B(self, q, params=None):
        xp = array_module(q)

        dof = self.dof

        # Both joints are actuated: B = I
        B = xp.eye(dof)

        return B

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m1 = params["m1"]
        m2 = params["m2"]
        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        gravity = params["gravity"]

        _, s1, _, _, _, s12 = self.trig(q)
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity

        # Gravity restoring torque on each joint (the sign matches the +y-down screen)
        g = xp.array([-g1 * s1 - g2 * s12, -g2 * s12])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]
        d2 = params["d2"]

        # Linear viscous joint damping, D = diag(d₁, d₂)
        D = xp.diag(xp.array([d1, d2]))
        tau_d = D @ dq

        return tau_d

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.l2
        ground_half_width = self.ground_half_width

        # The joint circles scale with the longer link
        radius = 0.08 * max(l1, l2)

        return {
            "world": [
                CustomLine(
                    [
                        [-ground_half_width, 0.0, 0.0],
                        [ground_half_width, 0.0, 0.0],
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
        params = self.params if params is None else params
        xp = array_module(x)

        dof = self.dof
        l1 = params["l1"]
        l2 = self.l2

        q = x[:dof]
        _, s1, _, _, c12, s12 = self.trig(q)
        c1 = xp.cos(q[0])

        # The elbow at the end of the first link, the tip at the end of the second
        p1 = xp.array([l1 * s1, l1 * c1])
        p2 = p1 + xp.array([l2 * s12, l2 * c12])

        # The rods hang along their local −y, so each frame turns by π − θ; the
        # torque frames point their x axis along the link, at π/2 − θ
        theta1 = xp.pi - q[0]
        theta12 = xp.pi - (q[0] + q[1])
        rod1_angle = xp.pi / 2.0 - q[0]
        rod2_angle = xp.pi / 2.0 - q[0] - q[1]

        return {
            "link1": SE2(0.0, 0.0, theta1),
            "joint1": SE2(p1[0], p1[1], 0.0),
            "link2": SE2(p1[0], p1[1], theta12),
            "joint2": SE2(p2[0], p2[1], 0.0),
            "torque1": SE2(0.0, 0.0, rod1_angle),
            "torque2": SE2(p1[0], p1[1], rod2_angle),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "torque1": [self.joint_torque_arc(u[0], params)],
            "torque2": [self.joint_torque_arc(u[1], params)],
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def joint_torque_arc(self, tau, params=None):
        """The arc of one joint torque: a third of a turn at the input's upper bound."""
        params = self.params if params is None else params

        l1 = params["l1"]
        l2 = self.l2

        torque_radius = 0.2 * max(l1, l2)
        u_lim = float(self.inputs["u"].upper_bound[0])
        torque_scale = 2.0 * np.pi / 3.0 / u_lim

        # The sweep is negated for the q = 0 upright, +y link convention
        arc = TorqueArrow(
            sweep=-tau * torque_scale,
            radius=torque_radius,
            head_ratio=0.4,
            color="red",
            linewidth=2,
        )

        return arc


class Acrobot(DoublePendulum):
    """Double pendulum actuated only at the elbow."""

    def __init__(self):
        super().__init__()

        # Ports: one torque, on the elbow, within ±10 N·m
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

        # The torque acts on the elbow only
        B = xp.array([[0.0], [1.0]])

        return B

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # Only the elbow is actuated: a single torque arc, at joint 2
        return {"torque2": [self.joint_torque_arc(u[0], params)]}


if __name__ == "__main__":
    # sys = DoublePendulum()
    sys = Acrobot()
    sys.x0 = np.array([0.2, -0.15, 0.0, 0.0])
    sys.inputs["u"].nominal_value[0] = 1.0
    # sys.inputs["u"].nominal_value[1] = 1.0
    sys.compute_trajectory(tf=4.0)
    sys.animate(time_factor_video=1.0)
