"""Planar and spatial serial arms: one, two, three and five links, and a velocity-controlled base."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    Rod,
    Sphere,
    TorqueArrow,
)
from minilink.graphical.catalog.shapes import link_pose_3d, point_pose

###############################################################################
# Public API
###############################################################################


class OneLinkManipulator(Manipulator):
    """One-link planar manipulator."""

    def __init__(self):
        super().__init__(dof=1, actuators=1, task_dim=2)
        self.name = "One Link Manipulator"

        # Parameters
        self.params = {
            "l1": 2.5,
            "lc1": 1.2,
            "m1": 1.0,
            "I1": 0.0,
            "gravity": 9.81,
            "d1": 0.1,
        }

        # Signal labels and units
        self.state.labels = ["theta", "dtheta"]
        self.state.units = ["rad", "rad/s"]
        self.inputs["u"].labels = ["tau"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the arm's full reach
        reach = self.params["l1"]
        self.camera_scale, self.camera_target = planar_reach_camera(reach)

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m1 = params["m1"]
        lc1 = params["lc1"]
        I1 = params["I1"]

        # Inertia of the link about its pivot
        H11 = m1 * lc1**2 + I1

        H = xp.array([[H11]])

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # A single joint has no Coriolis or centrifugal coupling: C = 0
        C = xp.zeros((1, 1))

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m1 = params["m1"]
        lc1 = params["lc1"]
        gravity = params["gravity"]

        theta = q[0]

        # Gravity torque about the pivot
        g = xp.array([-m1 * gravity * lc1 * xp.sin(theta)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]

        # Linear viscous joint damping
        tau_d = xp.array([d1 * dq[0]])

        return tau_d

    def forward_kinematics(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]

        theta = q[0]

        # The effector, with q = 0 pointing up the y axis
        p = xp.array([l1 * xp.sin(theta), l1 * xp.cos(theta)])

        return p

    def J(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]

        theta = q[0]

        J11 = l1 * xp.cos(theta)
        J21 = -l1 * xp.sin(theta)

        # fmt: off
        J = xp.array([
            [J11],
            [J21],
        ])
        # fmt: on

        return J

    def get_kinematic_geometry(self):
        lengths = np.array([self.params["l1"]])

        return planar_kinematic_geometry(lengths)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        l1 = params["l1"]

        q, _ = self.x2q(x)
        lengths = xp.array([l1])

        points, angles = planar_joint_positions(q, lengths)

        return planar_frames(points, angles)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        lengths = np.array([params["l1"]])
        upper_bound = self.inputs["u"].upper_bound

        return planar_torque_geometry(lengths, u, upper_bound)


class TwoLinkManipulator(Manipulator):
    """Two-link planar manipulator."""

    def __init__(self):
        super().__init__(dof=2, actuators=2, task_dim=2)
        self.name = "Two Link Manipulator"

        # Parameters
        self.params = {
            "l1": 0.5,
            "l2": 0.3,
            "lc1": 0.2,
            "lc2": 0.1,
            "m1": 1.0,
            "m2": 1.0,
            "I1": 0.0,
            "I2": 0.0,
            "gravity": 9.81,
            "d1": 0.5,
            "d2": 0.5,
        }

        # Signal labels and units
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the arm's full reach
        reach = self.params["l1"] + self.params["l2"]
        self.camera_scale, self.camera_target = planar_reach_camera(reach)

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

        # Coupled inertia of the two planar links
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
        xp = array_module(q, dq)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]

        _, _, _, s2, _, _ = self.trig(q)
        h = m2 * l1 * lc2 * s2

        # Coriolis and centrifugal coupling, driven by the second joint's rate
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

        # Gravity torque on each joint
        g = xp.array([-g1 * s1 - g2 * s12, -g2 * s12])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]
        d2 = params["d2"]

        # Linear viscous joint damping
        tau_d = xp.array([d1 * dq[0], d2 * dq[1]])

        return tau_d

    def forward_kinematics(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]
        l2 = params["l2"]

        c1, s1, _, _, c12, s12 = self.trig(q)

        # The effector, with q = 0 pointing up the y axis
        p = xp.array([l1 * s1 + l2 * s12, l1 * c1 + l2 * c12])

        return p

    def J(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]
        l2 = params["l2"]

        c1, s1, _, _, c12, s12 = self.trig(q)

        J11 = l1 * c1 + l2 * c12
        J12 = l2 * c12
        J21 = -l1 * s1 - l2 * s12
        J22 = -l2 * s12

        # fmt: off
        J = xp.array([
            [J11, J12],
            [J21, J22],
        ])
        # fmt: on

        return J

    def lengths(self, params=None):
        """The link lengths ``[l1, l2]``."""
        params = self.params if params is None else params

        lengths = np.array([params["l1"], params["l2"]])

        return lengths

    def get_kinematic_geometry(self):
        lengths = self.lengths()

        return planar_kinematic_geometry(lengths)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        lengths = self.lengths(params)

        q, _ = self.x2q(x)

        points, angles = planar_joint_positions(q, lengths)

        return planar_frames(points, angles)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        lengths = self.lengths(params)
        upper_bound = self.inputs["u"].upper_bound

        return planar_torque_geometry(lengths, u, upper_bound)


class ThreeLinkManipulator3D(Manipulator):
    """Three-link spatial manipulator from the Pyro catalog."""

    def __init__(self):
        super().__init__(dof=3, actuators=3, task_dim=3)
        self.name = "Three Link Manipulator 3D"

        # Parameters
        self.params = {
            "l1": 1.0,
            "l2": 1.0,
            "l3": 1.0,
            "lc1": 1.0,
            "lc2": 1.0,
            "lc3": 1.0,
            "m1": 1.0,
            "m2": 1.0,
            "m3": 1.0,
            "I1z": 1.0,
            "I2x": 1.0,
            "I2y": 1.0,
            "I2z": 1.0,
            "I3x": 1.0,
            "I3y": 1.0,
            "I3z": 1.0,
            "gravity": 9.81,
            "d1": 1.0,
            "d2": 1.0,
            "d3": 1.0,
        }

        # Signal labels and units
        self.state.labels = [
            "theta1",
            "theta2",
            "theta3",
            "dtheta1",
            "dtheta2",
            "dtheta3",
        ]
        self.state.units = ["rad", "rad", "rad", "rad/s", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2", "tau3"]
        self.inputs["u"].units = ["Nm", "Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the arm's full reach
        reach = self.params["l1"] + self.params["l2"] + self.params["l3"]
        self.camera_scale = max(1.5 * reach, 0.5)
        self.camera_target = np.array([0.0, 0.0, -0.5 * reach])

    def trig(self, q):
        """Shorthands ``c1, s1, c2, s2, c3, s3, c23, s23`` of the joint angles."""
        xp = array_module(q)

        c1 = xp.cos(q[0])
        s1 = xp.sin(q[0])
        c2 = xp.cos(q[1])
        s2 = xp.sin(q[1])
        c3 = xp.cos(q[2])
        s3 = xp.sin(q[2])
        c23 = xp.cos(q[1] + q[2])
        s23 = xp.sin(q[1] + q[2])

        return c1, s1, c2, s2, c3, s3, c23, s23

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        m3 = params["m3"]
        I1z = params["I1z"]
        I2x = params["I2x"]
        I2y = params["I2y"]
        I2z = params["I2z"]
        I3x = params["I3x"]
        I3y = params["I3y"]
        I3z = params["I3z"]
        # Book notation (Murray, Li & Sastry, Example 4.3): l1 is the second link's
        # length, r1 and r2 the centre-of-mass distances of the second and third links
        l1 = params["l2"]
        r1 = params["lc2"]
        r2 = params["lc3"]

        _, _, c2, s2, c3, _, c23, s23 = self.trig(q)

        # Spatial inertia: the base yaw decoupled from the shoulder and elbow block
        H11 = (
            I2y * s2**2
            + I3y * s23**2
            + I1z
            + I2z * c2**2
            + I3z * c23**2
            + m2 * (r1 * c2) ** 2
            + m3 * (l1 * c2 + r2 * c23) ** 2
        )
        H22 = I2x + I3x + m3 * l1**2 + m2 * r1**2 + m3 * r2**2 + 2.0 * m3 * l1 * r2 * c3
        H23 = I3x + m3 * r2**2 + m3 * l1 * r2 * c3
        H33 = I3x + m3 * r2**2

        # fmt: off
        H = xp.array([
            [H11, 0.0, 0.0],
            [0.0, H22, H23],
            [0.0, H23, H33],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        xp = array_module(q, dq)

        m2 = params["m2"]
        m3 = params["m3"]
        I2y = params["I2y"]
        I2z = params["I2z"]
        I3y = params["I3y"]
        I3z = params["I3z"]
        l1 = params["l2"]
        r1 = params["lc2"]
        r2 = params["lc3"]

        _, _, c2, s2, c3, s3, c23, s23 = self.trig(q)

        # Christoffel-symbol coefficients
        T112 = (
            (I2y - I2z - m2 * r1**2) * c2 * s2
            + (I3y - I3z) * c23 * s23
            - m3 * (l1 * c2 + r2 * c23) * (l1 * s2 + r2 * s23)
        )
        T113 = (I3y - I3z) * c23 * s23 - m3 * r2 * s23 * (l1 * c2 + r2 * c23)
        T211 = -T112
        T223 = -l1 * m3 * r2 * s3
        T311 = -T113
        T322 = l1 * m3 * r2 * s3

        # Coriolis and centrifugal coupling, linear in the joint rates
        C11 = T112 * dq[1] + T113 * dq[2]
        C12 = T112 * dq[0]
        C13 = T113 * dq[0]
        C21 = T211 * dq[0]
        C22 = T223 * dq[2]
        C23 = T223 * (dq[1] + dq[2])
        C31 = T311 * dq[0]
        C32 = T322 * dq[1]
        C33 = 0.0

        # fmt: off
        C = xp.array([
            [C11, C12, C13],
            [C21, C22, C23],
            [C31, C32, C33],
        ])
        # fmt: on

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        m3 = params["m3"]
        l2 = params["l2"]
        lc2 = params["lc2"]
        lc3 = params["lc3"]
        gravity = params["gravity"]

        _, _, c2, _, _, _, c23, _ = self.trig(q)

        # Gravity torque on the shoulder and elbow joints; the base yaw carries none
        g2 = (m2 * gravity * lc2 + m3 * gravity * l2) * c2 + m3 * gravity * lc3 * c23
        g3 = m3 * gravity * lc3 * c23
        g = xp.array([0.0, -g2, -g3])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]
        d2 = params["d2"]
        d3 = params["d3"]

        # Linear viscous joint damping
        tau_d = xp.array([d1 * dq[0], d2 * dq[1], d3 * dq[2]])

        return tau_d

    def forward_kinematics(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]
        l2 = params["l2"]
        l3 = params["l3"]

        c1, s1, c2, s2, _, _, c23, s23 = self.trig(q)
        radius = l2 * c2 + l3 * c23

        # The effector: its horizontal reach turned by the base yaw, its height
        p = xp.array([radius * c1, radius * s1, l1 - l2 * s2 - l3 * s23])

        return p

    def J(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l2 = params["l2"]
        l3 = params["l3"]

        c1, s1, c2, s2, _, _, c23, s23 = self.trig(q)
        radius = l2 * c2 + l3 * c23
        vertical = l2 * s2 + l3 * s23

        J11 = -radius * s1
        J12 = -vertical * c1
        J13 = -l3 * s23 * c1
        J21 = radius * c1
        J22 = -vertical * s1
        J23 = -l3 * s23 * s1
        J31 = 0.0
        J32 = -radius
        J33 = -l3 * c23

        # fmt: off
        J = xp.array([
            [J11, J12, J13],
            [J21, J22, J23],
            [J31, J32, J33],
        ])
        # fmt: on

        return J

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        l3 = self.params["l3"]

        radius = 0.06 * max(l1, l2, l3)

        return {
            "link0": [Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2)],
            "link1": [Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2)],
            "link2": [Rod(length=l3, radius=0.03 * l3, color="blue", linewidth=2)],
            "joint0": [Sphere(radius=radius, color="blue", opacity=0.9)],
            "joint1": [Sphere(radius=radius, color="blue", opacity=0.9)],
            "joint2": [Sphere(radius=radius, color="blue", opacity=0.9)],
            "joint3": [Sphere(radius=radius, color="blue", opacity=0.9)],
        }

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        l1 = params["l1"]
        l2 = params["l2"]
        l3 = params["l3"]

        q, _ = self.x2q(x)
        c1, s1, c2, s2, _, _, c23, s23 = self.trig(q)

        # The base, shoulder, elbow and effector points along the chain
        p0 = xp.array([0.0, 0.0, 0.0])
        p1 = xp.array([0.0, 0.0, l1])
        p2 = xp.array([l2 * c2 * c1, l2 * c2 * s1, l1 - l2 * s2])
        p3 = xp.array(
            [
                (l2 * c2 + l3 * c23) * c1,
                (l2 * c2 + l3 * c23) * s1,
                l1 - l2 * s2 - l3 * s23,
            ]
        )

        return {
            "link0": link_pose_3d(p0, p1),
            "link1": link_pose_3d(p1, p2),
            "link2": link_pose_3d(p2, p3),
            "joint0": point_pose(p0),
            "joint1": point_pose(p1),
            "joint2": point_pose(p2),
            "joint3": point_pose(p3),
        }


class FiveLinkPlanarManipulator(Manipulator):
    """Five-link planar manipulator with Pyro's default unit joint dynamics.

    Kinematics and Jacobian are fully specified; dynamics use the
    :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem`
    placeholder ``H = I`` (no gravity/Coriolis). Prefer one- to three-link
    arms for computed-torque demos.
    """

    def __init__(self):
        super().__init__(dof=5, actuators=5, task_dim=2)
        self.name = "Five Link Planar Manipulator"

        # Parameters
        self.params = {"l": np.array([0.5, 0.5, 0.5, 0.5, 0.5])}

        # Signal labels and units
        self.state.labels = [
            "theta1",
            "theta2",
            "theta3",
            "theta4",
            "theta5",
            "dtheta1",
            "dtheta2",
            "dtheta3",
            "dtheta4",
            "dtheta5",
        ]
        self.state.units = ["rad"] * self.dof + ["rad/s"] * self.dof
        self.inputs["u"].labels = [f"tau{i + 1}" for i in range(self.dof)]
        self.inputs["u"].units = ["Nm"] * self.dof
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the arm's full reach
        reach = float(np.sum(self.params["l"]))
        self.camera_scale, self.camera_target = planar_reach_camera(reach)

    def absolute_trig(self, q):
        """Cosine and sine of each link's absolute angle (the joint angles summed)."""
        xp = array_module(q)

        angles = xp.cumsum(q)
        cos_abs = xp.cos(angles)
        sin_abs = xp.sin(angles)

        return cos_abs, sin_abs

    def forward_kinematics(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l = params["l"]

        cos_abs, sin_abs = self.absolute_trig(q)

        # The effector: the links summed in their absolute angles
        p = xp.array([xp.sum(l * sin_abs), xp.sum(l * cos_abs)])

        return p

    def J(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l = params["l"]
        dof = self.dof

        cos_abs, sin_abs = self.absolute_trig(q)

        # Column j: the effector moves with every link from joint j outward
        J = xp.array(
            [
                [xp.sum(l[j:] * cos_abs[j:]) for j in range(dof)],
                [-xp.sum(l[j:] * sin_abs[j:]) for j in range(dof)],
            ]
        )

        return J

    def get_kinematic_geometry(self):
        lengths = self.params["l"]

        return planar_kinematic_geometry(lengths)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        lengths = params["l"]

        q, _ = self.x2q(x)

        points, angles = planar_joint_positions(q, lengths)

        return planar_frames(points, angles)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        lengths = params["l"]
        upper_bound = self.inputs["u"].upper_bound

        return planar_torque_geometry(lengths, u, upper_bound)


class SpeedControlledManipulator(DynamicSystem):
    """Joint-space integrator for velocity-controlled manipulators.

    Dynamics: ``dq = u``. State and default output ``y`` are joint angles ``q``.
    Use :meth:`from_manipulator` to inherit ``forward_kinematics`` and ``J`` from
    a torque-controlled :class:`Manipulator`.
    """

    def __init__(self, dof, effector_dim):
        # The dimensions; the arm lending its kinematics comes from from_manipulator
        self.dof = int(dof)
        self.effector_dim = int(effector_dim)
        self._kin_arm = None
        super().__init__(
            n=self.dof,
            input_dim=self.dof,
            output_dim=self.dof,
            expose_state=True,
        )
        self.name = f"{self.dof} Joint Speed Controlled Manipulator"

        # Signal labels and units: the joint angles, driven by the joint rates
        self.state.labels = [f"q{i + 1}" for i in range(self.dof)]
        self.state.units = ["rad"] * self.dof
        self.inputs["u"].labels = [f"dq{i + 1}" for i in range(self.dof)]
        self.inputs["u"].units = ["rad/s"] * self.dof
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Ports: the effector position
        p_labels = [f"p{i}" for i in range(self.effector_dim)]
        self.add_output_port(
            "p",
            dim=self.effector_dim,
            function=self.h_p,
            dependencies=(),
            labels=p_labels,
            units=["[m]"] * self.effector_dim,
        )

        # Graphics: the camera frames the arm's full reach
        reach = float(np.sum(self.link_lengths()))
        self.camera_scale, self.camera_target = planar_reach_camera(reach)

    @classmethod
    def from_manipulator(cls, arm: Manipulator):
        """Build a velocity-controlled plant with kinematics from *arm*."""
        require_manipulator(arm)

        # The plant, named after the arm whose kinematics it reads
        inst = cls(arm.dof, arm.task_dim)
        inst._kin_arm = arm
        inst.name = f"{arm.dof} Joint Speed Controlled {arm.name}"

        # Graphics: the camera frames the arm's full reach
        reach = float(np.sum(inst.link_lengths()))
        inst.camera_scale, inst.camera_target = planar_reach_camera(reach)

        return inst

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(u)

        # q̇ = u: the input is the joint-rate command
        dx = xp.asarray(u)

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def h_p(self, x, u, t=0, params=None):
        return self.forward_kinematics(x, params)

    def forward_kinematics(self, q, params=None):
        if self._kin_arm is not None:
            return self._kin_arm.forward_kinematics(q, params)

        xp = array_module(q)

        effector_dim = self.effector_dim

        # No arm: the effector stays at the origin
        p = xp.zeros(effector_dim)

        return p

    def J(self, q, params=None):
        if self._kin_arm is not None:
            return self._kin_arm.J(q, params)

        xp = array_module(q)

        effector_dim = self.effector_dim
        dof = self.dof

        # No arm: the joints do not move the effector
        J = xp.zeros((effector_dim, dof))

        return J

    def forward_differential_kinematic_effector(self, q, dq):
        J = self.J(q)

        # ṗ = J(q) q̇
        pdot = J @ dq

        return pdot

    def link_lengths(self, params=None):
        """The drawn link lengths: the arm's, read from ``params`` when they are given."""
        if self._kin_arm is not None:
            arm = self._kin_arm
            arm_params = arm.params if params is None else params
            if hasattr(arm, "lengths"):
                return arm.lengths(arm_params)
            if "l" in arm_params:
                return np.asarray(arm_params["l"], dtype=float)
        if hasattr(self, "l"):
            return np.asarray(self.l, dtype=float)
        return np.ones(self.dof)

    def get_kinematic_geometry(self):
        lengths = self.link_lengths()

        return planar_kinematic_geometry(lengths)

    def tf(self, x, u, t=0, params=None):
        lengths = self.link_lengths(params)

        # The joints along the chain, and the effector's frame at its tip
        points, angles = planar_joint_positions(x, lengths)
        effector = self.forward_kinematics(x, params)

        return planar_frames(points, angles, effector)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        J = self.J(x, params)

        # ṗ = J(q) u, drawn at the effector
        velocity = J @ u

        return planar_velocity_geometry(velocity)


###############################################################################
# Internal machinery
###############################################################################


def planar_reach_camera(reach):
    """The orthographic zoom and target that frame a planar arm of reach ``reach``."""
    reach = float(reach)
    camera_scale = max(1.5 * reach, 0.35)
    camera_target = np.array([0.0, 0.5 * reach, 0.0])

    return camera_scale, camera_target


def planar_kinematic_geometry(lengths):
    """Static planar arm geometry: one ``link{i}`` rod with joint circles at each end."""
    radius = 0.08 * max(float(np.max(lengths)), 1e-12)

    geometry = {}
    for i, length in enumerate(lengths):
        joint_base = Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
        joint_tip = Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
        joint_tip.local_transform = translation(0.0, -float(length), 0.0)
        geometry[f"link{i}"] = [
            joint_base,
            Rod(
                length=float(length),
                radius=0.03 * float(length),
                color="blue",
                linewidth=2,
            ),
            joint_tip,
        ]

    return geometry


def planar_joint_positions(q, lengths):
    """Joint points of a planar chain from its base, and each link's absolute angle."""
    xp = array_module(q)

    # Each link steps from the last joint along its absolute angle, q = 0 up the y axis
    angles = xp.cumsum(q)
    points = [xp.array([0.0, 0.0])]
    for length, angle in zip(lengths, angles):
        step = length * xp.array([xp.sin(angle), xp.cos(angle)])
        points.append(points[-1] + step)

    return xp.asarray(points), angles


def planar_frames(points, angles, effector=None):
    """World transforms for the planar arm links (and the effector frame)."""
    frames = {}
    for i, (point, angle) in enumerate(zip(points[:-1], angles)):
        frames[f"link{i}"] = SE2(point[0], point[1], np.pi - angle)
    if effector is not None:
        frames["velocity"] = translation(effector[0], effector[1], 0.0)

    return frames


def planar_torque_geometry(lengths, u, upper_bound):
    """Per-frame joint torque arcs (honest, keyed to the ``link{i}`` frames).

    Sweep sign matches :class:`~minilink.dynamics.catalog.pendulum.double_pendulum.DoublePendulum`
    (negated ``u``) for the shared ``q=0`` upright / ``+y`` link convention.
    """
    torque_radius = 0.2 * max(float(np.max(lengths)), 1e-12)

    dynamic = {}
    for i in range(len(lengths)):
        # A third of a turn at the torque limit (5 when unbounded, never below 1)
        limit = float(abs(upper_bound[i])) if np.isfinite(upper_bound[i]) else 5.0
        limit = max(limit, 1.0)
        sweep = -u[i] * (2.0 * np.pi / 3.0) / limit
        arc = TorqueArrow(
            sweep=sweep,
            radius=torque_radius,
            head_ratio=0.4,
            color="red",
            linewidth=2,
        )
        arc.local_transform = SE2(0.0, 0.0, -np.pi / 2.0)
        dynamic[f"link{i}"] = [arc]

    return dynamic


def require_manipulator(arm):
    """Refuse a plant that is not a torque-controlled :class:`Manipulator`."""
    if not isinstance(arm, Manipulator):
        raise TypeError("from_manipulator requires a Manipulator instance")


def planar_velocity_geometry(velocity):
    """Per-frame effector velocity arrow (honest, keyed to the ``velocity`` frame)."""
    return {
        "velocity": [
            Arrow(
                base=(0.0, 0.0),
                vector=(velocity[0], velocity[1]),
                scale=0.4,
                color="green",
                linewidth=2,
            )
        ]
    }


if __name__ == "__main__":
    sys = TwoLinkManipulator()
    sys.x0 = np.array([0.2, -0.3, 0.0, 0.0])
    sys.compute_forced(
        lambda t: np.array([0.1 * np.sin(t), 0.1 * np.cos(t)]),
        tf=4.0,
        n_steps=160,
    )
    sys.animate()
