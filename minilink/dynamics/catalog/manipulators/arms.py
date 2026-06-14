import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    Rod,
    Sphere,
    TorqueArrow,
    arrow_transform,
    point_transform,
    pose2d_matrix,
    rod_between_transform,
    torque_pose2d_matrix,
)


def _planar_joint_positions(q, lengths):
    q = np.asarray(q, dtype=float)
    lengths = np.asarray(lengths, dtype=float)
    angles = np.cumsum(q)
    points = [np.array([0.0, 0.0])]
    for length, angle in zip(lengths, angles):
        step = length * np.array([np.sin(angle), np.cos(angle)])
        points.append(points[-1] + step)
    return np.asarray(points), angles


def _planar_geometry(lengths, *, include_velocity=False, include_torque=False):
    radius = 0.08 * max(float(np.max(lengths)), 1e-12)
    geometry = [
        Rod(
            length=float(length), radius=0.03 * float(length), color="blue", linewidth=2
        )
        for length in lengths
    ]
    geometry.extend(
        Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
        for _ in range(len(lengths) + 1)
    )
    if include_velocity:
        geometry.append(Arrow(color="green", linewidth=2, origin="tail"))
    if include_torque:
        torque_radius = 0.2 * max(float(np.max(lengths)), 1e-12)
        geometry.extend(
            TorqueArrow(
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            )
            for _ in lengths
        )
    return geometry


def _planar_link_transforms(points, angles):
    return [
        pose2d_matrix(point[0], point[1], np.pi - angle)
        for point, angle in zip(points[:-1], angles)
    ]


def _planar_point_transforms(points):
    return [pose2d_matrix(point[0], point[1], 0.0) for point in points]


def _planar_velocity_transform(q, dq, effector, jacobian):
    velocity = jacobian(q) @ dq
    return arrow_transform(
        effector[0],
        effector[1],
        velocity[0],
        velocity[1],
        scale=0.4,
    )


def _planar_torque_transforms(points, angles, u, upper_bound):
    transforms = []
    for i, (point, angle) in enumerate(zip(points[:-1], angles)):
        limit = float(abs(upper_bound[i])) if np.isfinite(upper_bound[i]) else 5.0
        limit = max(limit, 1.0)
        sweep = u[i] * (2.0 * np.pi / 3.0) / limit
        start_angle = np.pi / 2.0 - angle
        transforms.append(torque_pose2d_matrix(point[0], point[1], start_angle, sweep))
    return transforms


class SpeedControlledManipulator(DynamicSystem):
    """Joint-space integrator for velocity-controlled manipulators."""

    def __init__(self, dof, effector_dim):
        self.dof = int(dof)
        self.effector_dim = int(effector_dim)
        super().__init__(
            n=self.dof,
            input_dim=self.dof,
            output_dim=self.dof,
            expose_state=True,
        )
        self.name = f"{self.dof} Joint Speed Controlled Manipulator"
        self.state.labels = [f"q{i + 1}" for i in range(self.dof)]
        self.state.units = ["rad"] * self.dof
        self.inputs["u"].labels = [f"dq{i + 1}" for i in range(self.dof)]
        self.inputs["u"].units = ["rad/s"] * self.dof
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        # velocity-controlled joints: the input is the joint-rate command
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def forward_kinematic_effector(self, q):
        return np.zeros(self.effector_dim)

    def J(self, q):
        return np.zeros((self.effector_dim, self.dof))

    def forward_differential_kinematic_effector(self, q, dq):
        return self.J(q) @ dq

    def _link_lengths(self):
        if hasattr(self, "l"):
            return np.asarray(self.l, dtype=float)
        return np.ones(self.dof)

    def get_kinematic_geometry(self):
        lengths = self._link_lengths()
        return _planar_geometry(lengths, include_velocity=True)

    def get_kinematic_transforms(self, x, u, t):
        lengths = self._link_lengths()
        points, angles = _planar_joint_positions(x, lengths)
        effector = self.forward_kinematic_effector(x)
        return (
            _planar_link_transforms(points, angles)
            + _planar_point_transforms(points)
            + [_planar_velocity_transform(x, u, effector, self.J)]
        )


class OneLinkManipulator(MechanicalSystem):
    """One-link planar manipulator."""

    def __init__(self):
        super().__init__(dof=1, actuators=1)
        self.name = "One Link Manipulator"
        self.params = {
            "l1": 2.5,
            "lc1": 1.2,
            "m1": 1.0,
            "I1": 0.0,
            "gravity": 9.81,
            "d1": 0.1,
        }
        self.state.labels = ["theta", "dtheta"]
        self.state.units = ["rad", "rad/s"]
        self.inputs["u"].labels = ["tau"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def H(self, q, params=None):
        params = self.params if params is None else params
        m1 = params["m1"]
        lc1 = params["lc1"]
        I1 = params["I1"]

        # rotational inertia of the single link about the pivot
        return np.array([[m1 * lc1**2 + I1]])

    def C(self, q, dq, params=None):
        return np.zeros((1, 1))

    def g(self, q, params=None):
        params = self.params if params is None else params
        m1 = params["m1"]
        lc1 = params["lc1"]
        gravity = params["gravity"]

        # gravity torque about the pivot
        return np.array([-m1 * gravity * lc1 * np.sin(q[0])])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1 = params["d1"]

        # linear viscous joint damping
        return np.array([d1 * dq[0]])

    def forward_kinematic_effector(self, q):
        l1 = self.params["l1"]
        return np.array([l1 * np.sin(q[0]), l1 * np.cos(q[0])])

    def J(self, q):
        l1 = self.params["l1"]
        # fmt: off
        return np.array([
            [ l1 * np.cos(q[0])],
            [-l1 * np.sin(q[0])],
        ])
        # fmt: on

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        return _planar_geometry(np.array([l1]), include_torque=True)

    def get_kinematic_transforms(self, x, u, t):
        l1 = self.params["l1"]
        q, _ = self.x2q(x)
        points, angles = _planar_joint_positions(q, np.array([l1]))
        return (
            _planar_link_transforms(points, angles)
            + _planar_point_transforms(points)
            + _planar_torque_transforms(points, angles, u, self.inputs["u"].upper_bound)
        )


class TwoLinkManipulator(MechanicalSystem):
    """Two-link planar manipulator."""

    def __init__(self):
        super().__init__(dof=2, actuators=2)
        self.name = "Two Link Manipulator"
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
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def _trig(self, q):
        return (
            np.cos(q[0]),
            np.sin(q[0]),
            np.cos(q[1]),
            np.sin(q[1]),
            np.cos(q[0] + q[1]),
            np.sin(q[0] + q[1]),
        )

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

        # coupled inertia of the two planar links
        h11 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        h12 = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        h22 = m2 * lc2**2 + I2
        # fmt: off
        return np.array([
            [h11, h12],
            [h12, h22],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        _, _, _, s2, _, _ = self._trig(q)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]

        # Coriolis/centrifugal coupling driven by the second joint rate
        h = m2 * l1 * lc2 * s2
        # fmt: off
        return np.array([
            [-h * dq[1], -h * (dq[0] + dq[1])],
            [ h * dq[0],                  0.0],
        ])
        # fmt: on

    def g(self, q, params=None):
        params = self.params if params is None else params
        _, s1, _, _, _, s12 = self._trig(q)

        m1 = params["m1"]
        m2 = params["m2"]
        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        gravity = params["gravity"]

        # gravity torque on each joint
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity
        return np.array([-g1 * s1 - g2 * s12, -g2 * s12])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1 = params["d1"]
        d2 = params["d2"]

        # linear viscous joint damping
        return np.diag([d1, d2]) @ dq

    def forward_kinematic_effector(self, q):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        _, s1, _, _, c12, s12 = self._trig(q)
        c1 = np.cos(q[0])
        return np.array([l1 * s1 + l2 * s12, l1 * c1 + l2 * c12])

    def J(self, q):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        c1, s1, _, _, c12, s12 = self._trig(q)
        # fmt: off
        return np.array([
            [ l1 * c1 + l2 * c12,  l2 * c12],
            [-l1 * s1 - l2 * s12, -l2 * s12],
        ])
        # fmt: on

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        return _planar_geometry(np.array([l1, l2]), include_torque=True)

    def get_kinematic_transforms(self, x, u, t):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        q, _ = self.x2q(x)
        points, angles = _planar_joint_positions(q, np.array([l1, l2]))
        return (
            _planar_link_transforms(points, angles)
            + _planar_point_transforms(points)
            + _planar_torque_transforms(points, angles, u, self.inputs["u"].upper_bound)
        )


class ThreeLinkManipulator3D(MechanicalSystem):
    """Three-link spatial manipulator from the Pyro catalog."""

    def __init__(self):
        super().__init__(dof=3, actuators=3)
        self.name = "Three Link Manipulator 3D"
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

    def _trig(self, q):
        c1, s1 = np.cos(q[0]), np.sin(q[0])
        c2, s2 = np.cos(q[1]), np.sin(q[1])
        c3, s3 = np.cos(q[2]), np.sin(q[2])
        c23, s23 = np.cos(q[1] + q[2]), np.sin(q[1] + q[2])
        return c1, s1, c2, s2, c3, s3, c23, s23

    def H(self, q, params=None):
        params = self.params if params is None else params
        _, _, c2, s2, c3, _, c23, s23 = self._trig(q)

        m2 = params["m2"]
        m3 = params["m3"]
        I1z = params["I1z"]
        I2x = params["I2x"]
        I2y = params["I2y"]
        I2z = params["I2z"]
        I3x = params["I3x"]
        I3y = params["I3y"]
        I3z = params["I3z"]
        # book notation (Murray, Li & Sastry, Example 4.3): l1 = link-2 length,
        # r1 / r2 = link-2 / link-3 center-of-mass distances
        l1 = params["l2"]
        r1 = params["lc2"]
        r2 = params["lc3"]

        # spatial inertia: base yaw decoupled from the shoulder/elbow block
        h11 = (
            I2y * s2**2
            + I3y * s23**2
            + I1z
            + I2z * c2**2
            + I3z * c23**2
            + m2 * (r1 * c2) ** 2
            + m3 * (l1 * c2 + r2 * c23) ** 2
        )
        h22 = I2x + I3x + m3 * l1**2 + m2 * r1**2 + m3 * r2**2 + 2.0 * m3 * l1 * r2 * c3
        h23 = I3x + m3 * r2**2 + m3 * l1 * r2 * c3
        h33 = I3x + m3 * r2**2
        # fmt: off
        return np.array([
            [h11, 0.0, 0.0],
            [0.0, h22, h23],
            [0.0, h23, h33],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        _, _, c2, s2, c3, s3, c23, s23 = self._trig(q)

        m2 = params["m2"]
        m3 = params["m3"]
        I2y = params["I2y"]
        I2z = params["I2z"]
        I3y = params["I3y"]
        I3z = params["I3z"]
        l1 = params["l2"]
        r1 = params["lc2"]
        r2 = params["lc3"]

        # Christoffel-symbol coupling terms
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
        # fmt: off
        return np.array([
            [T112 * dq[1] + T113 * dq[2], T112 * dq[0],           T113 * dq[0]],
            [               T211 * dq[0], T223 * dq[2], T223 * (dq[1] + dq[2])],
            [               T311 * dq[0], T322 * dq[1],                    0.0],
        ])
        # fmt: on

    def g(self, q, params=None):
        params = self.params if params is None else params
        _, _, c2, _, _, _, c23, _ = self._trig(q)

        m2 = params["m2"]
        m3 = params["m3"]
        l2 = params["l2"]
        lc2 = params["lc2"]
        lc3 = params["lc3"]
        gravity = params["gravity"]

        # gravity torque on the shoulder and elbow joints
        g2 = (m2 * gravity * lc2 + m3 * gravity * l2) * c2 + m3 * gravity * lc3 * c23
        g3 = m3 * gravity * lc3 * c23
        return np.array([0.0, -g2, -g3])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1 = params["d1"]
        d2 = params["d2"]
        d3 = params["d3"]

        # linear viscous joint damping
        return np.diag([d1, d2, d3]) @ dq

    def forward_kinematic_effector(self, q):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        l3 = self.params["l3"]
        c1, s1, c2, s2, _, _, c23, s23 = self._trig(q)
        radius = l2 * c2 + l3 * c23
        return np.array([radius * c1, radius * s1, l1 - l2 * s2 - l3 * s23])

    def J(self, q):
        l2 = self.params["l2"]
        l3 = self.params["l3"]
        c1, s1, c2, s2, _, _, c23, s23 = self._trig(q)
        radius = l2 * c2 + l3 * c23
        vertical = l2 * s2 + l3 * s23
        # fmt: off
        return np.array([
            [-radius * s1, -vertical * c1, -l3 * s23 * c1],
            [ radius * c1, -vertical * s1, -l3 * s23 * s1],
            [         0.0,        -radius,      -l3 * c23],
        ])
        # fmt: on

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        l3 = self.params["l3"]
        radius = 0.06 * max(l1, l2, l3)
        return [
            Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2),
            Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2),
            Rod(length=l3, radius=0.03 * l3, color="blue", linewidth=2),
            Sphere(radius=radius, color="blue", opacity=0.9),
            Sphere(radius=radius, color="blue", opacity=0.9),
            Sphere(radius=radius, color="blue", opacity=0.9),
            Sphere(radius=radius, color="blue", opacity=0.9),
        ]

    def get_kinematic_transforms(self, x, u, t):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        l3 = self.params["l3"]
        q, _ = self.x2q(x)
        c1, s1, c2, s2, _, _, c23, s23 = self._trig(q)

        # joint centers along the spatial arm
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, l1])
        p2 = np.array([l2 * c2 * c1, l2 * c2 * s1, l1 - l2 * s2])
        p3 = np.array(
            [
                (l2 * c2 + l3 * c23) * c1,
                (l2 * c2 + l3 * c23) * s1,
                l1 - l2 * s2 - l3 * s23,
            ]
        )
        return [
            rod_between_transform(p0, p1),
            rod_between_transform(p1, p2),
            rod_between_transform(p2, p3),
            point_transform(p0),
            point_transform(p1),
            point_transform(p2),
            point_transform(p3),
        ]


class FiveLinkPlanarManipulator(MechanicalSystem):
    """Five-link planar manipulator with Pyro's default unit joint dynamics."""

    def __init__(self):
        super().__init__(dof=5, actuators=5)
        self.name = "Five Link Planar Manipulator"
        self.effector_dim = 2
        self.params = {"l": np.array([0.5, 0.5, 0.5, 0.5, 0.5])}
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

    def _absolute_trig(self, q):
        angles = np.cumsum(q)
        return np.cos(angles), np.sin(angles)

    def forward_kinematic_effector(self, q):
        l = self.params["l"]
        cos_abs, sin_abs = self._absolute_trig(q)
        return np.array([np.sum(l * sin_abs), np.sum(l * cos_abs)])

    def J(self, q):
        l = self.params["l"]
        cos_abs, sin_abs = self._absolute_trig(q)
        J = np.zeros((2, self.dof))
        for joint in range(self.dof):
            J[0, joint] = np.sum(l[joint:] * cos_abs[joint:])
            J[1, joint] = -np.sum(l[joint:] * sin_abs[joint:])
        return J

    def get_kinematic_geometry(self):
        return _planar_geometry(self.params["l"], include_torque=True)

    def get_kinematic_transforms(self, x, u, t):
        l = self.params["l"]
        q, _ = self.x2q(x)
        points, angles = _planar_joint_positions(q, l)
        return (
            _planar_link_transforms(points, angles)
            + _planar_point_transforms(points)
            + _planar_torque_transforms(points, angles, u, self.inputs["u"].upper_bound)
        )


if __name__ == "__main__":
    sys = TwoLinkManipulator()
    sys.x0 = np.array([0.2, -0.3, 0.0, 0.0])
    sys.compute_forced(
        lambda t: np.array([0.1 * np.sin(t), 0.1 * np.cos(t)]),
        tf=4.0,
        n_steps=160,
    )
    sys.animate()
