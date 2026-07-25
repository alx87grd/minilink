"""UR5 six-axis manipulator with numeric rigid-body dynamics.

The kinematic dimensions, masses, and centers of mass follow the public
``ros-industrial/universal_robot`` UR5 description. Link inertias are cylinder
approximations, so this model is intended for controls teaching and simulation,
not calibrated prediction of a particular robot.

Dynamics are evaluated from link inertial wrenches instead of expanded symbolic
expressions:

    H(q) qdd + C(q, qd) qd + d(q, qd) + g(q) = tau
"""

import numpy as np

from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.graphical.catalog.shapes import link_pose_3d
from minilink.graphical.catalog.skins import ur5_skin


def _dh_transform(theta, d, a, alpha):
    """Standard DH transform from one link frame to the next."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    # fmt: off
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,       ca,      d],
        [0.0,      0.0,      0.0,    1.0],
    ])
    # fmt: on


def _cylinder_inertia(mass, length, radius, axis):
    """Diagonal COM inertia for a solid cylinder aligned with *axis*."""
    axial = 0.5 * mass * radius**2
    transverse = mass * (3.0 * radius**2 + length**2) / 12.0
    diagonal = np.full(3, transverse)
    diagonal[axis] = axial
    return np.diag(diagonal)


def _skew(v):
    # fmt: off
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
    # fmt: on


def _motion_cross(v):
    cross = np.zeros((6, 6))
    cross[:3, :3] = _skew(v[:3])
    cross[3:, :3] = _skew(v[3:])
    cross[3:, 3:] = _skew(v[:3])
    return cross


def _spatial_inertia(mass, com, inertia):
    c = _skew(com)
    spatial = np.zeros((6, 6))
    spatial[:3, :3] = inertia + mass * c.T @ c
    spatial[:3, 3:] = mass * c
    spatial[3:, :3] = mass * c.T
    spatial[3:, 3:] = mass * np.eye(3)
    return spatial


class UR5Manipulator(Manipulator):
    """Six-degree-of-freedom UR5 arm with a schematic 3-D skin.

    Kinematics use the standard Denavit-Hartenberg convention. The published
    UR5 masses and centers of mass are paired with simple cylinder inertias;
    consequently the model is suitable for controller examples but is not a
    factory-calibrated representation.
    """

    def __init__(self):
        super().__init__(dof=6, actuators=6, task_dim=3)
        self.name = "UR5 Manipulator"

        masses = np.array([3.7, 8.393, 2.275, 1.219, 1.219, 0.1879])
        self.params = {
            "a": np.array([0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]),
            "d": np.array([0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]),
            "alpha": np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]),
            "mass": masses,
            "com": np.array(
                [
                    [0.0, -0.02561, 0.00193],
                    [0.2125, 0.0, 0.11336],
                    [0.11993, 0.0, 0.0265],
                    [0.0, -0.0018, 0.01634],
                    [0.0, 0.0018, 0.01634],
                    [0.0, 0.0, -0.001159],
                ]
            ),
            "inertia": np.array(
                [
                    _cylinder_inertia(masses[0], 0.15, 0.06, axis=2),
                    _cylinder_inertia(masses[1], 0.56, 0.06, axis=0),
                    _cylinder_inertia(masses[2], 0.39225, 0.06, axis=0),
                    _cylinder_inertia(masses[3], 0.12, 0.06, axis=2),
                    _cylinder_inertia(masses[4], 0.12, 0.06, axis=2),
                    _cylinder_inertia(masses[5], 0.0345, 0.0375, axis=2),
                ]
            ),
            "gravity": 9.81,
            "damping": np.full(6, 0.05),
        }

        names = ["base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3"]
        self.state.labels = [f"{name} angle" for name in names] + [
            f"{name} velocity" for name in names
        ]
        self.state.units = ["rad"] * 6 + ["rad/s"] * 6
        self.inputs["u"].labels = [f"{name} torque" for name in names]
        self.inputs["u"].units = ["Nm"] * 6
        self.inputs["u"].lower_bound = -np.array([150, 150, 150, 28, 28, 28])
        self.inputs["u"].upper_bound = np.array([150, 150, 150, 28, 28, 28])
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        self.skin = ur5_skin
        self.camera_scale = 1.3
        self.camera_target = np.array([-0.3, 0.0, 0.25])

    def _chain(self, q, params=None):
        params = self.params if params is None else params
        a = np.asarray(params["a"], dtype=float)
        d = np.asarray(params["d"], dtype=float)
        alpha = np.asarray(params["alpha"], dtype=float)

        T = np.eye(4)
        joint_frames = []
        joint_origins = []
        joint_axes = []
        link_frames = []
        for i in range(self.dof):
            joint_frames.append(T.copy())
            joint_origins.append(T[:3, 3].copy())
            joint_axes.append(T[:3, 2].copy())
            T = T @ _dh_transform(q[i], d[i], a[i], alpha[i])
            link_frames.append(T.copy())
        return (
            np.asarray(joint_frames),
            np.asarray(joint_origins),
            np.asarray(joint_axes),
            np.asarray(link_frames),
        )

    def _rnea(self, q, dq, acceleration, params=None, *, gravity=True):
        """Recursive Newton-Euler inverse dynamics in link coordinates."""
        params = self.params if params is None else params
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)
        acceleration = np.asarray(acceleration, dtype=float)
        a = np.asarray(params["a"], dtype=float)
        d = np.asarray(params["d"], dtype=float)
        alpha = np.asarray(params["alpha"], dtype=float)
        mass = np.asarray(params["mass"], dtype=float)
        com = np.asarray(params["com"], dtype=float)
        inertia = np.asarray(params["inertia"], dtype=float)

        Xup = np.zeros((self.dof, 6, 6))
        motion = np.zeros((self.dof, 6))
        spatial_acceleration = np.zeros((self.dof, 6))
        force = np.zeros((self.dof, 6))
        base_acceleration = np.zeros(6)
        if gravity:
            base_acceleration[5] = float(params["gravity"])

        parent_motion = np.zeros(6)
        parent_acceleration = base_acceleration
        z = np.array([0.0, 0.0, 1.0])
        for i in range(self.dof):
            T = _dh_transform(q[i], d[i], a[i], alpha[i])
            R = T[:3, :3]
            r = T[:3, 3]
            Rt = R.T

            Xup[i, :3, :3] = Rt
            Xup[i, 3:, :3] = -Rt @ _skew(r)
            Xup[i, 3:, 3:] = Rt

            S = np.concatenate([Rt @ z, Rt @ np.cross(z, r)])
            joint_motion = S * dq[i]
            motion[i] = Xup[i] @ parent_motion + joint_motion
            spatial_acceleration[i] = (
                Xup[i] @ parent_acceleration
                + S * acceleration[i]
                + _motion_cross(motion[i]) @ joint_motion
            )

            I = _spatial_inertia(mass[i], com[i], inertia[i])
            momentum = I @ motion[i]
            force[i] = (
                I @ spatial_acceleration[i] - _motion_cross(motion[i]).T @ momentum
            )
            parent_motion = motion[i]
            parent_acceleration = spatial_acceleration[i]

        tau = np.zeros(self.dof)
        for i in reversed(range(self.dof)):
            T = _dh_transform(q[i], d[i], a[i], alpha[i])
            R = T[:3, :3]
            r = T[:3, 3]
            S = np.concatenate([R.T @ z, R.T @ np.cross(z, r)])
            tau[i] = S @ force[i]
            if i > 0:
                force[i - 1] = force[i - 1] + Xup[i].T @ force[i]
        return tau

    def H(self, q, params=None):
        """Joint-space inertia matrix."""
        params = self.params if params is None else params
        zero = np.zeros(self.dof)
        H = np.column_stack(
            [
                self._rnea(q, zero, direction, params, gravity=False)
                for direction in np.eye(self.dof)
            ]
        )
        return 0.5 * (H + H.T)

    def C(self, q, dq, params=None):
        """Coriolis matrix chosen so ``C(q, dq) @ dq`` is the RNEA bias force."""
        params = self.params if params is None else params
        dq = np.asarray(dq, dtype=float)
        speed_squared = float(dq @ dq)
        if speed_squared < 1e-16:
            return np.zeros((self.dof, self.dof))
        bias = self._rnea(q, dq, np.zeros(self.dof), params, gravity=False)
        return np.outer(bias, dq) / speed_squared

    def g(self, q, params=None):
        """Gravity generalized force."""
        params = self.params if params is None else params
        zero = np.zeros(self.dof)
        return self._rnea(q, zero, zero, params, gravity=True)

    def d(self, q, dq, u=None, t=0.0, params=None):
        """Linear viscous joint damping."""
        params = self.params if params is None else params
        return np.asarray(params["damping"]) * dq

    def forward_kinematics(self, q, params=None):
        """Tool-center position in the world frame."""
        params = self.params if params is None else params
        _, _, _, links = self._chain(q, params)
        return links[-1, :3, 3]

    def J(self, q, params=None):
        """Translational tool Jacobian."""
        params = self.params if params is None else params
        _, origins, axes, links = self._chain(q, params)
        p = links[-1, :3, 3]
        return np.column_stack(
            [np.cross(axes[i], p - origins[i]) for i in range(self.dof)]
        )

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        q, _ = self.x2q(x)
        joint_frames, origins, _, links = self._chain(q, params)
        points = np.vstack([origins, links[-1, :3, 3]])

        frames = {"base": np.eye(4), "tool": links[-1]}
        for i in range(self.dof):
            frames[f"link{i}"] = link_pose_3d(points[i], points[i + 1])
            frames[f"joint{i}"] = joint_frames[i]
        frames["joint6"] = links[-1]
        return frames


if __name__ == "__main__":
    sys = UR5Manipulator()
    q0 = np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0])
    sys.x0 = sys.q2x(q0, np.zeros(6))
    sys.compute_forced(lambda t: np.zeros(6), tf=3.0, n_steps=120)
    sys.animate(renderer="meshcat", is_3d=True)
