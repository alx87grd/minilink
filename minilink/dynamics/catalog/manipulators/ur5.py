"""UR5 six-axis manipulator with numeric rigid-body dynamics.

The kinematic dimensions, masses, and centers of mass follow the public
``ros-industrial/universal_robot`` UR5 description. Link inertias are cylinder
approximations, so this model is intended for controls teaching and simulation,
not calibrated prediction of a particular robot.

Dynamics, kinematics, Jacobian, and ``tf`` are native-array paths: the same
methods run under NumPy and trace under JAX via ``array_module``.

Equation of motion::

    H(q) qdd + C(q, qd) qd + d(q, qd) + g(q) = tau
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.graphical.catalog.shapes import link_pose_3d
from minilink.graphical.catalog.skins import ur5_skin


def _dh_transform(theta, d, a, alpha):
    """Standard DH transform from one link frame to the next."""
    xp = array_module(theta, d, a, alpha)
    ct, st = xp.cos(theta), xp.sin(theta)
    ca, sa = xp.cos(alpha), xp.sin(alpha)
    one, zero = xp.ones_like(ct), xp.zeros_like(ct)
    # fmt: off
    return xp.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [zero,      sa,       ca,      d],
        [zero,    zero,     zero,    one],
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
    xp = array_module(v)
    v = xp.asarray(v)
    zero = xp.zeros_like(v[0])
    # fmt: off
    return xp.array([
        [ zero, -v[2],  v[1]],
        [ v[2],  zero, -v[0]],
        [-v[1],  v[0],  zero],
    ])
    # fmt: on


def _motion_cross(v):
    xp = array_module(v)
    v = xp.asarray(v)
    w = _skew(v[:3])
    vlin = _skew(v[3:])
    z = xp.zeros((3, 3))
    top = xp.concatenate([w, z], axis=1)
    bottom = xp.concatenate([vlin, w], axis=1)
    return xp.concatenate([top, bottom], axis=0)


def _spatial_inertia(mass, com, inertia):
    xp = array_module(com, inertia, mass)
    mass = xp.asarray(mass)
    com = xp.asarray(com)
    inertia = xp.asarray(inertia)
    c = _skew(com)
    I_ang = inertia + mass * (c.T @ c)
    m_c = mass * c
    m_eye = mass * xp.eye(3)
    top = xp.concatenate([I_ang, m_c], axis=1)
    bottom = xp.concatenate([m_c.T, m_eye], axis=1)
    return xp.concatenate([top, bottom], axis=0)


def _as_params(params, *keys, like):
    """Cast selected parameter values to the backend of *like*."""
    xp = array_module(like)
    return tuple(xp.asarray(params[key]) for key in keys)


class UR5Manipulator(Manipulator):
    """Six-degree-of-freedom UR5 arm with a schematic 3-D skin.

    Kinematics use the standard Denavit-Hartenberg convention. The published
    UR5 masses and centers of mass are paired with simple cylinder inertias;
    consequently the model is suitable for controller examples but is not a
    factory-calibrated representation.

    Equation paths (``H``/``C``/``g``/``d``, FK, ``J``, ``tf``) are NumPy/JAX
    native-array: pass NumPy arrays for ordinary use, or JAX arrays for JIT,
    gradients, and ``compile_backend="jax"`` simulation.
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
        xp = array_module(q)
        q = xp.asarray(q)
        a, d, alpha = _as_params(params, "a", "d", "alpha", like=q)

        T = xp.eye(4)
        joint_frames = []
        joint_origins = []
        joint_axes = []
        link_frames = []
        for i in range(self.dof):
            joint_frames.append(T)
            joint_origins.append(T[:3, 3])
            joint_axes.append(T[:3, 2])
            T = T @ _dh_transform(q[i], d[i], a[i], alpha[i])
            link_frames.append(T)
        return (
            xp.stack(joint_frames),
            xp.stack(joint_origins),
            xp.stack(joint_axes),
            xp.stack(link_frames),
        )

    def _rnea(self, q, dq, acceleration, params=None, *, gravity=True):
        """Recursive Newton-Euler inverse dynamics in link coordinates."""
        params = self.params if params is None else params
        xp = array_module(q, dq, acceleration)
        q = xp.asarray(q)
        dq = xp.asarray(dq)
        acceleration = xp.asarray(acceleration)
        a, d, alpha, mass, com, inertia = _as_params(
            params, "a", "d", "alpha", "mass", "com", "inertia", like=q
        )
        gravity_value = xp.asarray(params["gravity"])

        Xup = []
        force = []
        g_vec = xp.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) * gravity_value
        base_acceleration = g_vec if gravity else xp.zeros(6)

        parent_motion = xp.zeros(6)
        parent_acceleration = base_acceleration
        z = xp.array([0.0, 0.0, 1.0])
        for i in range(self.dof):
            T = _dh_transform(q[i], d[i], a[i], alpha[i])
            R = T[:3, :3]
            r = T[:3, 3]
            Rt = R.T

            X = xp.concatenate(
                [
                    xp.concatenate([Rt, xp.zeros((3, 3))], axis=1),
                    xp.concatenate([-Rt @ _skew(r), Rt], axis=1),
                ],
                axis=0,
            )
            Xup.append(X)

            S = xp.concatenate([Rt @ z, Rt @ xp.cross(z, r)])
            joint_motion = S * dq[i]
            vi = X @ parent_motion + joint_motion
            ai = (
                X @ parent_acceleration
                + S * acceleration[i]
                + _motion_cross(vi) @ joint_motion
            )
            I = _spatial_inertia(mass[i], com[i], inertia[i])
            fi = I @ ai - _motion_cross(vi).T @ (I @ vi)

            force.append(fi)
            parent_motion = vi
            parent_acceleration = ai

        tau = []
        for i in reversed(range(self.dof)):
            T = _dh_transform(q[i], d[i], a[i], alpha[i])
            R = T[:3, :3]
            r = T[:3, 3]
            S = xp.concatenate([R.T @ z, R.T @ xp.cross(z, r)])
            tau.append(S @ force[i])
            if i > 0:
                force[i - 1] = force[i - 1] + Xup[i].T @ force[i]
        return xp.stack(tau[::-1])

    def H(self, q, params=None):
        """Joint-space inertia matrix."""
        params = self.params if params is None else params
        xp = array_module(q)
        q = xp.asarray(q)
        zero = xp.zeros(self.dof)
        columns = [
            self._rnea(q, zero, direction, params, gravity=False)
            for direction in xp.eye(self.dof)
        ]
        H = xp.stack(columns, axis=1)
        return 0.5 * (H + H.T)

    def C(self, q, dq, params=None):
        """Coriolis matrix chosen so ``C(q, dq) @ dq`` is the RNEA bias force."""
        params = self.params if params is None else params
        xp = array_module(q, dq)
        q = xp.asarray(q)
        dq = xp.asarray(dq)
        speed_squared = dq @ dq
        # Near rest the rank-1 map is ill-conditioned; treat bias as zero.
        denom = xp.maximum(speed_squared, 1e-12)
        bias = self._rnea(q, dq, xp.zeros(self.dof), params, gravity=False)
        C = xp.outer(bias, dq) / denom
        return xp.where(speed_squared < 1e-12, xp.zeros((self.dof, self.dof)), C)

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        """Forward dynamics via RNEA bias ``C v + g`` (avoids ill-conditioned ``C``)."""
        params = self.params if params is None else params
        xp = array_module(q, v, u)
        q = xp.asarray(q)
        v = xp.asarray(v)
        u = xp.asarray(u)
        H = self.H(q, params)
        bias = self._rnea(q, v, xp.zeros(self.dof), params, gravity=True)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)
        return xp.linalg.solve(H, tau - bias - d)

    def g(self, q, params=None):
        """Gravity generalized force."""
        params = self.params if params is None else params
        xp = array_module(q)
        q = xp.asarray(q)
        zero = xp.zeros(self.dof)
        return self._rnea(q, zero, zero, params, gravity=True)

    def d(self, q, dq, u=None, t=0.0, params=None):
        """Linear viscous joint damping."""
        params = self.params if params is None else params
        xp = array_module(q, dq)
        dq = xp.asarray(dq)
        damping = xp.asarray(params["damping"])
        return damping * dq

    def forward_kinematics(self, q, params=None):
        """Tool-center position in the world frame."""
        params = self.params if params is None else params
        _, _, _, links = self._chain(q, params)
        return links[-1, :3, 3]

    def J(self, q, params=None):
        """Translational tool Jacobian."""
        params = self.params if params is None else params
        xp = array_module(q)
        _, origins, axes, links = self._chain(q, params)
        p = links[-1, :3, 3]
        columns = [xp.cross(axes[i], p - origins[i]) for i in range(self.dof)]
        return xp.stack(columns, axis=1)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        q, _ = self.x2q(x)
        joint_frames, origins, _, links = self._chain(q, params)
        tip = links[-1, :3, 3]
        points = xp.concatenate([origins, tip.reshape(1, 3)], axis=0)

        frames = {"base": xp.eye(4), "tool": links[-1]}
        for i in range(self.dof):
            frames[f"link{i}"] = link_pose_3d(points[i], points[i + 1])
            frames[f"joint{i}"] = joint_frames[i]
        frames["joint6"] = links[-1]
        return frames


if __name__ == "__main__":
    sys = UR5Manipulator()
    q0 = np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0])
    sys.x0 = sys.q2x(q0, np.zeros(6))
    sys.compute_forced(
        lambda t: np.zeros(6),
        tf=3.0,
        n_steps=120,
        compile_backend="jax",
    )
    sys.animate(renderer="meshcat", is_3d=True)
