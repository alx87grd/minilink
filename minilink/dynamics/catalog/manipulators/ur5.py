"""UR5 six-axis manipulator: spatial ABA forward dynamics and RNEA inverse dynamics."""

import numpy as np

from minilink.core.backends import array_module
from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.graphical.catalog.shapes import link_pose_3d, point_pose
from minilink.graphical.catalog.skins import ur5_skin

###############################################################################
# Public API
###############################################################################


class UR5Manipulator(Manipulator):
    """Six-degree-of-freedom UR5 arm with a schematic 3-D skin.

    Kinematics use the standard Denavit-Hartenberg convention. Dimensions,
    masses and centers of mass follow the public ``ros-industrial/universal_robot``
    description; they are paired with simple cylinder inertias, so the model is
    suitable for controller examples but is not a factory-calibrated
    representation.

    Equation paths (``H``/``C``/``g``/``d``, FK, ``J``, ``tf``) are NumPy/JAX
    native-array. Forward dynamics and ``f`` use the spatial ABA;
    ``inverse_dynamics`` uses spatial RNEA; ``H``/``C``/``g`` remain RNEA-based
    for analysis and matrix-form checks.
    """

    def __init__(self):
        super().__init__(dof=6, actuators=6, task_dim=3)

        # The plant's own range: angles and rates within ±2π
        self.state.lower_bound = np.full(self.n, -2 * np.pi)
        self.state.upper_bound = np.full(self.n, 2 * np.pi)

        # Parameters: the DH chain, the published masses and centers of mass, and
        # cylinder inertias
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
                    cylinder_inertia(masses[0], 0.15, 0.06, axis=2),
                    cylinder_inertia(masses[1], 0.56, 0.06, axis=0),
                    cylinder_inertia(masses[2], 0.39225, 0.06, axis=0),
                    cylinder_inertia(masses[3], 0.12, 0.06, axis=2),
                    cylinder_inertia(masses[4], 0.12, 0.06, axis=2),
                    cylinder_inertia(masses[5], 0.0345, 0.0375, axis=2),
                ]
            ),
            "gravity": 9.81,
            "damping": np.full(6, 0.05),
        }

        # Signal labels, units and torque limits
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

        # Graphics
        self.skin = ur5_skin
        self.camera_scale = 1.3
        self.camera_target = np.array([-0.3, 0.0, 0.25])

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        """Forward dynamics via articulated-body algorithm (default simulation path)."""
        params = self.params if params is None else params
        xp = array_module(q, v, u)

        q = xp.asarray(q)
        v = xp.asarray(v)
        u = xp.asarray(u)
        tau = self.generalized_force(q, v, u, t, params)
        d = self.d(q, v, u, t, params)

        # H q̈ = τ − d − C q̇ − g, solved in O(n) by the articulated-body algorithm
        tau_net = tau - d
        qdd = self.aba(q, v, tau_net, params, gravity=True)

        return qdd

    def forward_dynamics_rnea_h(self, q, v, u, t=0.0, params=None):
        """Forward dynamics via RNEA bias + explicit ``H`` (teaching / verification)."""
        params = self.params if params is None else params
        xp = array_module(q, v, u)

        dof = self.dof

        q = xp.asarray(q)
        v = xp.asarray(v)
        u = xp.asarray(u)
        H = self.H(q, params)
        bias = self.rnea(q, v, xp.zeros(dof), params, gravity=True)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)

        # H q̈ = τ − (C q̇ + g) − d, the bias C q̇ + g from RNEA at zero acceleration
        qdd = xp.linalg.solve(H, tau - bias - d)

        return qdd

    def forward_dynamics_aba(self, q, v, u, t=0.0, params=None):
        """Alias of :meth:`forward_dynamics` (explicit ABA name for comparisons)."""
        return self.forward_dynamics(q, v, u, t, params)

    def inverse_dynamics(self, q, v, acceleration, u=None, t=0.0, params=None):
        """Inverse dynamics via spatial RNEA (default path)."""
        params = self.params if params is None else params

        d = self.d(q, v, u, t, params)
        tau_rnea = self.rnea(q, v, acceleration, params, gravity=True)

        # τ = RNEA(q, q̇, q̈) + d(q, q̇)
        tau = tau_rnea + d

        return tau

    def inverse_dynamics_matrix(self, q, v, acceleration, u=None, t=0.0, params=None):
        """Inverse dynamics via explicit ``H``, ``C``, ``g`` (teaching / verification)."""
        params = self.params if params is None else params
        xp = array_module(q, v, acceleration)

        q = xp.asarray(q)
        v = xp.asarray(v)
        acceleration = xp.asarray(acceleration)
        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)

        # τ = H q̈ + C q̇ + g + d
        tau = H @ acceleration + C @ v + g + d

        return tau

    def inverse_dynamics_rnea(self, q, v, acceleration, u=None, t=0.0, params=None):
        """Alias of :meth:`inverse_dynamics` (explicit RNEA name for comparisons)."""
        return self.inverse_dynamics(q, v, acceleration, u, t, params)

    def H(self, q, params=None):
        """Joint-space inertia matrix."""
        params = self.params if params is None else params
        xp = array_module(q)

        dof = self.dof

        q = xp.asarray(q)
        zero = xp.zeros(dof)

        # Column j is the inverse dynamics of a unit acceleration of joint j, at rest
        # and without gravity
        columns = [
            self.rnea(q, zero, direction, params, gravity=False)
            for direction in xp.eye(dof)
        ]
        H = xp.stack(columns, axis=1)

        # H = ½ (H + Hᵀ), symmetric against round-off
        H = 0.5 * (H + H.T)

        return H

    def C(self, q, dq, params=None):
        """Coriolis matrix chosen so ``C(q, dq) @ dq`` is the RNEA bias force."""
        params = self.params if params is None else params
        xp = array_module(q, dq)

        dof = self.dof

        q = xp.asarray(q)
        dq = xp.asarray(dq)

        # The bias b = C q̇ is the inverse dynamics at zero acceleration, without gravity
        speed_squared = dq @ dq
        denom = xp.maximum(speed_squared, 1e-12)
        bias = self.rnea(q, dq, xp.zeros(dof), params, gravity=False)

        # C = b q̇ᵀ / ‖q̇‖², a rank-1 map; near rest (‖q̇‖² < 10⁻¹²) it is ill-conditioned,
        # so the bias is taken as zero there
        C = xp.outer(bias, dq) / denom
        C = xp.where(speed_squared < 1e-12, xp.zeros((dof, dof)), C)

        return C

    def g(self, q, params=None):
        """Gravity generalized force."""
        params = self.params if params is None else params
        xp = array_module(q)

        dof = self.dof

        q = xp.asarray(q)
        zero = xp.zeros(dof)

        # g(q) is the inverse dynamics at rest, q̇ = q̈ = 0
        g = self.rnea(q, zero, zero, params, gravity=True)

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        """Linear viscous joint damping."""
        params = self.params if params is None else params
        xp = array_module(q, dq)

        damping = xp.asarray(params["damping"])

        dq = xp.asarray(dq)

        # τ_d = b q̇, joint by joint
        tau_d = damping * dq

        return tau_d

    def forward_kinematics(self, q, params=None):
        """Tool-center position in the world frame."""
        params = self.params if params is None else params

        _, _, _, links = self.kinematic_chain(q, params)

        # The tool sits at the origin of the last link frame
        p = links[-1, :3, 3]

        return p

    def J(self, q, params=None):
        """Translational tool Jacobian."""
        params = self.params if params is None else params
        xp = array_module(q)

        dof = self.dof

        _, origins, axes, links = self.kinematic_chain(q, params)
        p = links[-1, :3, 3]

        # Column i: the tool velocity per unit rate of revolute joint i, zᵢ × (p − oᵢ)
        columns = [xp.cross(axes[i], p - origins[i]) for i in range(dof)]
        J = xp.stack(columns, axis=1)

        return J

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        dof = self.dof

        q, _ = self.x2q(x)
        joint_frames, origins, _, links = self.kinematic_chain(q, params)
        tip = links[-1, :3, 3]
        points = xp.concatenate([origins, tip.reshape(1, 3)], axis=0)

        # Each link drawn between its joint and the next, each joint at its DH frame
        frames = {"base": xp.eye(4), "tool": links[-1]}
        for i in range(dof):
            frames[f"link{i}"] = link_pose_3d(points[i], points[i + 1])
            frames[f"joint{i}"] = joint_frames[i]
        frames["joint6"] = links[-1]

        # Translation-only frame so a world-frame tool force is not rotated
        # by the wrist (see Manipulator.get_dynamic_geometry, port ``f``).
        frames["force"] = point_pose(tip)

        return frames

    # The recursive algorithms: the DH chain, RNEA and ABA

    def kinematic_chain(self, q, params=None):
        """Each joint's frame, origin and axis, and each link's frame, in the base frame."""
        params = self.params if params is None else params
        xp = array_module(q)

        dof = self.dof
        a = xp.asarray(params["a"])
        d = xp.asarray(params["d"])
        alpha = xp.asarray(params["alpha"])

        q = xp.asarray(q)

        # From the base outward: joint i turns about the z axis of T_{i−1}, and
        # T_i = T_{i−1} A_i(θᵢ)
        T = xp.eye(4)
        joint_frames = []
        joint_origins = []
        joint_axes = []
        link_frames = []
        for i in range(dof):
            joint_frames.append(T)
            joint_origins.append(T[:3, 3])
            joint_axes.append(T[:3, 2])
            T = T @ dh_transform(q[i], d[i], a[i], alpha[i])
            link_frames.append(T)

        joint_frames = xp.stack(joint_frames)
        joint_origins = xp.stack(joint_origins)
        joint_axes = xp.stack(joint_axes)
        link_frames = xp.stack(link_frames)

        return joint_frames, joint_origins, joint_axes, link_frames

    def rnea(self, q, dq, acceleration, params=None, *, gravity=True):
        """Recursive Newton-Euler inverse dynamics in link coordinates."""
        params = self.params if params is None else params
        xp = array_module(q, dq, acceleration)

        dof = self.dof
        a = xp.asarray(params["a"])
        d = xp.asarray(params["d"])
        alpha = xp.asarray(params["alpha"])
        mass = xp.asarray(params["mass"])
        com = xp.asarray(params["com"])
        inertia = xp.asarray(params["inertia"])
        gravity_value = xp.asarray(params["gravity"])

        q = xp.asarray(q)
        dq = xp.asarray(dq)
        acceleration = xp.asarray(acceleration)

        # The base accelerates upward at g, which stands for gravity on every link:
        # a₀ = [0, 0, 0, 0, 0, g]
        a_0 = xp.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) * gravity_value
        if not gravity:
            a_0 = xp.zeros(6)

        # Forward pass, from the base outward
        Xup = []
        S_list = []
        force = []
        parent_motion = xp.zeros(6)
        parent_acceleration = a_0
        for i in range(dof):
            # The link's spatial transform from its parent, and its joint axis
            X, S = link_transform(q[i], d[i], a[i], alpha[i])
            Xup.append(X)
            S_list.append(S)

            # vᵢ = Xᵢ v_{i−1} + Sᵢ q̇ᵢ, and aᵢ = Xᵢ a_{i−1} + Sᵢ q̈ᵢ + vᵢ × Sᵢ q̇ᵢ
            joint_motion = S * dq[i]
            vi = X @ parent_motion + joint_motion
            ai = (
                X @ parent_acceleration
                + S * acceleration[i]
                + motion_cross(vi) @ joint_motion
            )

            # fᵢ = Iᵢ aᵢ + vᵢ ×* Iᵢ vᵢ, the force the link needs
            I = spatial_inertia(mass[i], com[i], inertia[i])
            fi = I @ ai - motion_cross(vi).T @ (I @ vi)

            force.append(fi)
            parent_motion = vi
            parent_acceleration = ai

        # Backward pass: τᵢ = Sᵢᵀ fᵢ, and each link's force passes on to its parent,
        # f_{i−1} += Xᵢᵀ fᵢ
        tau_parts = []
        for i in reversed(range(dof)):
            tau_parts.append(S_list[i] @ force[i])
            if i > 0:
                force[i - 1] = force[i - 1] + Xup[i].T @ force[i]

        tau = xp.stack(tau_parts[::-1])

        return tau

    def aba(self, q, dq, tau, params=None, *, gravity=True):
        """Articulated-body forward dynamics — O(n) generalized accelerations."""
        params = self.params if params is None else params
        xp = array_module(q, dq, tau)

        dof = self.dof
        a = xp.asarray(params["a"])
        d = xp.asarray(params["d"])
        alpha = xp.asarray(params["alpha"])
        mass = xp.asarray(params["mass"])
        com = xp.asarray(params["com"])
        inertia = xp.asarray(params["inertia"])
        gravity_value = xp.asarray(params["gravity"])

        q = xp.asarray(q)
        dq = xp.asarray(dq)
        tau = xp.asarray(tau)

        # The base accelerates upward at g, which stands for gravity on every link:
        # a₀ = [0, 0, 0, 0, 0, g]
        a_0 = xp.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) * gravity_value
        if not gravity:
            a_0 = xp.zeros(6)

        # First pass, from the base outward: velocities, and each link's own inertia
        # and bias force
        Xup = []
        S_list = []
        c = []
        Ia = []
        pa = []
        parent_motion = xp.zeros(6)
        for i in range(dof):
            # The link's spatial transform from its parent, and its joint axis
            X, S = link_transform(q[i], d[i], a[i], alpha[i])
            Xup.append(X)
            S_list.append(S)

            # vᵢ = Xᵢ v_{i−1} + Sᵢ q̇ᵢ, cᵢ = vᵢ × Sᵢ q̇ᵢ, and pᵢ = vᵢ ×* Iᵢ vᵢ
            joint_motion = S * dq[i]
            vi = X @ parent_motion + joint_motion
            ci = motion_cross(vi) @ joint_motion
            I = spatial_inertia(mass[i], com[i], inertia[i])
            p_i = -motion_cross(vi).T @ (I @ vi)

            c.append(ci)
            Ia.append(I)
            pa.append(p_i)
            parent_motion = vi

        # Second pass, from the tip inward: each articulated body folds into its parent
        U = []
        d_inv = []
        u = []
        for i in reversed(range(dof)):
            # Uᵢ = Iᴬᵢ Sᵢ, Dᵢ = Sᵢᵀ Uᵢ, and uᵢ = τᵢ − Sᵢᵀ pᴬᵢ
            Ui = Ia[i] @ S_list[i]
            di = S_list[i] @ Ui
            ui = tau[i] - S_list[i] @ pa[i]
            U.append(Ui)
            d_inv.append(xp.reciprocal(di))
            u.append(ui)

            # Iᵃ = Iᴬᵢ − Uᵢ Dᵢ⁻¹ Uᵢᵀ and pᵃ = pᴬᵢ + Iᵃ cᵢ + Uᵢ Dᵢ⁻¹ uᵢ, carried to the
            # parent: Iᴬ_{i−1} += Xᵢᵀ Iᵃ Xᵢ and pᴬ_{i−1} += Xᵢᵀ pᵃ
            if i > 0:
                Ii = Ia[i] - xp.outer(Ui, Ui) * d_inv[-1]
                pai = pa[i] + Ii @ c[i] + Ui * ui * d_inv[-1]
                Ia[i - 1] = Ia[i - 1] + Xup[i].T @ Ii @ Xup[i]
                pa[i - 1] = pa[i - 1] + Xup[i].T @ pai
        U.reverse()
        d_inv.reverse()
        u.reverse()

        # Third pass, from the base outward: a′ᵢ = Xᵢ a_{i−1} + cᵢ,
        # q̈ᵢ = Dᵢ⁻¹ (uᵢ − Uᵢᵀ a′ᵢ), and aᵢ = a′ᵢ + Sᵢ q̈ᵢ
        qdd_parts = []
        parent_acceleration = a_0
        for i in range(dof):
            ai = Xup[i] @ parent_acceleration + c[i]
            qdd_i = (u[i] - U[i] @ ai) * d_inv[i]
            qdd_parts.append(qdd_i)
            parent_acceleration = ai + S_list[i] * qdd_i

        qdd = xp.stack(qdd_parts)

        return qdd


###############################################################################
# Internal machinery
###############################################################################


def cylinder_inertia(mass, length, radius, axis):
    """Diagonal COM inertia for a solid cylinder aligned with *axis*."""
    # I_axial = ½ m r², and I_transverse = m (3 r² + l²) / 12 about the other two axes
    axial = 0.5 * mass * radius**2
    transverse = mass * (3.0 * radius**2 + length**2) / 12.0

    diagonal = np.full(3, transverse)
    diagonal[axis] = axial
    inertia = np.diag(diagonal)

    return inertia


def dh_transform(theta, d, a, alpha):
    """Standard DH transform from one link frame to the next."""
    xp = array_module(theta, d, a, alpha)

    ct = xp.cos(theta)
    st = xp.sin(theta)
    ca = xp.cos(alpha)
    sa = xp.sin(alpha)
    one = xp.ones_like(ct)
    zero = xp.zeros_like(ct)

    # A = Rot_z(θ) Trans_z(d) Trans_x(a) Rot_x(α)
    # fmt: off
    A = xp.array([
        [ct,   -st * ca,  st * sa, a * ct],
        [st,    ct * ca, -ct * sa, a * st],
        [zero,  sa,       ca,      d],
        [zero,  zero,     zero,    one],
    ])
    # fmt: on

    return A


def link_transform(theta, d, a, alpha):
    """The spatial transform ``X`` from the parent frame to the link, and the joint's motion subspace ``S``."""
    xp = array_module(theta, d, a, alpha)

    T = dh_transform(theta, d, a, alpha)
    R = T[:3, :3]
    r = T[:3, 3]
    Rt = R.T
    z = xp.array([0.0, 0.0, 1.0])

    # X = [Rᵀ 0; −Rᵀ [r]× Rᵀ], the parent's coordinates carried into the link's
    top = xp.concatenate([Rt, xp.zeros((3, 3))], axis=1)
    bottom = xp.concatenate([-Rt @ skew(r), Rt], axis=1)
    X = xp.concatenate([top, bottom], axis=0)

    # S = [Rᵀ z; Rᵀ (z × r)]: the revolute joint turns about the parent's z axis
    S = xp.concatenate([Rt @ z, Rt @ xp.cross(z, r)])

    return X, S


def skew(v):
    """The cross-product matrix ``[v]×`` of a 3-vector."""
    xp = array_module(v)

    v = xp.asarray(v)
    zero = xp.zeros_like(v[0])

    # fmt: off
    v_cross = xp.array([
        [ zero, -v[2],  v[1]],
        [ v[2],  zero, -v[0]],
        [-v[1],  v[0],  zero],
    ])
    # fmt: on

    return v_cross


def motion_cross(v):
    """The spatial motion cross-product matrix ``v×`` of a 6-vector ``[ω; v]``."""
    xp = array_module(v)

    v = xp.asarray(v)
    w = skew(v[:3])
    vlin = skew(v[3:])
    z = xp.zeros((3, 3))

    # v× = [[ω]× 0; [v]× [ω]×]
    top = xp.concatenate([w, z], axis=1)
    bottom = xp.concatenate([vlin, w], axis=1)
    v_cross = xp.concatenate([top, bottom], axis=0)

    return v_cross


def spatial_inertia(mass, com, inertia):
    """The 6×6 spatial inertia of a body from its mass, center of mass and COM inertia."""
    xp = array_module(com, inertia, mass)

    mass = xp.asarray(mass)
    com = xp.asarray(com)
    inertia = xp.asarray(inertia)
    c = skew(com)

    # I = [I_c + m [c]×ᵀ [c]×, m [c]×; m [c]×ᵀ, m 𝟙], about the link frame's origin
    I_ang = inertia + mass * (c.T @ c)
    m_c = mass * c
    m_eye = mass * xp.eye(3)

    top = xp.concatenate([I_ang, m_c], axis=1)
    bottom = xp.concatenate([m_c.T, m_eye], axis=1)
    I = xp.concatenate([top, bottom], axis=0)

    return I


if __name__ == "__main__":
    sys = UR5Manipulator()
    q0 = np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0])
    sys.x0 = sys.q2x(q0, np.zeros(6))
    sys.compute_forced(
        lambda t: np.zeros(6),
        tf=3.0,
        n_steps=120,
        backend="jax",
    )
    sys.animate(renderer="meshcat", is_3d=True)
