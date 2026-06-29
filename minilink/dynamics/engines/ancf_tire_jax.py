"""
Differentiable JAX prototype of an ANCF-style tire ring.

The model is intentionally small: a tire centerline is represented as a closed
ring of cubic Hermite ANCF elements. Each node carries absolute position and
absolute slope coordinates, and elastic forces are obtained by differentiating
stretch, bending, and area-preservation energy with JAX.

TODO: User Architectural Review. This is an ANCF ring/cable tire prototype, not
Chrono's full laminated ANCF shell tire. It is meant to prove the Minilink
system wrapping, differentiability, contact, and animation path before adding
full shell elements, pressure loads, rim constraints, and tire-section layers.
"""

from typing import NamedTuple

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Plane,
    Sphere,
    camera_matrix,
    identity_matrix,
    translation_matrix,
)


class ANCFTireModel(NamedTuple):
    """Immutable parameters for the differentiable ANCF tire-ring kernel."""

    n_nodes: int
    radius: float
    mass: float
    node_mass: float
    slope_mass: float
    segment_length: float
    area0: float
    inv_mass_q: object
    gravity: object
    plane_normal: object
    plane_offset: float
    k_stretch: float
    k_bend: float
    k_area: float
    k_slope: float
    k_contact: float
    c_contact: float
    mu_static: float
    mu_dynamic: float
    stribeck_velocity: float
    friction_velocity: float
    c_internal: float
    contact_radius: float
    contact_smoothing: float
    xi: object
    wi: object


def make_ancf_tire_model(
    *,
    n_nodes: int = 32,
    radius: float = 0.55,
    mass: float = 18.0,
    slope_mass_ratio: float = 0.05,
    gravity=(0.0, 0.0, -9.81),
    plane_normal=(0.0, 0.0, 1.0),
    plane_offset: float = 0.0,
    k_stretch: float = 2.5e4,
    k_bend: float = 30.0,
    k_area: float = 1.0e4,
    k_slope: float = 2.0e3,
    k_contact: float = 8.0e4,
    c_contact: float = 550.0,
    mu_static: float = 0.9,
    mu_dynamic: float = 0.75,
    stribeck_velocity: float = 0.25,
    friction_velocity: float = 0.05,
    c_internal: float = 0.0,
    contact_radius: float = 0.015,
    contact_smoothing: float = 1.0e-4,
) -> ANCFTireModel:
    """
    Build a differentiable tire-ring model.

    Parameters
    ----------
    n_nodes : int
        Number of ANCF nodes around the closed ring.
    radius : float
        Reference ring radius.
    mass : float
        Total tire mass.
    slope_mass_ratio : float
        Regularizing mass ratio for the slope coordinates relative to node mass.

    Returns
    -------
    ANCFTireModel
        Immutable model record consumed by :func:`ancf_tire_ode`.
    """
    if n_nodes < 4:
        raise ValueError("n_nodes must be at least 4 for a closed tire ring.")
    if radius <= 0.0:
        raise ValueError("radius must be positive.")
    if mass <= 0.0:
        raise ValueError("mass must be positive.")
    if slope_mass_ratio <= 0.0:
        raise ValueError("slope_mass_ratio must be positive.")

    jnp = require_jax_numpy()
    n_nodes = int(n_nodes)
    radius = float(radius)
    mass = float(mass)
    node_mass = mass / n_nodes
    slope_mass = slope_mass_ratio * node_mass
    segment_length = 2.0 * np.pi * radius / n_nodes
    area0 = np.pi * radius * radius

    inv_node = 1.0 / node_mass
    inv_slope = 1.0 / slope_mass
    inv_mass_node = jnp.asarray(
        [inv_node, inv_node, inv_node, inv_slope, inv_slope, inv_slope]
    )
    inv_mass_q = jnp.tile(inv_mass_node, n_nodes)

    n = jnp.asarray(plane_normal, dtype=float)
    n = n / (jnp.linalg.norm(n) + 1e-12)

    # Two-point Gauss rule on xi in [0, 1].
    d = 0.5 / np.sqrt(3.0)
    xi = jnp.asarray([0.5 - d, 0.5 + d])
    wi = jnp.asarray([0.5, 0.5])

    return ANCFTireModel(
        n_nodes=n_nodes,
        radius=radius,
        mass=mass,
        node_mass=node_mass,
        slope_mass=slope_mass,
        segment_length=float(segment_length),
        area0=float(area0),
        inv_mass_q=inv_mass_q,
        gravity=jnp.asarray(gravity, dtype=float),
        plane_normal=n,
        plane_offset=float(plane_offset),
        k_stretch=float(k_stretch),
        k_bend=float(k_bend),
        k_area=float(k_area),
        k_slope=float(k_slope),
        k_contact=float(k_contact),
        c_contact=float(c_contact),
        mu_static=float(mu_static),
        mu_dynamic=float(mu_dynamic),
        stribeck_velocity=float(stribeck_velocity),
        friction_velocity=float(friction_velocity),
        c_internal=float(c_internal),
        contact_radius=float(contact_radius),
        contact_smoothing=float(max(contact_smoothing, 1e-9)),
        xi=xi,
        wi=wi,
    )


def pack_ancf_state(q, v):
    """Pack generalized coordinates and velocities into ``x = [q; v]``."""
    jnp = require_jax_numpy()
    return jnp.concatenate([jnp.asarray(q).reshape(-1), jnp.asarray(v).reshape(-1)])


def unpack_ancf_state(x, n_nodes: int):
    """Unpack state into node coordinate and velocity arrays of shape ``(N, 6)``."""
    jnp = require_jax_numpy()
    x = jnp.asarray(x)
    q_dim = 6 * n_nodes
    q = x[:q_dim].reshape((n_nodes, 6))
    v = x[q_dim : 2 * q_dim].reshape((n_nodes, 6))
    return q, v


def ancf_tire_initial_state(
    model: ANCFTireModel,
    *,
    center=(0.0, 0.0, 1.2),
    linear_velocity=(0.0, 0.0, 0.0),
    angular_velocity=(0.0, -30.0, 0.0),
):
    """
    Reference circular tire state with optional rigid-body initial velocity.

    The wheel axis is the world Y axis. Nodes lie in the XZ plane, so an
    angular velocity about Y gives the falling tire visible spin before impact.
    """
    jnp = require_jax_numpy()
    n_nodes = model.n_nodes
    theta = 2.0 * jnp.pi * jnp.arange(n_nodes) / n_nodes
    dtheta = 2.0 * jnp.pi / n_nodes

    c = jnp.asarray(center, dtype=float)
    v0 = jnp.asarray(linear_velocity, dtype=float)
    w0 = jnp.asarray(angular_velocity, dtype=float)

    zeros = jnp.zeros_like(theta)
    rel = jnp.stack(
        [
            model.radius * jnp.cos(theta),
            zeros,
            model.radius * jnp.sin(theta),
        ],
        axis=1,
    )
    tangent = jnp.stack(
        [
            -model.radius * jnp.sin(theta),
            zeros,
            model.radius * jnp.cos(theta),
        ],
        axis=1,
    )
    slope = tangent * dtheta

    pos = c[None, :] + rel
    pos_dt = v0[None, :] + jnp.cross(w0[None, :], rel)
    slope_dt = jnp.cross(w0[None, :], slope)

    q = jnp.concatenate([pos, slope], axis=1).reshape(-1)
    v = jnp.concatenate([pos_dt, slope_dt], axis=1).reshape(-1)
    return pack_ancf_state(q, v)


def _hermite_terms(xi):
    xi2 = xi * xi
    xi3 = xi2 * xi
    h00 = 2.0 * xi3 - 3.0 * xi2 + 1.0
    h10 = xi3 - 2.0 * xi2 + xi
    h01 = -2.0 * xi3 + 3.0 * xi2
    h11 = xi3 - xi2

    h00_x = 6.0 * xi2 - 6.0 * xi
    h10_x = 3.0 * xi2 - 4.0 * xi + 1.0
    h01_x = -6.0 * xi2 + 6.0 * xi
    h11_x = 3.0 * xi2 - 2.0 * xi

    h00_xx = 12.0 * xi - 6.0
    h10_xx = 6.0 * xi - 4.0
    h01_xx = -12.0 * xi + 6.0
    h11_xx = 6.0 * xi - 2.0
    return (
        (h00, h10, h01, h11),
        (h00_x, h10_x, h01_x, h11_x),
        (h00_xx, h10_xx, h01_xx, h11_xx),
    )


def ancf_tire_potential_energy(model: ANCFTireModel, q):
    """
    Elastic energy for the closed ANCF tire ring.

    Stretch is based on Green strain from the Hermite centerline derivative.
    Bending penalizes curvature magnitude error from the reference circle.
    Area preservation mimics a simple inflated tire cross-section.
    """
    jnp = require_jax_numpy()
    Q = jnp.asarray(q).reshape((model.n_nodes, 6))
    r = Q[:, 0:3]
    d = Q[:, 3:6]
    r_next = jnp.roll(r, -1, axis=0)
    d_next = jnp.roll(d, -1, axis=0)

    xi = model.xi[:, None, None]
    wi = model.wi[:, None]
    _, hx, hxx = _hermite_terms(xi)
    h00_x, h10_x, h01_x, h11_x = hx
    h00_xx, h10_xx, h01_xx, h11_xx = hxx

    r_xi = (
        h00_x * r[None, :, :]
        + h10_x * d[None, :, :]
        + h01_x * r_next[None, :, :]
        + h11_x * d_next[None, :, :]
    )
    r_xixi = (
        h00_xx * r[None, :, :]
        + h10_xx * d[None, :, :]
        + h01_xx * r_next[None, :, :]
        + h11_xx * d_next[None, :, :]
    )

    eps = 1e-12
    L0 = model.segment_length
    r_xi_norm = jnp.sqrt(jnp.sum(r_xi * r_xi, axis=-1) + eps)

    # Axial Green strain along the ANCF centerline.
    strain = 0.5 * ((r_xi_norm / L0) ** 2 - 1.0)
    stretch_energy = jnp.sum(0.5 * model.k_stretch * strain * strain * L0 * wi)

    curvature_vec = jnp.cross(r_xi, r_xixi)
    curvature = jnp.sqrt(jnp.sum(curvature_vec * curvature_vec, axis=-1) + eps)
    curvature = curvature / (r_xi_norm * r_xi_norm * r_xi_norm + eps)
    curvature_error = curvature - 1.0 / model.radius
    bending_energy = jnp.sum(
        0.5 * model.k_bend * curvature_error * curvature_error * L0 * wi
    )

    slope_norm = jnp.sqrt(jnp.sum(d * d, axis=1) + eps)
    slope_strain = slope_norm / L0 - 1.0
    slope_energy = jnp.sum(0.5 * model.k_slope * slope_strain * slope_strain * L0)

    rel = r - jnp.mean(r, axis=0, keepdims=True)
    area_vec = 0.5 * jnp.sum(jnp.cross(rel, jnp.roll(rel, -1, axis=0)), axis=0)
    area = jnp.sqrt(jnp.dot(area_vec, area_vec) + eps)
    area_energy = 0.5 * model.k_area * (area - model.area0) ** 2

    return stretch_energy + bending_energy + slope_energy + area_energy


def _smooth_positive(x, width):
    jnp = require_jax_numpy()
    return width * jnp.logaddexp(0.0, x / width)


def plane_contact_node_forces(model: ANCFTireModel, q, v):
    """
    Smooth tire-plane contact forces at the tire nodes.

    The normal force is a regularized penalty spring-damper. Tangential force is
    a differentiable Coulomb/Stribeck law capped by ``mu * F_n`` and opposed to
    the local slip velocity at the node.
    """
    jnp = require_jax_numpy()
    Q = jnp.asarray(q).reshape((model.n_nodes, 6))
    V = jnp.asarray(v).reshape((model.n_nodes, 6))
    r = Q[:, 0:3]
    r_dt = V[:, 0:3]

    n = model.plane_normal
    signed_dist = r @ n - model.plane_offset
    penetration = model.contact_radius - signed_dist
    active = 0.5 * (jnp.tanh(0.5 * penetration / model.contact_smoothing) + 1.0)
    depth = _smooth_positive(penetration, model.contact_smoothing)
    vn = r_dt @ n
    raw_force = model.k_contact * depth - model.c_contact * vn * active
    force_width = model.k_contact * model.contact_smoothing
    f_n = active * _smooth_positive(raw_force, force_width)
    force_n = f_n[:, None] * n[None, :]

    v_t = r_dt - vn[:, None] * n[None, :]
    slip_speed = jnp.sqrt(jnp.sum(v_t * v_t, axis=1) + 1e-12)
    slip_dir = v_t / slip_speed[:, None]
    mu = model.mu_dynamic + (model.mu_static - model.mu_dynamic) * jnp.exp(
        -((slip_speed / model.stribeck_velocity) ** 2)
    )
    friction_mag = mu * f_n * jnp.tanh(slip_speed / model.friction_velocity)
    force_t = -friction_mag[:, None] * slip_dir

    return force_n + force_t


def ancf_tire_generalized_forces(model: ANCFTireModel, q, v, u=None):
    """Return generalized forces for nodal position and slope coordinates."""
    import jax

    jnp = require_jax_numpy()
    q = jnp.asarray(q).reshape(-1)
    v = jnp.asarray(v).reshape(-1)

    # Conservative ANCF internal force: Q = -dV/dq.
    dV_dq = jax.grad(lambda qq: ancf_tire_potential_energy(model, qq))(q)
    Q = (-dV_dq - model.c_internal * v).reshape((model.n_nodes, 6))

    contact = plane_contact_node_forces(model, q, v)
    gravity = model.node_mass * model.gravity[None, :]
    Q = Q.at[:, 0:3].add(contact + gravity)

    if u is not None:
        force_u = jnp.asarray(u).reshape((model.n_nodes, 3))
        Q = Q.at[:, 0:3].add(force_u)

    return Q.reshape(-1)


def ancf_tire_ode(model: ANCFTireModel, x, u=None):
    """Compute ``dx/dt`` for the ANCF tire-ring state."""
    jnp = require_jax_numpy()
    q_nodes, v_nodes = unpack_ancf_state(x, model.n_nodes)
    q = q_nodes.reshape(-1)
    v = v_nodes.reshape(-1)

    Q = ancf_tire_generalized_forces(model, q, v, u)
    a = Q * model.inv_mass_q

    return jnp.concatenate([v, a])


def _is_jax_array(a) -> bool:
    return type(a).__module__.startswith("jax")


class ANCFTireSystem(DynamicSystem):
    """
    Minilink wrapper for one differentiable ANCF tire-ring world.

    State layout:

    ``x = [q; v]`` where each node contributes ``q_i = [r_i, d_i]`` and
    ``v_i = [rdot_i, ddot_i]``. The input port applies external nodal forces
    ``u_i = [Fx_i, Fy_i, Fz_i]``.
    """

    def __init__(
        self,
        model: ANCFTireModel,
        *,
        center=(0.0, 0.0, 1.2),
        linear_velocity=(0.0, 0.0, 0.0),
        angular_velocity=(0.0, -30.0, 0.0),
        contact_force_scale=0.0015,
        contact_force_threshold=1.0,
        follow_camera=False,
        camera_target=None,
        camera_scale=None,
        name="ANCFTireSystem",
    ):
        self.model = model
        self.contact_force_scale = float(contact_force_scale)
        self.contact_force_threshold = float(contact_force_threshold)
        self.follow_camera = bool(follow_camera)
        n_nodes = model.n_nodes
        q_dim = 6 * n_nodes
        n = 2 * q_dim
        m = 3 * n_nodes
        super().__init__(
            n=n,
            input_dim=m,
            output_dim=n,
            expose_state=True,
            y_dependencies=(),
        )
        self.name = name

        x0 = ancf_tire_initial_state(
            model,
            center=center,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
        self.x0 = np.asarray(x0, dtype=float)

        self.params["n_nodes"] = model.n_nodes
        self.params["radius"] = model.radius
        self.params["mass"] = model.mass
        self.params["k_contact"] = model.k_contact
        self.params["c_contact"] = model.c_contact
        self.params["mu_static"] = model.mu_static
        self.params["mu_dynamic"] = model.mu_dynamic
        self.params["contact_force_scale"] = self.contact_force_scale
        self.params["contact_force_threshold"] = self.contact_force_threshold

        labels = []
        units = []
        for prefix, unit in (("q", "m"), ("v", "m/s")):
            for i in range(n_nodes):
                labels += [
                    f"{prefix}{i}_rx",
                    f"{prefix}{i}_ry",
                    f"{prefix}{i}_rz",
                    f"{prefix}{i}_dx",
                    f"{prefix}{i}_dy",
                    f"{prefix}{i}_dz",
                ]
                units += [unit] * 6
        self.state.labels = labels
        self.state.units = units
        self.outputs["y"].labels = list(labels)
        self.outputs["y"].units = list(units)

        u_labels = []
        for i in range(n_nodes):
            u_labels += [f"F{i}_x", f"F{i}_y", f"F{i}_z"]
        self.inputs["u"].labels = u_labels
        self.inputs["u"].units = ["N"] * m

        self.solver_info["smallest_time_constant"] = 0.002
        self.camera_plot_axes = (0, 2)
        if camera_target is None:
            c = np.asarray(center, dtype=float).reshape(3)
            self.camera_target = np.array(
                [
                    c[0] + model.radius,
                    c[1],
                    model.plane_offset + model.radius,
                ],
                dtype=float,
            )
        else:
            self.camera_target = np.asarray(camera_target, dtype=float).reshape(3)
        self.camera_scale = max(2.0, 4.0 * model.radius)
        if camera_scale is not None:
            self.camera_scale = float(camera_scale)

    def f(self, x, u, t=0.0, params=None):
        model = self.model

        dx = ancf_tire_ode(model, x, u)

        if _is_jax_array(x) or _is_jax_array(u):
            return dx
        return np.asarray(dx, dtype=float)

    def h(self, x, u, t=0.0, params=None):
        return x

    def node_positions(self, x):
        """Return tire node positions as a NumPy array of shape ``(N, 3)``."""
        q_dim = 6 * self.model.n_nodes
        q = np.asarray(x, dtype=float)[:q_dim].reshape((self.model.n_nodes, 6))
        return q[:, 0:3]

    def contact_forces(self, x):
        """Return node contact forces as a NumPy array of shape ``(N, 3)``."""
        q, v = unpack_ancf_state(x, self.model.n_nodes)
        f_contact = plane_contact_node_forces(
            self.model,
            q.reshape(-1),
            v.reshape(-1),
        )
        return np.asarray(f_contact, dtype=float)

    def get_kinematic_geometry(self):
        geometry = {}
        for i in range(self.model.n_nodes):
            geometry[f"seg{i}"] = [
                CustomLine(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], color="black", linewidth=3
                )
            ]
        for i in range(self.model.n_nodes):
            geometry[f"node{i}"] = [
                Sphere(radius=0.025, center=[0.0, 0.0, 0.0], color="red", opacity=1.0)
            ]
        for i in range(self.model.n_nodes):
            geometry[f"arrow{i}"] = [Arrow(color="red", linewidth=4)]
        geometry["world"] = [
            Plane(
                normal=np.asarray(self.model.plane_normal, dtype=float),
                offset=float(self.model.plane_offset),
                size=6.0,
                thickness=0.03,
                color="lightgray",
                opacity=0.65,
            )
        ]
        return geometry

    def tf(self, x, u, t=0, params=None):
        p = self.node_positions(x)
        f_contact = self.contact_forces(x)
        frames = {"world": identity_matrix()}
        n = self.model.n_nodes
        for i in range(n):
            frames[f"seg{i}"] = _line_segment_transform(p[i], p[(i + 1) % n])
        for i in range(n):
            frames[f"node{i}"] = translation_matrix(p[i, 0], p[i, 1], p[i, 2])
        for i in range(n):
            if np.linalg.norm(f_contact[i]) > self.contact_force_threshold:
                f = self.contact_force_scale * f_contact[i]
                frames[f"arrow{i}"] = _vector_arrow_transform(p[i], f)
            else:
                frames[f"arrow{i}"] = _vector_arrow_transform(p[i], np.zeros(3))
        return frames

    def get_camera_transform(self, x, u, t):
        target = np.asarray(self.camera_target, dtype=float)
        if self.follow_camera:
            p = self.node_positions(x)
            target = np.mean(p, axis=0)
            target[2] = max(
                target[2], self.model.plane_offset + 0.7 * self.model.radius
            )
        return camera_matrix(
            target=target,
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )


def _line_segment_transform(p0, p1):
    p0 = np.asarray(p0, dtype=float).reshape(3)
    p1 = np.asarray(p1, dtype=float).reshape(3)
    delta = p1 - p0
    length = np.linalg.norm(delta)

    T = np.eye(4)
    T[:3, 3] = p0
    if length < 1e-12:
        return T

    x_axis = delta / length
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x_axis, reference)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(reference, x_axis)
    y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-12)
    z_axis = np.cross(x_axis, y_axis)

    T[:3, 0] = delta
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis
    return T


def _vector_arrow_transform(origin, vector):
    origin = np.asarray(origin, dtype=float).reshape(3)
    vector = np.asarray(vector, dtype=float).reshape(3)
    length = np.linalg.norm(vector)

    T = np.eye(4)
    T[:3, 3] = origin
    if length < 1e-12:
        # Keep the transform non-singular for Meshcat/Three.js keyframe
        # decomposition while making the local unit arrow visually disappear.
        T[:3, :3] = 1e-9 * np.eye(3)
        return T

    x_axis = vector / length
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x_axis, reference)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(reference, x_axis)
    y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-12)
    z_axis = np.cross(x_axis, y_axis)

    head_width = max(0.02, 0.2 * length)
    T[:3, 0] = vector
    T[:3, 1] = head_width * y_axis
    T[:3, 2] = head_width * z_axis
    return T
