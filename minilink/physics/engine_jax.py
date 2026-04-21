"""
Minimal JAX rigid-body engine core (MVP).

Current features:
- spherical rigid bodies,
- fixed plane contact via penalty spring-damper,
- full 6-DoF body state (position, quaternion, linear/angular velocity),
- pure functional ODE API suitable for JIT/grad.
"""

from __future__ import annotations

from typing import NamedTuple


class SphereModel(NamedTuple):
    mass: float
    radius: float


class PlaneModel(NamedTuple):
    normal: tuple[float, float, float]
    offset: float


class WorldModel(NamedTuple):
    n_bodies: int
    masses: object
    radii: object
    inertias: object
    inv_masses: object
    inv_inertias: object
    gravity: object
    plane_normal: object
    plane_offset: float
    k_contact: float
    c_contact: float


def _jnp():
    try:
        import jax.numpy as jnp
    except ImportError as e:
        raise ImportError(
            "JAX physics engine requires JAX. Install with: pip install jax jaxlib"
        ) from e
    return jnp


def _normalize(v, eps=1e-12):
    jnp = _jnp()
    return v / (jnp.linalg.norm(v) + eps)


def make_world_model(
    spheres: list[SphereModel],
    plane: PlaneModel,
    *,
    gravity=(0.0, 0.0, -9.81),
    k_contact=5e4,
    c_contact=300.0,
):
    """Create immutable world parameters for the physics engine."""
    jnp = _jnp()
    n = len(spheres)
    if n == 0:
        raise ValueError("At least one sphere is required.")

    masses = jnp.asarray([s.mass for s in spheres])
    radii = jnp.asarray([s.radius for s in spheres])
    inertias = 0.4 * masses * radii * radii  # I = 2/5 m r^2
    inv_masses = 1.0 / masses
    inv_inertias = 1.0 / inertias

    nrm = _normalize(jnp.asarray(plane.normal))
    g = jnp.asarray(gravity)
    return WorldModel(
        n_bodies=n,
        masses=masses,
        radii=radii,
        inertias=inertias,
        inv_masses=inv_masses,
        inv_inertias=inv_inertias,
        gravity=g,
        plane_normal=nrm,
        plane_offset=float(plane.offset),
        k_contact=float(k_contact),
        c_contact=float(c_contact),
    )


def pack_state(pos, quat, lin_vel, ang_vel):
    """Pack state arrays (N,3),(N,4),(N,3),(N,3) to flat vector (13N,)."""
    jnp = _jnp()
    x = jnp.concatenate([pos, quat, lin_vel, ang_vel], axis=1)
    return x.reshape(-1)


def unpack_state(x, n_bodies: int):
    """Unpack flat state vector (13N,) to arrays."""
    jnp = _jnp()
    X = jnp.asarray(x).reshape((n_bodies, 13))
    pos = X[:, 0:3]
    quat = X[:, 3:7]
    lin_vel = X[:, 7:10]
    ang_vel = X[:, 10:13]
    return pos, quat, lin_vel, ang_vel


def normalize_quaternion(q, eps=1e-12):
    """Normalize quaternion q=[w,x,y,z]."""
    jnp = _jnp()
    return q / (jnp.linalg.norm(q, axis=-1, keepdims=True) + eps)


def quat_derivative(q, w):
    """Quaternion kinematics: qdot = 0.5 * omega(w) @ q."""
    jnp = _jnp()
    wx, wy, wz = w[..., 0], w[..., 1], w[..., 2]
    O = jnp.stack(
        [
            jnp.stack([0.0 * wx, -wx, -wy, -wz], axis=-1),
            jnp.stack([wx, 0.0 * wx, wz, -wy], axis=-1),
            jnp.stack([wy, -wz, 0.0 * wx, wx], axis=-1),
            jnp.stack([wz, wy, -wx, 0.0 * wx], axis=-1),
        ],
        axis=-2,
    )
    return 0.5 * jnp.einsum("nij,nj->ni", O, q)


def plane_contact_force(world: WorldModel, pos, lin_vel):
    """
    Contact force for spheres against a fixed plane.

    Penetration depth:
        d = radius - (n dot p - offset)
    Force (normal-only spring-damper):
        f_n = max(0, k*d - c*(n dot v))
        F = f_n * n if d > 0 else 0
    """
    jnp = _jnp()
    n = world.plane_normal
    signed_dist = jnp.einsum("j,nj->n", n, pos) - world.plane_offset
    penetration = world.radii - signed_dist
    vn = jnp.einsum("j,nj->n", n, lin_vel)
    f_mag = world.k_contact * penetration - world.c_contact * vn
    f_mag = jnp.where(penetration > 0.0, jnp.maximum(f_mag, 0.0), 0.0)
    return f_mag[:, None] * n[None, :]


def world_ode(world: WorldModel, x, u):
    """Compute dx/dt for the full world state."""
    jnp = _jnp()
    pos, quat, lin_vel, ang_vel = unpack_state(x, world.n_bodies)
    quat = normalize_quaternion(quat)

    u = jnp.asarray(u).reshape((world.n_bodies, 6))
    force_u = u[:, 0:3]
    torque_u = u[:, 3:6]

    force_contact = plane_contact_force(world, pos, lin_vel)
    force_gravity = world.masses[:, None] * world.gravity[None, :]
    acc = (force_u + force_contact + force_gravity) * world.inv_masses[:, None]

    # Sphere inertia tensor is isotropic => component-wise scaling.
    ang_acc = torque_u * world.inv_inertias[:, None]
    qdot = quat_derivative(quat, ang_vel)

    return pack_state(lin_vel, qdot, acc, ang_acc)
