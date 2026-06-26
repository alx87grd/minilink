"""
Rigid-body pose and transform algebra (native-array, JAX-traceable).

Homogeneous 4x4 transforms for :meth:`~minilink.core.system.System.tf` and
collision placement. Matrices are built functionally (``xp.stack`` / ``xp.array``)
so they trace under JAX when inputs are JAX arrays.
"""

from minilink.core.backends import array_module

# Public API


def identity_matrix(xp=None):
    """Return a 4x4 identity transform."""
    if xp is None:
        import numpy as np

        xp = np
    elif not hasattr(xp, "eye"):
        xp = array_module(xp)
    return xp.eye(4)


def translation_matrix(dx=0.0, dy=0.0, dz=0.0):
    """Pure translation homogeneous transform."""
    xp = array_module(dx, dy, dz)
    return xp.array(
        [
            [1.0, 0.0, 0.0, dx],
            [0.0, 1.0, 0.0, dy],
            [0.0, 0.0, 1.0, dz],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def pose2d_matrix(x=0.0, y=0.0, theta=0.0):
    """SE(2) pose embedded in 4x4 (rotation about Z, translation in XY)."""
    xp = array_module(x, y, theta)
    c = xp.cos(theta)
    s = xp.sin(theta)
    return xp.array(
        [
            [c, -s, 0.0, x],
            [s, c, 0.0, y],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def rotation_matrix_x(theta=0.0):
    """Rotation about the X axis."""
    xp = array_module(theta)
    c = xp.cos(theta)
    s = xp.sin(theta)
    return xp.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def rotation_matrix_y(theta=0.0):
    """Rotation about the Y axis."""
    xp = array_module(theta)
    c = xp.cos(theta)
    s = xp.sin(theta)
    return xp.array(
        [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def rotation_matrix_z(theta=0.0):
    """Rotation about the Z axis."""
    xp = array_module(theta)
    c = xp.cos(theta)
    s = xp.sin(theta)
    return xp.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def invert_transform(T):
    """Invert a rigid 4x4 transform (rotation + translation)."""
    xp = array_module(T)
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    top = xp.concatenate([R_inv, t_inv.reshape(3, 1)], axis=1)
    bottom = xp.array([0.0, 0.0, 0.0, 1.0]).reshape(1, 4)
    return xp.concatenate([top, bottom], axis=0)


def apply_transform(T, q):
    """World point of body-frame point ``q``: ``R @ q + t``."""
    xp = array_module(T, q)
    q = xp.asarray(q)
    d = q.shape[0]
    return T[:d, :d] @ q + T[:d, d]


def point_transform(point):
    """Translation placing the origin at *point* (z defaults to 0)."""
    xp = array_module(point)
    point = xp.asarray(point)
    z = point[2] if point.size > 2 else 0.0
    return translation_matrix(point[0], point[1], z)


def rod_between_transform(p0, p1):
    """Pose a unit rod (local -Y along the segment) from *p0* to *p1* in 3-D."""
    xp = array_module(p0, p1)
    p0 = xp.asarray(p0, dtype=float)
    p1 = xp.asarray(p1, dtype=float)
    delta = p1 - p0
    length = xp.linalg.norm(delta)

    if length < 1e-12:
        return _translation_only(xp, p0)

    y_axis = -delta / length
    reference = xp.array([0.0, 0.0, 1.0])
    if abs(xp.dot(y_axis, reference)) > 0.95:
        reference = xp.array([1.0, 0.0, 0.0])
    x_axis = xp.cross(reference, y_axis)
    x_axis = x_axis / xp.linalg.norm(x_axis)
    z_axis = xp.cross(x_axis, y_axis)
    return xp.stack(
        [
            xp.concatenate([x_axis, xp.array([p0[0]])]),
            xp.concatenate([y_axis, xp.array([p0[1]])]),
            xp.concatenate([z_axis, xp.array([p0[2]])]),
            xp.array([0.0, 0.0, 0.0, 1.0]),
        ]
    )


def single_body_tf(x, ix=0, iy=1, ith=2):
    """Planar rigid-body pose from state indices (common vehicle shortcut)."""
    return pose2d_matrix(x[ix], x[iy], x[ith])


# Internal machinery


def _translation_only(xp, p):
    z = p[2] if p.size > 2 else 0.0
    return xp.array(
        [
            [1.0, 0.0, 0.0, p[0]],
            [0.0, 1.0, 0.0, p[1]],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
