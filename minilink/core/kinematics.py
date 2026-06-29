"""Rigid-body orientation and pose algebra (SO(3) / SE(3)).

A native-array, JAX-functional transform toolkit — the rigid-body peer of
:mod:`minilink.core.sets` and :mod:`minilink.core.costs`. Like them it is pure
functions over arrays: no classes, no rendering, no solver, no knowledge of
:class:`~minilink.core.system.System`. Anything that needs "how is this body
oriented" or "where is this body in the world" imports it.

Two layers mirror the course notes (*Orientation* and *Poses*):

- an **orientation layer** of 3×3 rotations ``R`` (:func:`Rx` / :func:`Ry` /
  :func:`Rz`; the inverse is simply ``R.T``);
- a **pose layer** of 4×4 homogeneous transforms ``T`` built on it
  (:func:`SE3`, :func:`SE2`, :func:`translation`, :func:`identity`, :func:`inv`,
  :func:`apply`).

Matrices are assembled with ``xp.array`` / ``xp.concatenate`` via
``xp = array_module(...)`` and **never** by in-place index assignment, so a ``tf``
written on top of this module traces under ``jax.jit`` / ``vmap`` and the same
toolkit feeds both rendering (graphical ``tf``) and collision probing
(``planning/spatial``).

Composition is the matrix product ``@`` (``W_T_C = W_T_B @ B_T_C``); tag locals
with frame keys so the algebra reads like the notes.
"""

import numpy as np

from minilink.core.backends import array_module

# Orientation layer — 3x3 rotations R (SO(3))


def Rx(theta):
    """3×3 rotation about the X axis by ``theta`` (radians)."""
    xp = array_module(theta)
    c, s = xp.cos(theta), xp.sin(theta)
    one, zero = xp.ones_like(c), xp.zeros_like(c)

    # fmt: off
    return xp.array([
        [ one, zero, zero],
        [zero,    c,   -s],
        [zero,    s,    c],
    ])
    # fmt: on


def Ry(theta):
    """3×3 rotation about the Y axis by ``theta`` (radians)."""
    xp = array_module(theta)
    c, s = xp.cos(theta), xp.sin(theta)
    one, zero = xp.ones_like(c), xp.zeros_like(c)

    # fmt: off
    return xp.array([
        [   c, zero,    s],
        [zero,  one, zero],
        [  -s, zero,    c],
    ])
    # fmt: on


def Rz(theta):
    """3×3 rotation about the Z axis by ``theta`` (radians); the planar rotation."""
    xp = array_module(theta)
    c, s = xp.cos(theta), xp.sin(theta)
    one, zero = xp.ones_like(c), xp.zeros_like(c)

    # fmt: off
    return xp.array([
        [   c,   -s, zero],
        [   s,    c, zero],
        [zero, zero,  one],
    ])
    # fmt: on


# Pose layer — 4x4 homogeneous transforms T (SE(3))


def SE3(R, p=0.0):
    """4×4 pose ``[[R, p], [0, 1]]`` from a 3×3 rotation ``R`` and translation ``p``.

    ``p`` defaults to the origin; pass a length-3 vector to place the frame.
    """
    xp = array_module(R, p)
    R = xp.asarray(R)
    p = xp.zeros(3) if xp.ndim(p) == 0 else xp.asarray(p)

    top = xp.concatenate([R, p.reshape(3, 1)], axis=1)
    bottom = xp.asarray([[0.0, 0.0, 0.0, 1.0]])
    return xp.concatenate([top, bottom], axis=0)


def SE2(x, y, theta):
    """4×4 planar pose: rotation ``theta`` about Z plus translation ``(x, y)``.

    Equivalent to ``SE3(Rz(theta), [x, y, 0])``, written out for readability.
    """
    xp = array_module(theta, x, y)
    c, s = xp.cos(theta), xp.sin(theta)
    one, zero = xp.ones_like(c), xp.zeros_like(c)
    x, y = x * one, y * one

    # fmt: off
    return xp.array([
        [   c,   -s, zero,    x],
        [   s,    c, zero,    y],
        [zero, zero,  one, zero],
        [zero, zero, zero,  one],
    ])
    # fmt: on


def translation(dx=0.0, dy=0.0, dz=0.0):
    """4×4 pure translation by ``(dx, dy, dz)`` (identity orientation)."""
    xp = array_module(dx, dy, dz)
    one, zero = xp.ones(()), xp.zeros(())
    dx, dy, dz = dx * one, dy * one, dz * one

    # fmt: off
    return xp.array([
        [ one, zero, zero,   dx],
        [zero,  one, zero,   dy],
        [zero, zero,  one,   dz],
        [zero, zero, zero,  one],
    ])
    # fmt: on


def identity(xp=None):
    """4×4 identity pose (the world / root frame). ``xp`` selects the backend."""
    if xp is None:
        xp = np
    return xp.eye(4)


def inv(T):
    """Rigid inverse of a 4×4 pose: ``inv(A_T_B) = B_T_A = [[Rᵀ, −Rᵀp], [0, 1]]``.

    Cheaper and more stable than a generic inverse for a rigid transform.
    """
    xp = array_module(T)
    R = T[:3, :3]
    p = T[:3, 3]

    R_inv = R.T
    top = xp.concatenate([R_inv, (-R_inv @ p).reshape(3, 1)], axis=1)
    bottom = xp.asarray([[0.0, 0.0, 0.0, 1.0]])
    return xp.concatenate([top, bottom], axis=0)


def apply(T, pts):
    """Transform body-frame point(s) ``pts`` into the reference frame of ``T``.

    ``pts`` is a single point ``(d,)`` or a batch ``(N, d)``. The leading ``d×d``
    rotation block and translation column are used, so the same call works for a
    4×4 SE(3) pose with 3-D points and a 3×3 SE(2) pose with planar points.
    """
    xp = array_module(T, pts)
    pts = xp.asarray(pts)
    d = pts.shape[-1]

    R = T[:d, :d]
    p = T[:d, d]
    return pts @ R.T + p


if __name__ == "__main__":
    # world <- body <- wheel chain, the canonical tf composition
    W_T_body = SE2(1.0, 2.0, np.pi / 2)
    body_T_wheel = SE2(0.5, 0.0, 0.0)
    W_T_wheel = W_T_body @ body_T_wheel
    print("wheel origin in world:", apply(W_T_wheel, np.zeros(3)))
