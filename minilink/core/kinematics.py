"""Rigid-body orientation and pose algebra: rotations ``R`` (SO(3)) and poses ``T`` (SE(3))."""

import numpy as np

from minilink.core.backends import array_module

###############################################################################
# Public API
###############################################################################

# Pure functions over native arrays, assembled with ``xp.concatenate`` so a ``tf`` built
# on them traces under JAX. Composition is the matrix product, ``W_T_C = W_T_B @ B_T_C``;
# tag locals with frame keys so the algebra reads like the notes.


# Orientation layer: 3×3 rotations R ∈ SO(3)


def Rx(theta):
    """3×3 rotation about the X axis by ``theta`` (radians)."""
    xp = array_module(theta)

    c = xp.cos(theta)
    s = xp.sin(theta)
    one = xp.ones_like(c)
    zero = xp.zeros_like(c)

    # fmt: off
    R = xp.array([
        [ one, zero, zero],
        [zero,    c,   -s],
        [zero,    s,    c],
    ])
    # fmt: on

    return R


def Ry(theta):
    """3×3 rotation about the Y axis by ``theta`` (radians)."""
    xp = array_module(theta)

    c = xp.cos(theta)
    s = xp.sin(theta)
    one = xp.ones_like(c)
    zero = xp.zeros_like(c)

    # fmt: off
    R = xp.array([
        [   c, zero,    s],
        [zero,  one, zero],
        [  -s, zero,    c],
    ])
    # fmt: on

    return R


def Rz(theta):
    """3×3 rotation about the Z axis by ``theta`` (radians); the planar rotation."""
    xp = array_module(theta)

    c = xp.cos(theta)
    s = xp.sin(theta)
    one = xp.ones_like(c)
    zero = xp.zeros_like(c)

    # fmt: off
    R = xp.array([
        [   c,   -s, zero],
        [   s,    c, zero],
        [zero, zero,  one],
    ])
    # fmt: on

    return R


# Pose layer: 4×4 homogeneous transforms T ∈ SE(3)


def SE3(R, p=0.0):
    """4×4 pose ``[[R, p], [0, 1]]`` from a 3×3 rotation ``R`` and translation ``p``.

    ``p`` defaults to the origin; pass a length-3 vector to place the frame.
    """
    xp = array_module(R, p)

    R = xp.asarray(R)
    p = xp.zeros(3) if xp.ndim(p) == 0 else xp.asarray(p)

    # T = [[R, p], [0, 1]]
    top = xp.concatenate([R, p.reshape(3, 1)], axis=1)
    bottom = xp.asarray([[0.0, 0.0, 0.0, 1.0]])
    T = xp.concatenate([top, bottom], axis=0)

    return T


def SE2(x, y, theta):
    """4×4 planar pose: rotation ``theta`` about Z plus translation ``(x, y)``.

    Equivalent to ``SE3(Rz(theta), [x, y, 0])``, written out for readability.
    """
    xp = array_module(theta, x, y)

    c = xp.cos(theta)
    s = xp.sin(theta)
    one = xp.ones_like(c)
    zero = xp.zeros_like(c)

    # The translation on the rotation's shape and backend
    x = x * one
    y = y * one

    # fmt: off
    T = xp.array([
        [   c,   -s, zero,    x],
        [   s,    c, zero,    y],
        [zero, zero,  one, zero],
        [zero, zero, zero,  one],
    ])
    # fmt: on

    return T


def translation(dx=0.0, dy=0.0, dz=0.0):
    """4×4 pure translation by ``(dx, dy, dz)`` (identity orientation)."""
    xp = array_module(dx, dy, dz)

    one = xp.ones(())
    zero = xp.zeros(())

    # The offsets as 0-d arrays of the backend
    dx = dx * one
    dy = dy * one
    dz = dz * one

    # fmt: off
    T = xp.array([
        [ one, zero, zero,   dx],
        [zero,  one, zero,   dy],
        [zero, zero,  one,   dz],
        [zero, zero, zero,  one],
    ])
    # fmt: on

    return T


def identity(xp=None):
    """4×4 identity pose (the world / root frame). ``xp`` selects the backend."""
    if xp is None:
        xp = np

    T = xp.eye(4)

    return T


def inv(T):
    """Rigid inverse of a 4×4 pose: ``inv(A_T_B) = B_T_A = [[Rᵀ, −Rᵀp], [0, 1]]``.

    Cheaper and more stable than a generic inverse for a rigid transform.
    """
    xp = array_module(T)

    R = T[:3, :3]
    p = T[:3, 3]

    # T⁻¹ = [[Rᵀ, −Rᵀ p], [0, 1]]
    R_inv = R.T
    p_inv = -R_inv @ p

    top = xp.concatenate([R_inv, p_inv.reshape(3, 1)], axis=1)
    bottom = xp.asarray([[0.0, 0.0, 0.0, 1.0]])
    T_inv = xp.concatenate([top, bottom], axis=0)

    return T_inv


def apply(T, pts):
    """Transform body-frame point(s) ``pts`` into the reference frame of ``T``.

    ``pts`` is a single point ``(d,)`` or a batch ``(N, d)``. The leading ``d×d``
    rotation block and translation column are used. A 4×4 pose with 2-D points
    uses the planar embed (translation in column 3); a ``(d+1)×(d+1)`` matrix
    uses the standard homogeneous column ``d``.
    """
    xp = array_module(T, pts)

    pts = xp.asarray(pts)
    d = pts.shape[-1]
    n = int(T.shape[0])

    # The rotation block, and the translation column (d, or the last in a planar embed)
    R = T[:d, :d]
    p = T[:d, d] if n == d + 1 else T[:d, n - 1]

    # p_ref = R p_body + p, one row per point
    pts_ref = pts @ R.T + p

    return pts_ref


if __name__ == "__main__":
    # The world ← body ← wheel chain, the canonical tf composition
    W_T_body = SE2(1.0, 2.0, np.pi / 2)
    body_T_wheel = SE2(0.5, 0.0, 0.0)
    W_T_wheel = W_T_body @ body_T_wheel
    print("wheel origin in world:", apply(W_T_wheel, np.zeros(3)))
