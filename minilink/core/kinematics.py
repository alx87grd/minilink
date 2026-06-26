"""
Rigid-body pose builders for forward kinematics.

A pose is a 4x4 homogeneous transform placing a body frame in the world. These
builders follow the hybrid NumPy/JAX idiom (``xp = array_module(...)``) so the
same code runs under NumPy for rendering and traces under JAX for collision /
trajectory optimization. They are the shared foundation under
:meth:`minilink.core.system.System.frames`: graphics and collision both place
geometry with the *same* poses, never recomputing forward kinematics twice.

All transforms are global (world-frame) and full 4x4 SE(3); planar callers use
the upper-left 2x2 rotation and the ``[:2, 3]`` translation.
"""

from minilink.core.backends import array_module

# Public API


def se3_translation(x, y=0.0, z=0.0):
    """
    Return the 4x4 transform of a pure translation ``(x, y, z)``.

    Parameters
    ----------
    x, y, z : float or array
        World translation components. Any may be a JAX scalar so the result
        traces under ``jit`` / ``vmap``.

    Returns
    -------
    ndarray
        4x4 homogeneous transform with identity orientation.
    """
    xp = array_module(x, y, z)
    one, zero = xp.ones(()), xp.zeros(())
    x, y, z = xp.asarray(x), xp.asarray(y), xp.asarray(z)

    # homogeneous translation: [[I, p], [0, 1]]
    return xp.stack(
        [
            xp.stack([one, zero, zero, x]),
            xp.stack([zero, one, zero, y]),
            xp.stack([zero, zero, one, z]),
            xp.stack([zero, zero, zero, one]),
        ]
    )


def se2(x, y, theta):
    """
    Return the 4x4 transform of a planar pose ``(x, y, theta)`` in the XY plane.

    Parameters
    ----------
    x, y : float or array
        World translation in the plane.
    theta : float or array
        Heading angle about the world Z axis, in radians.

    Returns
    -------
    ndarray
        4x4 homogeneous transform: Z-rotation ``theta`` then translation.
    """
    xp = array_module(x, y, theta)
    c, s = xp.cos(theta), xp.sin(theta)
    one, zero = xp.ones(()), xp.zeros(())
    x, y = xp.asarray(x), xp.asarray(y)

    # planar rigid pose embedded in SE(3)
    return xp.stack(
        [
            xp.stack([c, -s, zero, x]),
            xp.stack([s, c, zero, y]),
            xp.stack([zero, zero, one, zero]),
            xp.stack([zero, zero, zero, one]),
        ]
    )


if __name__ == "__main__":
    import numpy as np

    print(se3_translation(1.0, 2.0))
    print(se2(1.0, 2.0, np.pi / 2))
