"""
Robot collision bodies for spatial scene queries.

A :class:`RobotBody` is the moving counterpart of a workspace
:class:`~minilink.core.geometry.Shape`: it carries body-frame collision
geometry and places it in the world with one pose transform per rigid part.
The contract is two methods — ``shapes`` (static geometry, one per part) and
``body_poses(x, u, t, params)`` (a homogeneous transform per part, aligned
one-to-one with ``shapes``). A disc is a single part; a manipulator is many,
its poses coming from forward kinematics. This mirrors the ``System``
visualization contract (static geometry plus aligned transforms) but with
collision-grade geometry, and it never imports any graphics.

Collision against a scene uses world ``(center, radius)`` probes from each
placed part via :func:`collision_spheres` and :func:`apply_transform`.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module
from minilink.core.geometry import Shape, Sphere

# Public API


class RobotBody(ABC):
    """
    Collision model: per-part body-frame geometry placed by world poses.

    Subclasses expose :attr:`shapes` (static, one per rigid part) and
    :meth:`body_poses` (one homogeneous transform per part, aligned with
    ``shapes``). The state-to-pose map is general forward kinematics, not an
    assumption that the state *is* the pose.
    """

    @property
    @abstractmethod
    def shapes(self) -> tuple[Shape, ...]:
        """Body-frame collision geometry, one :class:`Shape` per part."""
        ...

    @abstractmethod
    def body_poses(self, x, u=None, t=0.0, params=None):
        """Return one homogeneous transform per part, aligned with :attr:`shapes`."""
        ...


@dataclass(frozen=True)
class TranslationBody(RobotBody):
    """
    Single-part body placed by translating its geometry to state coordinates.

    The pose is a pure translation with identity orientation — exact for
    rotation-invariant geometry such as spheres, and valid in any workspace
    dimension. The ``position`` indices pick the workspace coordinates out of a
    state of any size, and their count sets the workspace dimension. Oriented
    bodies (SE(2)/SE(3) rotation) are a later addition.

    Parameters
    ----------
    shape : Shape
        Body-frame collision geometry (a :class:`Sphere` in the first pass).
    position : tuple of int
        State indices giving the body-frame origin in the world.
    """

    shape: Shape
    position: tuple = (0, 1)

    @property
    def shapes(self) -> tuple[Shape, ...]:
        return (self.shape,)

    def body_poses(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        p = xp.stack([x[i] for i in self.position])
        d = p.shape[0]

        # homogeneous transform of a pure translation: [[I, p], [0, 1]]
        top = xp.concatenate([xp.eye(d), p.reshape(d, 1)], axis=1)
        bottom = xp.concatenate([xp.zeros(d), xp.ones(1)]).reshape(1, d + 1)
        return (xp.concatenate([top, bottom], axis=0),)


def sphere(radius, *, position=(0, 1)) -> TranslationBody:
    """Return a sphere body of the given radius (a disc in 2-D)."""
    return TranslationBody(Sphere(np.zeros(len(position)), radius), position)


def point(*, position=(0, 1)) -> TranslationBody:
    """Return a radius-zero point body."""
    return TranslationBody(Sphere(np.zeros(len(position)), 0.0), position)


def apply_transform(T, q):
    """Return the world point of body-frame point ``q`` under transform ``T``."""
    xp = array_module(T, q)
    q = xp.asarray(q)
    d = q.shape[0]

    # rotate then translate: world = R @ q + t
    return T[:d, :d] @ q + T[:d, d]


def collision_spheres(shape: Shape) -> tuple:
    """
    Return body-frame ``(center, radius)`` probes for a body part.

    This is the only place the sphere/capsule restriction lives. A
    :class:`Sphere` yields one probe; capsules and unions are added later.
    """
    if isinstance(shape, Sphere):
        return ((shape.center, shape.radius),)
    raise NotImplementedError("Only Sphere body parts are supported for now")
