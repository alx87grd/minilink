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
placed part via :func:`collision_spheres` and
:func:`minilink.core.kinematics.apply`.
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


@dataclass(frozen=True)
class PlanarRigidBody(RobotBody):
    """
    Oriented planar body: several parts placed by one SE(2) pose.

    Every part shares the rigid pose ``(x, y, theta)``, so body-frame sphere
    offsets sweep out an oriented footprint — e.g. a rectangular car outline —
    whose collision then depends on its heading.

    Parameters
    ----------
    parts : tuple of Shape
        Body-frame collision geometry (spheres in the first pass).
    position : tuple of int
        State indices of the body-frame origin in the plane.
    heading : int
        State index of the planar heading angle.
    """

    parts: tuple
    position: tuple = (0, 1)
    heading: int = 2

    @property
    def shapes(self) -> tuple[Shape, ...]:
        return self.parts

    def body_poses(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        tx, ty = x[self.position[0]], x[self.position[1]]
        c = xp.cos(x[self.heading])
        s = xp.sin(x[self.heading])

        # SE(2) homogeneous transform shared by every rigid part
        transform = xp.stack(
            [
                xp.stack([c, -s, tx]),
                xp.stack([s, c, ty]),
                xp.asarray([0.0, 0.0, 1.0]),
            ]
        )
        return tuple(transform for _ in self.parts)


def car(length, width, *, position=(0, 1), heading=2, margin=0.0) -> PlanarRigidBody:
    """
    Return an oriented rectangular car footprint probed at its outline.

    Collision is checked at the four corners and four edge midpoints of the
    ``length × width`` box (each grown by ``margin``), placed by an SE(2) pose —
    so the heading sets how the rectangle fits between obstacles.
    """
    hl, hw = 0.5 * length, 0.5 * width
    outline = [
        (hl, hw),
        (hl, -hw),
        (-hl, hw),
        (-hl, -hw),  # corners
        (hl, 0.0),
        (-hl, 0.0),
        (0.0, hw),
        (0.0, -hw),  # edge midpoints
    ]
    parts = tuple(Sphere([px, py], margin) for px, py in outline)
    return PlanarRigidBody(parts, position=position, heading=heading)


def collision_spheres(shape: Shape) -> tuple:
    """
    Return body-frame ``(center, radius)`` probes for a body part.

    This is the only place the sphere/capsule restriction lives. A
    :class:`Sphere` yields one probe; capsules and unions are added later.
    """
    if isinstance(shape, Sphere):
        return ((shape.center, shape.radius),)
    raise NotImplementedError("Only Sphere body parts are supported for now")
