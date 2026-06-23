"""
Robot collision bodies for environment queries.

A :class:`RobotBody` is the moving counterpart of a workspace
:class:`~minilink.core.geometry.Shape`: it carries body-frame collision
geometry and places it in the world with one pose transform per rigid part.
The contract is two methods — ``shapes`` (static geometry, one per part) and
``body_poses(x, u, t, params)`` (a homogeneous transform per part, aligned
one-to-one with ``shapes``). A disc is a single part; a manipulator is many,
its poses coming from forward kinematics. This mirrors the ``System``
visualization contract (static geometry plus aligned transforms) but with
collision-grade geometry, and it never imports any graphics.

Collision against an environment reduces each part to body-frame
``(center, radius)`` probes through :func:`collision_spheres`, so the cheap,
exact query ``obstacle.sdf(world_center) - radius`` is all that is needed. The
first pass supports a single :class:`~minilink.core.geometry.Sphere` part.
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
class PlanarBody(RobotBody):
    """
    Single-part planar body placed by translation and an optional heading.

    Parameters
    ----------
    shape : Shape
        Body-frame collision geometry (a :class:`Sphere` in the first pass).
    position : tuple of int
        State indices of the body-frame origin in the world plane.
    heading : int, optional
        State index of the planar rotation angle. ``None`` leaves the body
        unrotated (sufficient for a centered disc).
    """

    shape: Shape
    position: tuple = (0, 1)
    heading: int | None = None

    @property
    def shapes(self) -> tuple[Shape, ...]:
        return (self.shape,)

    def body_poses(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        tx = x[self.position[0]]
        ty = x[self.position[1]]
        c = 1.0 if self.heading is None else xp.cos(x[self.heading])
        s = 0.0 if self.heading is None else xp.sin(x[self.heading])

        # homogeneous SE(2) transform of the body frame in the world
        T = xp.stack(
            [
                xp.stack([c, -s, tx]),
                xp.stack([s, c, ty]),
                xp.asarray([0.0, 0.0, 1.0]),
            ]
        )
        return (T,)


def disc(radius, *, position=(0, 1), dim=2) -> PlanarBody:
    """Return a single-sphere planar body of the given radius."""
    return PlanarBody(Sphere(np.zeros(dim), radius), position=position)


def point(*, position=(0, 1), dim=2) -> PlanarBody:
    """Return a radius-zero planar body (a point robot)."""
    return PlanarBody(Sphere(np.zeros(dim), 0.0), position=position)


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
