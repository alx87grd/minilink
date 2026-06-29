"""
Collision bodies for spatial scene queries.

A :class:`CollisionBody` is the moving counterpart of a workspace
:class:`~minilink.core.geometry.Shape`: it carries body-frame collision
geometry and places it in the world with one pose transform per rigid part.

Production call sites bind frameless geometry to a planner :class:`~minilink.core.system.System`
via :func:`bind` so world poses come from ``sys.tf`` — one FK pass shared with
rendering. Standalone state-indexed bodies (:func:`sphere`, :func:`car`) remain
for unit tests and scripts without a planner plant.

Collision probes use :func:`collision_spheres` and
:func:`~minilink.core.kinematics.apply`.
"""

from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module
from minilink.core.geometry import Shape, Sphere
from minilink.core.kinematics import apply

__all__ = [
    "CollisionBody",
    "TranslationBody",
    "PlanarRigidBody",
    "bind",
    "car",
    "car_outline",
    "collision_spheres",
    "disc",
    "iter_probes",
    "point",
    "point_probe",
    "sphere",
]


class CollisionBody(ABC):
    """
    Collision model: per-part body-frame geometry placed by world poses.

    Subclasses expose :attr:`shapes` (static, one per rigid part) and
    :meth:`body_poses` (one homogeneous transform per part, aligned with
    ``shapes``), or override :meth:`iter_placed_parts` directly.
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

    def iter_placed_parts(self, x, u=None, t=0.0, params=None) -> Iterator:
        """Yield ``(world_transform, body_frame_shape)`` for each collision part."""
        for shape, transform in zip(
            self.shapes, self.body_poses(x, u, t, params), strict=True
        ):
            yield transform, shape


def _normalize_shapes(geometry) -> tuple[Shape, ...]:
    if isinstance(geometry, Shape):
        return (geometry,)
    return tuple(geometry)


def disc(radius: float) -> Sphere:
    """Body-frame disc/sphere probe at the origin (frameless geometry)."""
    return Sphere(np.zeros(2), float(radius))


def point_probe() -> Sphere:
    """Body-frame radius-zero point probe at the origin."""
    return Sphere(np.zeros(2), 0.0)


def car_outline(length: float, width: float, *, margin: float = 0.0) -> tuple[Sphere, ...]:
    """
    Body-frame rectangular car footprint: corners and edge midpoints.

    Each probe is a :class:`Sphere` offset in the body frame. Attach to a
    planner plant with :func:`bind` and ``frame="body"`` (default).
    """
    hl, hw = 0.5 * length, 0.5 * width
    outline = [
        (hl, hw),
        (hl, -hw),
        (-hl, hw),
        (-hl, -hw),
        (hl, 0.0),
        (-hl, 0.0),
        (0.0, hw),
        (0.0, -hw),
    ]
    return tuple(Sphere([px, py], margin) for px, py in outline)


def bind(sys, spec, *, frame: str = "body") -> CollisionBody:
    """
    Bind frameless collision geometry to a planner plant's ``tf`` dict.

    Single-frame (default ``frame="body"``)::

        body = bind(sys_mpc, disc(0.25))
        body = bind(sys_mpc, car_outline(2.4, 0.2, margin=0.05))

    Multi-frame — pass a **list** of ``(frame_key, geometry)`` pairs::

        body = bind(sys, [("body", car_outline(2.4, 0.2)), ("link3", disc(0.15))])
    """
    if (
        isinstance(spec, list)
        and spec
        and isinstance(spec[0], (list, tuple))
        and len(spec[0]) == 2
        and isinstance(spec[0][0], str)
    ):
        bindings = tuple((frame_key, _normalize_shapes(geometry)) for frame_key, geometry in spec)
    else:
        bindings = ((frame, _normalize_shapes(spec)),)
    return _BoundCollisionBody(sys, bindings)


class _BoundCollisionBody(CollisionBody):
    """Collision body whose world poses come from ``sys.tf`` frame lookup."""

    def __init__(self, sys, bindings):
        self._sys = sys
        self._bindings = bindings

    def _resolve_u(self, u):
        if u is not None:
            return u
        m = getattr(self._sys, "m", 0)
        if m:
            return np.zeros(m)
        return None

    @property
    def shapes(self) -> tuple[Shape, ...]:
        return tuple(shape for _, shapes in self._bindings for shape in shapes)

    def body_poses(self, x, u=None, t=0.0, params=None):
        frames = self._sys.tf(x, self._resolve_u(u), t, params)
        poses = []
        for frame_key, shapes in self._bindings:
            poses.extend([frames[frame_key]] * len(shapes))
        return tuple(poses)

    def iter_placed_parts(self, x, u=None, t=0.0, params=None):
        frames = self._sys.tf(x, self._resolve_u(u), t, params)
        for frame_key, shapes in self._bindings:
            transform = frames[frame_key]
            for shape in shapes:
                yield transform, shape


def iter_probes(body, x, u=None, t=0.0, params=None):
    """Yield ``(world_center, radius)`` for each body-frame collision sphere."""
    for transform, shape in body.iter_placed_parts(x, u, t, params):
        for center, radius in collision_spheres(shape):
            yield apply(transform, center), radius


@dataclass(frozen=True)
class TranslationBody(CollisionBody):
    """
    Single-part body placed by translating its geometry to state coordinates.

    Prefer :func:`bind` with :func:`disc` when a planner :class:`~minilink.core.system.System`
    is available. This class remains for tests and scripts without a plant.
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

        top = xp.concatenate([xp.eye(d), p.reshape(d, 1)], axis=1)
        bottom = xp.concatenate([xp.zeros(d), xp.ones(1)]).reshape(1, d + 1)
        return (xp.concatenate([top, bottom], axis=0),)


def sphere(radius, *, position=(0, 1)) -> TranslationBody:
    """State-indexed sphere body (deprecated for planner call sites — use :func:`bind`)."""
    return TranslationBody(Sphere(np.zeros(len(position)), radius), position)


def point(*, position=(0, 1)) -> TranslationBody:
    """State-indexed point body (deprecated for planner call sites — use :func:`bind`)."""
    return TranslationBody(Sphere(np.zeros(len(position)), 0.0), position)


@dataclass(frozen=True)
class PlanarRigidBody(CollisionBody):
    """
    Oriented planar body: several parts placed by one SE(2) pose from state indices.

    Prefer :func:`bind` with :func:`car_outline` when a planner plant is available.
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
    State-indexed oriented car footprint (deprecated for planner call sites — use :func:`bind`).

    Same probe layout as :func:`car_outline`, with pose from ``position``/``heading`` indices.
    """
    return PlanarRigidBody(car_outline(length, width, margin=margin), position=position, heading=heading)


def collision_spheres(shape: Shape) -> tuple:
    """
    Return body-frame ``(center, radius)`` probes for a body part.

    This is the only place the sphere/capsule restriction lives. A
    :class:`Sphere` yields one probe; capsules and unions are added later.
    """
    if isinstance(shape, Sphere):
        return ((shape.center, shape.radius),)
    raise NotImplementedError("Only Sphere body parts are supported for now")
