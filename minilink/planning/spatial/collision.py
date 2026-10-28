"""Collision bodies: body-frame geometry placed in the world by one pose per rigid part."""

from abc import ABC, abstractmethod
from collections.abc import Iterator

import numpy as np

from minilink.core.geometry import Shape, Sphere
from minilink.core.kinematics import apply

__all__ = [
    "CollisionBody",
    "mount",
    "car_outline",
    "collision_spheres",
    "disc",
    "iter_probes",
    "point_probe",
]


###############################################################################
# Public API
###############################################################################


class CollisionBody(ABC):
    """
    Collision model: per-part body-frame geometry placed by world poses.

    The moving counterpart of a workspace :class:`~minilink.core.geometry.Shape`:
    it carries body-frame collision geometry and places it in the world with
    one pose transform per rigid part.

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


def disc(radius: float) -> Sphere:
    """Body-frame disc/sphere probe at the origin (frameless geometry)."""
    return Sphere(np.zeros(2), float(radius))


def point_probe() -> Sphere:
    """Body-frame radius-zero point probe at the origin."""
    return Sphere(np.zeros(2), 0.0)


def car_outline(
    length: float, width: float, *, margin: float = 0.0
) -> tuple[Sphere, ...]:
    """
    Body-frame rectangular car footprint: corners and edge midpoints.

    Each probe is a :class:`Sphere` offset in the body frame. Attach to a
    planner plant with :func:`mount` and ``frame="body"`` (default).
    """
    # Half length and half width of the footprint
    hl = 0.5 * length
    hw = 0.5 * width

    # The four corners, then the midpoints of the four edges
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
    probes = tuple(Sphere([px, py], margin) for px, py in outline)

    return probes


def mount(geometry, *, on, frame: str = "body") -> CollisionBody:
    """
    Mount frameless collision geometry on a plant, at a frame of its ``tf`` dict.

    The world poses then come from ``on.tf``: one forward-kinematics pass,
    shared with rendering.

    Single-frame (default ``frame="body"``)::

        body = mount(disc(0.25), on=sys_mpc)
        body = mount(car_outline(2.4, 0.2, margin=0.05), on=sys_mpc)

    Multi-frame — pass a **list** of ``(frame_key, geometry)`` pairs::

        body = mount([("body", car_outline(2.4, 0.2)), ("link3", disc(0.15))], on=sys)
    """
    if (
        isinstance(geometry, list)
        and geometry
        and isinstance(geometry[0], (list, tuple))
        and len(geometry[0]) == 2
        and isinstance(geometry[0][0], str)
    ):
        # Several frames: one (frame_key, geometry) pair each
        bindings = tuple(
            (frame_key, normalize_shapes(shapes)) for frame_key, shapes in geometry
        )
    else:
        # One frame carries all the geometry
        bindings = ((frame, normalize_shapes(geometry)),)

    return BoundCollisionBody(on, bindings)


def iter_probes(body, x, u=None, t=0.0, params=None):
    """
    Yield ``(world_center, radius)`` for each body-frame collision sphere.

    The spheres come from :func:`collision_spheres`, placed in the world by
    :func:`~minilink.core.kinematics.apply`.
    """
    for transform, shape in body.iter_placed_parts(x, u, t, params):
        for center, radius in collision_spheres(shape):
            # c_world = T c_body
            world_center = apply(transform, center)

            yield world_center, radius


def collision_spheres(shape: Shape) -> tuple:
    """
    Return body-frame ``(center, radius)`` probes for a body part.

    This is the only place the sphere restriction lives: a :class:`Sphere`
    yields one probe, and any other shape is refused.
    """
    if isinstance(shape, Sphere):
        return ((shape.center, shape.radius),)

    raise NotImplementedError("Only Sphere body parts are supported")


###############################################################################
# Internal machinery
###############################################################################


def normalize_shapes(geometry) -> tuple[Shape, ...]:
    """One shape, or several, as a tuple of shapes."""
    if isinstance(geometry, Shape):
        return (geometry,)

    return tuple(geometry)


class BoundCollisionBody(CollisionBody):
    """Collision body whose world poses come from ``sys.tf`` frame lookup."""

    def __init__(self, sys, bindings):
        self._sys = sys
        self._bindings = bindings

    @property
    def shapes(self) -> tuple[Shape, ...]:
        return tuple(shape for _, shapes in self._bindings for shape in shapes)

    def body_poses(self, x, u=None, t=0.0, params=None):
        frames = self._sys.tf(x, self.resolve_u(u), t, params)

        # Each part takes the pose of the frame it is bound to
        poses = []
        for frame_key, shapes in self._bindings:
            poses.extend([frames[frame_key]] * len(shapes))

        return tuple(poses)

    def iter_placed_parts(self, x, u=None, t=0.0, params=None):
        frames = self._sys.tf(x, self.resolve_u(u), t, params)

        for frame_key, shapes in self._bindings:
            transform = frames[frame_key]
            for shape in shapes:
                yield transform, shape

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_u(self, u):
        """The input given, else a zero command of the system's size, else ``None``."""
        if u is not None:
            return u
        m = getattr(self._sys, "m", 0)
        if m:
            return np.zeros(m)

        return None
