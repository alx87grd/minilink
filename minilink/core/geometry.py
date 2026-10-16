"""Workspace solids by their signed distance ``sdf(p)``: negative inside, zero on the surface."""

import functools
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module
from minilink.core.sets import require_box_bounds, require_nonnegative_radius

###############################################################################
# Public API
###############################################################################


class Shape(ABC):
    """
    Base class for workspace solids defined by a signed-distance function.

    A point ``p`` is inside the solid when :meth:`sdf` is negative. The solid
    may depend on time and optional parameters, so the generic signature is
    ``sdf(p, t, params)``.
    """

    @property
    @abstractmethod
    def dim(self) -> int:
        """Workspace dimension (2 or 3)."""
        ...

    @abstractmethod
    def sdf(self, p, t=0.0, params=None):
        """Return the signed distance from ``p`` to the shape surface."""
        ...

    def contains(self, p, t: float = 0.0, params=None) -> bool:
        """Return ``True`` when ``p`` lies inside the solid (``sdf < 0``)."""
        p_arr = np.asarray(p, dtype=float).reshape(-1)
        sdf = self.sdf(p_arr, t, params)
        inside = bool(np.asarray(sdf) < 0.0)

        return inside

    def __or__(self, other: "Shape") -> "Union":
        """Return the union of this shape and ``other`` (``sdf = min``)."""
        return Union((self, other))

    def inflate(self, radius: float) -> "Inflated":
        """Return this shape grown by ``radius`` (Minkowski clearance)."""
        return Inflated(self, float(radius))


@dataclass(frozen=True)
class Sphere(Shape):
    """
    Ball ``||p - center|| <= radius`` (a disc in 2-D, a sphere in 3-D).

    Parameters
    ----------
    center : array_like
        Center with shape ``(dim,)``.
    radius : float
        Nonnegative radius.
    """

    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float).reshape(-1).copy()
        radius = float(self.radius)
        require_nonnegative_radius(radius)

        object.__setattr__(self, "center", center)
        object.__setattr__(self, "radius", radius)

    @property
    def dim(self) -> int:
        return int(self.center.size)

    def sdf(self, p, t=0.0, params=None):
        xp = array_module(p)

        center = self.center
        radius = self.radius

        sdf = xp.linalg.norm(p - center) - radius

        return sdf


@dataclass(frozen=True)
class Box(Shape):
    """
    Axis-aligned box ``lower <= p <= upper``.

    Parameters
    ----------
    lower, upper : array_like
        Opposite corners with shape ``(dim,)``.
    """

    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self) -> None:
        lower = np.asarray(self.lower, dtype=float).reshape(-1).copy()
        upper = np.asarray(self.upper, dtype=float).reshape(-1).copy()
        require_box_bounds(lower, upper)

        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def dim(self) -> int:
        return int(self.lower.size)

    def sdf(self, p, t=0.0, params=None):
        xp = array_module(p)

        lower = self.lower
        upper = self.upper

        # dᵢ = max(lowerᵢ − pᵢ, pᵢ − upperᵢ), the signed gap to the faces of axis i
        d = xp.maximum(lower - p, p - upper)

        # sdf = ‖max(d, 0)‖ + min(maxᵢ dᵢ, 0): the exterior distance plus the interior depth
        outside = xp.linalg.norm(xp.maximum(d, 0.0))
        inside = xp.minimum(xp.max(d), 0.0)
        sdf = outside + inside

        return sdf


@dataclass(frozen=True)
class Union(Shape):
    """
    Union of several shapes, with ``sdf = min`` over the members.

    This is the obstacle-field primitive: a point is occupied when it lies
    inside any member shape.
    """

    shapes: tuple

    def __post_init__(self) -> None:
        shapes = tuple(self.shapes)
        require_members(shapes)

        object.__setattr__(self, "shapes", shapes)

    @property
    def dim(self) -> int:
        return self.shapes[0].dim

    def sdf(self, p, t=0.0, params=None):
        xp = array_module(p)

        shapes = self.shapes

        # sdf = minᵢ sdfᵢ(p), the nearest member surface, reduced with xp.minimum so it
        # traces under JAX (Python min() would compare traced values and break jit)
        sdf = functools.reduce(
            xp.minimum, (shape.sdf(p, t, params) for shape in shapes)
        )

        return sdf


@dataclass(frozen=True)
class Inflated(Shape):
    """
    Shape grown outward by ``radius`` (Minkowski sum with a ball).

    Subtracting the radius from a base signed distance gives the clearance for
    a disc-shaped robot, or a safety margin around a hard obstacle.
    """

    base: Shape
    radius: float

    @property
    def dim(self) -> int:
        return self.base.dim

    def sdf(self, p, t=0.0, params=None):
        base = self.base
        radius = self.radius

        # sdf = sdf_base(p) − r: every surface moves outward by r
        sdf = base.sdf(p, t, params) - radius

        return sdf


###############################################################################
# Internal machinery
###############################################################################


def require_members(shapes):
    """Refuse a union of no shapes."""
    if not shapes:
        raise ValueError("Union requires at least one shape")


if __name__ == "__main__":
    disc = Sphere(center=[0.0, 0.0], radius=1.0)
    wall = Box(lower=[2.0, -1.0], upper=[3.0, 1.0])
    field = disc | wall

    print("disc.sdf([0, 0]) =", disc.sdf(np.array([0.0, 0.0])))  # -1.0 inside
    print("disc.sdf([2, 0]) =", disc.sdf(np.array([2.0, 0.0])))  # +1.0 outside
    print("field.sdf([2.5, 0]) =", field.sdf(np.array([2.5, 0.0])))  # inside wall
    print("field.contains([2.5, 0]) =", field.contains([2.5, 0.0]))
