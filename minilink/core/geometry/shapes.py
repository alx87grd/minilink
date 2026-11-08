"""Workspace solids by their signed distance ``sdf(p)``: negative inside, zero on the surface."""

import functools
from abc import ABC, abstractmethod

import numpy as np

from minilink.core.backends import array_module
from minilink.core.params import held_params, member_keys, own_params, param_view
from minilink.core.sets import require_box_bounds, require_nonnegative_radius

###############################################################################
# Public API
###############################################################################


class Shape(ABC):
    """
    Base class for workspace solids defined by a signed-distance function.

    A point ``p`` is inside the solid when :meth:`sdf` is negative. The solid
    may depend on time and optional parameters, so the generic signature is
    ``sdf(p, t, params)``. ``params`` is the shape's own numbers (``{}`` for a
    shape with none): ``params=None`` reads them, a tree passed instead moves
    the shape.
    """

    params = own_params()

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

    def __init__(self, center, radius):
        center = np.asarray(center, dtype=float).reshape(-1).copy()
        radius = float(radius)
        require_nonnegative_radius(radius)

        self.params = {"center": center, "radius": radius}

    center = param_view("center")
    radius = param_view("radius")

    @property
    def dim(self) -> int:
        return int(np.size(self.center))

    def sdf(self, p, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(p)

        center = params["center"]
        radius = params["radius"]

        sdf = xp.linalg.norm(p - center) - radius

        return sdf


class Box(Shape):
    """
    Axis-aligned box ``lower <= p <= upper``.

    Parameters
    ----------
    lower, upper : array_like
        Opposite corners with shape ``(dim,)``.
    """

    def __init__(self, lower, upper):
        lower = np.asarray(lower, dtype=float).reshape(-1).copy()
        upper = np.asarray(upper, dtype=float).reshape(-1).copy()
        require_box_bounds(lower, upper)

        self.params = {"lower": lower, "upper": upper}

    lower = param_view("lower")
    upper = param_view("upper")

    @property
    def dim(self) -> int:
        return int(np.size(self.lower))

    def sdf(self, p, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(p)

        lower = params["lower"]
        upper = params["upper"]

        # dᵢ = max(lowerᵢ − pᵢ, pᵢ − upperᵢ), the signed gap to the faces of axis i
        d = xp.maximum(lower - p, p - upper)

        # sdf = ‖max(d, 0)‖ + min(maxᵢ dᵢ, 0): the exterior distance plus the interior depth
        outside = xp.linalg.norm(xp.maximum(d, 0.0))
        inside = xp.minimum(xp.max(d), 0.0)
        sdf = outside + inside

        return sdf


class Union(Shape):
    """
    Union of several shapes, with ``sdf = min`` over the members.

    This is the obstacle-field primitive: a point is occupied when it lies
    inside any member shape. ``params`` holds the members' own trees by class
    name, a repeat numbered, ``{"Sphere": ..., "Sphere_2": ...}``.
    """

    params = held_params(
        "The members' numbers by class name, a repeat numbered (``Sphere``, ``Sphere_2``)."
    )

    def __init__(self, shapes):
        shapes = tuple(shapes)
        require_members(shapes)

        self.shapes = shapes
        self.params = {
            key: shape.params for key, shape in zip(member_keys(shapes), shapes)
        }

    def param_parts(self) -> dict:
        """The members by key: the parts an assignment routes to."""
        return dict(zip(member_keys(self.shapes), self.shapes))

    @property
    def dim(self) -> int:
        return self.shapes[0].dim

    def sdf(self, p, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(p)

        shapes = self.shapes

        # sdf = minᵢ sdfᵢ(p), the nearest member surface, reduced with xp.minimum so it
        # traces under JAX (Python min() would compare traced values and break jit)
        sdf = functools.reduce(
            xp.minimum,
            (
                shape.sdf(p, t, params[key])
                for key, shape in zip(member_keys(shapes), shapes)
            ),
        )

        return sdf


class Inflated(Shape):
    """
    Shape grown outward by ``radius`` (Minkowski sum with a ball).

    Subtracting the radius from a base signed distance gives the clearance for
    a disc-shaped robot, or a safety margin around a hard obstacle. ``params``
    holds ``{"base": the base shape's own numbers, "radius"}``.
    """

    params = held_params('``{"base": the base shape\'s numbers, "radius"}``.')

    def __init__(self, base: Shape, radius: float):
        self.base = base
        self.params = {"base": base.params, "radius": radius}

    radius = param_view("radius")

    def param_parts(self) -> dict:
        """The base shape: the part an assignment routes to."""
        return {"base": self.base}

    @property
    def dim(self) -> int:
        return self.base.dim

    def sdf(self, p, t=0.0, params=None):
        params = self.params if params is None else params

        base = self.base
        base_params = params["base"]
        radius = params["radius"]

        # sdf = sdf_base(p) − r: every surface moves outward by r
        sdf = base.sdf(p, t, base_params) - radius

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
