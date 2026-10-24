"""Allowable sets: ``x(t) in X(t)``, ``u(t) in U(x, t)`` and boundary sets ``x(tf) in Xf``."""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import (
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.inspect import inspect_text, repr_pretty

###############################################################################
# Public API
###############################################################################


class Set(ABC):
    """
    Base class for deterministic vector-valued allowable sets.

    A point is feasible when all components of :meth:`margin` are
    nonnegative. The set may depend on time and optional parameters, so
    the generic notation is ``z in Z(t)``. :meth:`margin` is a native-array
    equation path (NumPy in, NumPy out; JAX in, JAX out), and :meth:`contains`
    is derived from it on the same array module (a JAX boolean under a trace);
    :meth:`sample` follows the draw convention of
    :class:`~minilink.core.distributions.Distribution`.

    ``params`` is the set's own parameters (``None`` for a set without any): a
    set that reads them takes ``params=None`` in :meth:`margin` as its own.
    """

    params = None

    @abstractmethod
    def margin(self, z, t=0.0, params=None):
        """Return nonnegative feasibility margins for ``z``, shape ``(k,)``."""
        ...

    def contains(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Whether ``z`` belongs to the set: a boolean on ``z``'s array module."""
        xp = array_module(z)
        z_arr = xp.reshape(xp.asarray(z, dtype=float), (-1,))
        margin = xp.asarray(self.margin(z_arr, t=t, params=params), dtype=float)

        # z ∈ Z(t) ⇔ every margin ≥ 0
        inside = xp.all(margin >= 0.0)

        return inside

    def sample(self, key=None, n=None, params=None):
        """
        Draw one point ``(dim,)`` or ``n`` points ``(n, dim)`` from the set when supported.

        ``key`` is a NumPy generator, an integer seed, ``None`` (unseeded) or a JAX
        PRNG key (JAX arrays out, traceable): the draw convention of
        :class:`~minilink.core.distributions.Distribution`. Search planners use
        this optional method on samplable sets such as boxes; other sets leave it
        unimplemented and are sampled by rejection from a box.
        """
        raise NotImplementedError("Sampling is not available for this set")

    def bounding_box(self):
        """The tightest :class:`BoxSet` containing the set, or ``None`` when it has none."""
        return None

    def __and__(self, other: "Set") -> "IntersectionSet":
        """Return the intersection ``self & other`` of two sets."""
        return IntersectionSet.of(self, other)

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


class InputSet(ABC):
    """
    Base class for admissible input sets ``u in U(x, t)``.

    Input feasibility often depends on the current state, for example
    actuator envelopes or contact-dependent controls. The state argument
    is optional so constant input boxes stay simple. :meth:`margin` and
    :meth:`contains` follow the conventions of :class:`Set`.
    """

    params = None

    @abstractmethod
    def margin(self, u, x=None, t=0.0, params=None):
        """Return nonnegative feasibility margins for ``u``, shape ``(k,)``."""
        ...

    def contains(
        self,
        u: np.ndarray,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Whether ``u`` is admissible at ``(x, t)``: a boolean on ``u``'s array module."""
        xp = array_module(u)
        u_arr = xp.reshape(xp.asarray(u, dtype=float), (-1,))
        x_arr = None if x is None else xp.reshape(xp.asarray(x, dtype=float), (-1,))
        margin = xp.asarray(
            self.margin(u_arr, x=x_arr, t=t, params=params), dtype=float
        )

        # u ∈ U(x, t) ⇔ every margin ≥ 0
        admissible = xp.all(margin >= 0.0)

        return admissible

    def sample(self, key=None, n=None, x=None, t=0.0, params=None):
        """Draw one input ``(m,)`` or ``n`` inputs ``(n, m)`` when supported (same convention as :meth:`Set.sample`)."""
        raise NotImplementedError("Sampling is not available for this input set")

    def bounding_box(self):
        """The tightest :class:`BoxSet` of inputs containing the set, or ``None`` when it has none."""
        return None

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


@dataclass(frozen=True)
class BoxSet(Set):
    """
    Axis-aligned box ``lower <= z <= upper``.

    Parameters
    ----------
    lower, upper : array_like
        Lower and upper bounds with shape ``(n,)``.
    """

    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self) -> None:
        # The bounds as flat float copies, checked before they are stored
        lower = np.asarray(self.lower, dtype=float).reshape(-1).copy()
        upper = np.asarray(self.upper, dtype=float).reshape(-1).copy()
        require_box_bounds(lower, upper)

        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def dim(self) -> int:
        """Dimension of the boxed vector."""
        return int(self.lower.size)

    @classmethod
    def from_system_state(cls, sys) -> "BoxSet":
        """The state box of ``sys``: its state bounds (``sys.state.box``)."""
        return sys.state.box

    def margin(self, z, t=0.0, params=None):
        """Return the margins to the lower and upper faces, shape ``(2n,)``."""
        xp = array_module(z)

        lower = self.lower
        upper = self.upper

        # lower ≤ z ≤ upper ⇔ [z − lower; upper − z] ≥ 0
        margin = xp.concatenate((z - lower, upper - z))

        return margin

    def sample(self, key=None, n=None, params=None):
        """Draw uniformly from a finite box: ``(dim,)`` for one point, ``(n, dim)`` with ``n``."""
        lower = self.lower
        upper = self.upper
        dim = self.dim

        require_finite_box(self)
        require_sample_count(n)

        shape = (dim,) if n is None else (int(n), dim)

        # z ~ U(lower, upper), drawn on the key's backend
        if is_jax_key(key):
            jax = require_jax()
            z = jax.random.uniform(key, shape, minval=lower, maxval=upper)
        else:
            z = numpy_generator(key).uniform(lower, upper, size=shape)

        return z

    def bounding_box(self):
        """A box is its own bounding box."""
        return self


@dataclass(frozen=True)
class BoxInputSet(InputSet):
    """
    State-independent input box ``lower <= u <= upper``.
    """

    box: BoxSet

    @classmethod
    def from_bounds(cls, lower: np.ndarray, upper: np.ndarray) -> "BoxInputSet":
        """Create an input box from lower and upper arrays."""
        return cls(BoxSet(lower, upper))

    @classmethod
    def from_system_inputs(cls, sys) -> "BoxInputSet":
        """The input box of ``sys``: its input ports' bounds, stacked in port order."""
        boxes = [port.box for port in sys.inputs.values()]
        lower = np.concatenate([box.lower for box in boxes]) if boxes else np.zeros(0)
        upper = np.concatenate([box.upper for box in boxes]) if boxes else np.zeros(0)

        return cls(BoxSet(lower, upper))

    def margin(self, u, x=None, t=0.0, params=None):
        """Return the margins to the input box faces, shape ``(2m,)``."""
        return self.box.margin(u, t=t, params=params)

    def sample(self, key=None, n=None, x=None, t=0.0, params=None):
        """Draw uniformly from the input box: ``(m,)`` for one input, ``(n, m)`` with ``n``."""
        return self.box.sample(key, n, params)

    def bounding_box(self):
        """The input box itself."""
        return self.box


@dataclass(frozen=True)
class SingletonSet(Set):
    """
    Singleton boundary set ``z == point``.

    The margin uses a zero-tolerance absolute residual so exact equality
    can still be checked through :meth:`contains`. Optimizer equality
    constraints should use :meth:`residual` directly.
    """

    point: np.ndarray

    def __post_init__(self) -> None:
        point = np.asarray(self.point, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "point", point)

    @property
    def dim(self) -> int:
        """Dimension of the singleton point."""
        return int(self.point.size)

    def margin(self, z, t=0.0, params=None):
        """Return zero only when ``z`` equals the singleton point."""
        xp = array_module(z)

        residual = self.residual(z)

        # −|z − p| ≥ 0 only at z = p: the equality as a degenerate inequality
        margin = -xp.abs(residual)

        return margin

    def residual(self, z):
        """Return the equality residual ``z - point``, shape ``(n,)``."""
        point = self.point

        residual = z - point

        return residual

    def bounding_box(self):
        """The degenerate box ``point <= z <= point``."""
        return BoxSet(self.point, self.point)


@dataclass(frozen=True)
class BallSet(Set):
    """
    Euclidean ball ``||z - center|| <= radius``.
    """

    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        # The center and the radius as floats, checked before they are stored
        center = np.asarray(self.center, dtype=float).reshape(-1).copy()
        radius = float(self.radius)
        require_nonnegative_radius(radius)

        object.__setattr__(self, "center", center)
        object.__setattr__(self, "radius", radius)

    @property
    def dim(self) -> int:
        """Dimension of the ball center."""
        return int(self.center.size)

    def margin(self, z, t=0.0, params=None):
        """Return the signed distance to the ball boundary, shape ``(1,)``."""
        xp = array_module(z)

        center = self.center
        radius = self.radius

        # r − ‖z − c‖ ≥ 0 inside the ball, as a margin vector of one entry
        signed_distance = radius - xp.linalg.norm(z - center)
        margin = xp.reshape(signed_distance, (1,))

        return margin

    def bounding_box(self):
        """The box of half-width ``radius`` about the center."""
        center = self.center
        radius = self.radius

        # c − r ≤ z ≤ c + r
        lower = center - radius
        upper = center + radius

        return BoxSet(lower, upper)


@dataclass(frozen=True)
class CallableSet(Set):
    """
    Set backed by user callables.

    Parameters
    ----------
    margin_fn : callable, optional
        Function ``margin_fn(z, t, params) -> array``.
    contains_fn : callable, optional
        Function ``contains_fn(z, t, params) -> bool``.
    """

    margin_fn: Callable | None = None
    contains_fn: Callable | None = None

    def __post_init__(self) -> None:
        require_margin_or_contains(self.margin_fn, self.contains_fn)

    def margin(self, z, t=0.0, params=None):
        """Evaluate the user-supplied margin function."""
        margin_fn = self.margin_fn
        require_margin_fn(margin_fn)

        margin = margin_fn(z, t, params)

        return margin

    def contains(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Evaluate membership through the supplied callable when present, on a flat ``z``."""
        contains_fn = self.contains_fn
        if contains_fn is not None:
            return bool(contains_fn(np.asarray(z, dtype=float).reshape(-1), t, params))

        return super().contains(z, t=t, params=params)


@dataclass(frozen=True)
class IntersectionSet(Set):
    """
    Intersection of several sets, ``z in Z_1 and z in Z_2 and ...``.

    Built by the ``&`` operator on :class:`Set`; ``a & b & c`` is one flat
    intersection of three members.
    """

    sets: tuple

    def __post_init__(self) -> None:
        sets = tuple(self.sets)
        require_members(sets)

        object.__setattr__(self, "sets", sets)

    @classmethod
    def of(cls, *sets: Set) -> "IntersectionSet":
        """Build an intersection, flattening nested intersections into one member list."""
        members: list[Set] = []
        for set_ in sets:
            if isinstance(set_, IntersectionSet):
                members.extend(set_.sets)
            else:
                members.append(set_)

        return cls(tuple(members))

    def margin(self, z, t=0.0, params=None):
        """Concatenate the margins of all member sets (a scalar margin counts as one)."""
        xp = array_module(z)

        sets = self.sets

        # Each member's margins as a vector, a scalar margin as one entry
        margins = [
            xp.reshape(xp.asarray(set_.margin(z, t=t, params=params)), (-1,))
            for set_ in sets
        ]

        # z ∈ Z₁ ∩ Z₂ ∩ … ⇔ every member's margins ≥ 0
        margin = xp.concatenate(margins)

        return margin

    def bounding_box(self):
        """The intersection of the members' boxes; ``None`` when no member has one.

        Members whose boxes do not intersect make the set empty, which is refused.
        """
        boxes = [
            box
            for box in (set_.bounding_box() for set_ in self.sets)
            if box is not None
        ]
        if not boxes:
            return None

        # The boxes meet in max lowerᵢ ≤ z ≤ min upperᵢ
        lower = np.max([box.lower for box in boxes], axis=0)
        upper = np.min([box.upper for box in boxes], axis=0)
        require_nonempty_intersection(lower, upper)

        return BoxSet(lower, upper)


def is_finite_box(box) -> bool:
    """``True`` for a box with finite bounds on every axis."""
    return box is not None and bool(
        np.all(np.isfinite(box.lower)) and np.all(np.isfinite(box.upper))
    )


###############################################################################
# Internal machinery
###############################################################################


def require_box_bounds(lower, upper):
    """Refuse bounds of different shapes, or a lower bound above its upper bound."""
    if lower.shape != upper.shape:
        raise ValueError("lower and upper must have the same shape")
    if np.any(lower > upper):
        raise ValueError("lower must be less than or equal to upper")


def require_finite_box(box):
    """Refuse to sample a box with an infinite bound."""
    if not is_finite_box(box):
        raise ValueError("Cannot sample from a box with infinite bounds")


def require_sample_count(n):
    """Refuse a sample count below one (``None`` draws a single point)."""
    if n is not None and int(n) < 1:
        raise ValueError("n must be greater than or equal to 1")


def require_nonnegative_radius(radius):
    """Refuse a negative radius."""
    if radius < 0.0:
        raise ValueError("radius must be nonnegative")


def require_margin_or_contains(margin_fn, contains_fn):
    """Refuse a callable set given neither a margin nor a membership function."""
    if margin_fn is None and contains_fn is None:
        raise ValueError("Provide at least one of margin_fn or contains_fn")


def require_margin_fn(margin_fn):
    """Refuse a margin query on a callable set built from a membership function only."""
    if margin_fn is None:
        raise NotImplementedError("This CallableSet has no margin function")


def require_members(sets):
    """Refuse an intersection of no sets."""
    if not sets:
        raise ValueError("IntersectionSet requires at least one set")


def require_nonempty_intersection(lower, upper):
    """Refuse members whose boxes do not intersect: the intersection is empty."""
    if np.any(lower > upper):
        raise ValueError("the members' boxes do not intersect: the set is empty")
