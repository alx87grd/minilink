"""Allowable sets: ``x(t) in X(t)``, ``u(t) in U(x, t)`` and boundary sets ``x(tf) in Xf``."""

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from minilink.core.backends import (
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.inspect import inspect_text, repr_line, repr_pretty
from minilink.core.params import (
    delegated_params,
    held_params,
    member_keys,
    own_params,
    param_view,
)

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

    ``params`` is the set's own numbers (``{}`` for a set without any), by
    name, held as a plant holds its own: ``X.params["upper"] = ...`` moves the
    set, ``params=None`` in :meth:`margin` and :meth:`sample` reads them, and a
    tree passed instead is read in their place. A set's named attributes
    (``X.lower``) are views of its tree.
    """

    params = own_params()

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

    def sample(self, key, n=None, params=None):
        """
        Draw one point ``(dim,)`` or ``n`` points ``(n, dim)`` from the set when supported.

        ``key`` is a NumPy generator, an integer seed or a JAX PRNG key (JAX
        arrays out, traceable): the draw convention of
        :class:`~minilink.core.distributions.Distribution`. Search planners use
        this optional method on samplable sets such as boxes; other sets leave it
        unimplemented and are sampled by rejection from a box.
        """
        raise NotImplementedError("Sampling is not available for this set")

    def bounding_box(self):
        """The tightest :class:`BoxSet` containing the set, or ``None`` when it has none."""
        return None

    def as_cost(self, *, weight: float = 1.0, shaping: Callable | None = None):
        """
        The running cost of leaving the set: ``g = weight · shaping(V(x))``, ``V(x) = minᵢ mᵢ(x)``.

        The set's least margin seen as a field
        (:meth:`~minilink.core.fields.Field.as_cost`, the one implementation);
        ``shaping=None`` is the quadratic hinge ``max(−V, 0)²``, zero inside the
        set, so the cost is the soft form of the constraint.
        """
        # A real cycle: the fields build on the sets
        from minilink.core.fields import MarginField
        from minilink.core.shaping import quadratic_hinge

        shaping = quadratic_hinge() if shaping is None else shaping

        return MarginField(self).as_cost(weight=weight, shaping=shaping)

    def __and__(self, other: "Set") -> "IntersectionSet":
        """Return the intersection ``self & other`` of two sets."""
        return IntersectionSet.of(self, other)

    # Display

    def __str__(self):
        return inspect_text(self)

    def __repr__(self):
        return repr_line(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


class InputSet(ABC):
    """
    Base class for admissible input sets ``u in U(x, t)``.

    Input feasibility often depends on the current state, for example
    actuator envelopes or contact-dependent controls. The state argument
    is optional so constant input boxes stay simple. :meth:`margin`,
    :meth:`contains` and ``params`` follow the conventions of :class:`Set`.
    """

    params = own_params()

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

    def sample(self, key, n=None, x=None, t=0.0, params=None):
        """Draw one input ``(m,)`` or ``n`` inputs ``(n, m)`` when supported (same convention as :meth:`Set.sample`)."""
        raise NotImplementedError("Sampling is not available for this input set")

    def bounding_box(self):
        """The tightest :class:`BoxSet` of inputs containing the set, or ``None`` when it has none."""
        return None

    # Display

    def __str__(self):
        return inspect_text(self)

    def __repr__(self):
        return repr_line(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


class BoxSet(Set):
    """
    Axis-aligned box ``lower <= z <= upper``; ``params`` is ``{"lower", "upper"}``.

    Parameters
    ----------
    lower, upper : array_like
        Lower and upper bounds with shape ``(n,)``.
    """

    def __init__(self, lower, upper):
        # The bounds as flat float copies, checked before they are stored
        lower = np.asarray(lower, dtype=float).reshape(-1).copy()
        upper = np.asarray(upper, dtype=float).reshape(-1).copy()
        require_box_bounds(lower, upper)

        self.params = {"lower": lower, "upper": upper}

    lower = param_view("lower")
    upper = param_view("upper")

    @property
    def dim(self) -> int:
        """Dimension of the boxed vector."""
        return int(np.size(self.lower))

    def margin(self, z, t=0.0, params=None):
        """Return the margins to the lower and upper faces, shape ``(2n,)``."""
        params = self.params if params is None else params
        xp = array_module(z)

        lower = params["lower"]
        upper = params["upper"]
        require_member_size(z, lower, "the box holds vectors")

        # lower ≤ z ≤ upper ⇔ [z − lower; upper − z] ≥ 0
        margin = xp.concatenate((z - lower, upper - z))

        return margin

    def sample(self, key, n=None, params=None):
        """Draw uniformly from a finite box: ``(dim,)`` for one point, ``(n, dim)`` with ``n``; the center with no key."""
        params = self.params if params is None else params

        lower = params["lower"]
        upper = params["upper"]
        dim = self.dim

        require_finite_box(self)
        require_sample_count(n)

        shape = (dim,) if n is None else (int(n), dim)

        # A real cycle: the distributions build on the sets
        from minilink.core.distributions import (
            is_counter_key,
            standard_uniform,
            tiled_mean,
        )

        # No key, no draw: the box's center, (lower + upper) / 2
        if key is None:
            center = 0.5 * (lower + upper)
            return tiled_mean(center, n)

        # z ~ U(lower, upper), drawn on the key's backend (a counter key by the cipher)
        if is_counter_key(key):
            require_one_sample(n)
            z = lower + (upper - lower) * standard_uniform(key.seed, key.k, dim)
        elif is_jax_key(key):
            jax = require_jax()
            z = jax.random.uniform(key, shape, minval=lower, maxval=upper)
        else:
            z = numpy_generator(key).uniform(lower, upper, size=shape)

        return z

    def bounding_box(self):
        """A box is its own bounding box."""
        return self


class BoxInputSet(InputSet):
    """
    State-independent input box ``lower <= u <= upper``; ``params`` is its box's ``{"lower", "upper"}``.
    """

    params = delegated_params("box", "The input box's bounds: its box's numbers.")

    def __init__(self, box: BoxSet):
        self.box = box

    @property
    def lower(self):
        """The lower bound ``(m,)``, its box's."""
        return self.box.lower

    @property
    def upper(self):
        """The upper bound ``(m,)``, its box's."""
        return self.box.upper

    @property
    def dim(self) -> int:
        """Dimension of the input vector."""
        return self.box.dim

    def margin(self, u, x=None, t=0.0, params=None):
        """Return the margins to the input box faces, shape ``(2m,)``."""
        return self.box.margin(u, t=t, params=params)

    def sample(self, key, n=None, x=None, t=0.0, params=None):
        """Draw uniformly from the input box: ``(m,)`` for one input, ``(n, m)`` with ``n``."""
        return self.box.sample(key, n, params)

    def bounding_box(self):
        """The input box itself."""
        return self.box


class SingletonSet(Set):
    """
    Singleton boundary set ``z == point``.

    The margin uses a zero-tolerance absolute residual so exact equality
    can still be checked through :meth:`contains`. Optimizer equality
    constraints should use :meth:`residual` directly.
    """

    def __init__(self, point):
        point = np.asarray(point, dtype=float).reshape(-1).copy()

        self.params = {"point": point}

    point = param_view("point")

    @property
    def dim(self) -> int:
        """Dimension of the singleton point."""
        return int(np.size(self.point))

    def margin(self, z, t=0.0, params=None):
        """Return zero only when ``z`` equals the singleton point."""
        xp = array_module(z)

        residual = self.residual(z, params)

        # −|z − p| ≥ 0 only at z = p: the equality as a degenerate inequality
        margin = -xp.abs(residual)

        return margin

    def residual(self, z, params=None):
        """Return the equality residual ``z - point``, shape ``(n,)``."""
        params = self.params if params is None else params

        point = params["point"]
        require_member_size(z, point, "the singleton holds a vector")

        residual = z - point

        return residual

    def sample(self, key, n=None, params=None):
        """The point itself, whatever the key: ``(dim,)``, or ``(n, dim)`` with ``n``."""
        params = self.params if params is None else params
        require_sample_count(n)

        # A real cycle: the distributions build on the sets
        from minilink.core.distributions import tiled_mean

        # Every draw of {p} is p
        point = params["point"]

        return tiled_mean(point, n)

    def bounding_box(self):
        """The degenerate box ``point <= z <= point``."""
        return BoxSet(self.point, self.point)


class BallSet(Set):
    """
    Euclidean ball ``||z - center|| <= radius``; ``params`` is ``{"center", "radius"}``.
    """

    def __init__(self, center, radius):
        # The center and the radius as floats, checked before they are stored
        center = np.asarray(center, dtype=float).reshape(-1).copy()
        radius = float(radius)
        require_nonnegative_radius(radius)

        self.params = {"center": center, "radius": radius}

    center = param_view("center")
    radius = param_view("radius")

    @property
    def dim(self) -> int:
        """Dimension of the ball center."""
        return int(np.size(self.center))

    def margin(self, z, t=0.0, params=None):
        """Return the signed distance to the ball boundary, shape ``(1,)``."""
        params = self.params if params is None else params
        xp = array_module(z)

        center = params["center"]
        radius = params["radius"]
        require_member_size(z, center, "the ball holds vectors")

        # r − ‖z − c‖ ≥ 0 inside the ball, as a margin vector of one entry
        signed_distance = radius - xp.linalg.norm(z - center)
        margin = xp.reshape(signed_distance, (1,))

        return margin

    def sample(self, key, n=None, params=None):
        """Draw uniformly inside the ball: ``(dim,)`` for one point, ``(n, dim)`` with ``n``; the center with no key.

        ``key`` is a NumPy generator, an integer seed or a JAX key (JAX arrays
        out, traceable), as an ellipsoid draws; a counter key is refused.
        """
        params = self.params if params is None else params

        center = params["center"]
        radius = params["radius"]
        dim = self.dim
        require_sample_count(n)
        count = 1 if n is None else int(n)

        # A real cycle: the fields and the distributions build on the sets
        from minilink.core.distributions import tiled_mean
        from minilink.core.fields import (
            gaussian_and_uniform,
            require_ellipsoid_key,
            triangular_solver_of,
        )

        # No key, no draw: the ball's center
        if key is None:
            return tiled_mean(center, n)

        require_ellipsoid_key(key)
        xp, _ = triangular_solver_of(key)
        g, s = gaussian_and_uniform(key, count, dim)

        # z = c + r s^(1/n) g/‖g‖: a Gaussian direction, a radius whose n-th power is uniform
        direction = g / xp.linalg.norm(g, axis=1, keepdims=True)
        points = center + radius * s ** (1.0 / dim) * direction

        sample = points[0] if n is None else points

        return sample

    def bounding_box(self):
        """The box of half-width ``radius`` about the center."""
        center = self.center
        radius = self.radius

        # c − r ≤ z ≤ c + r
        lower = center - radius
        upper = center + radius

        return BoxSet(lower, upper)


class CallableSet(Set):
    """
    Set backed by a user margin, ``Z(t) = {z : margin(z, t, params) ≥ 0}``.

    Parameters
    ----------
    margin : callable
        Function ``margin(z, t, params) -> array``, nonnegative inside; a set
        is its margin, so membership is derived from it (:meth:`Set.contains`,
        every entry ``≥ 0``).
    params : dict, optional
        The numbers the function reads, handed to it when a call passes none.
    """

    def __init__(self, margin: Callable, params: dict | None = None):
        require_margin_function(margin)

        self.margin_fn = margin
        self.params = {} if params is None else params

    def margin(self, z, t=0.0, params=None):
        """Evaluate the user-supplied margin function."""
        params = self.params if params is None else params
        margin_fn = self.margin_fn

        margin = margin_fn(z, t, params)

        return margin


class IntersectionSet(Set):
    """
    Intersection of several sets, ``z in Z_1 and z in Z_2 and ...``.

    Built by the ``&`` operator on :class:`Set`; ``a & b & c`` is one flat
    intersection of three members. ``params`` holds the members' own trees by
    class name (a repeat numbered, ``{"BoxSet": ..., "BallSet": ...}``): an edit of a member's numbers is the
    intersection's, and an assignment routes each entry to its member.
    """

    params = held_params(
        'The members\' numbers by class name: ``{"BoxSet": ..., "BallSet": ...}``, a repeat numbered.'
    )

    def __init__(self, sets):
        sets = tuple(sets)
        require_members(sets)

        self.sets = sets
        self.params = {key: set_.params for key, set_ in zip(member_keys(sets), sets)}

    def param_parts(self) -> dict:
        """The members by key: the parts an assignment routes to."""
        return dict(zip(member_keys(self.sets), self.sets))

    @property
    def dim(self) -> int:
        """Dimension of the members' vector, read off the first member that states one."""
        for set_ in self.sets:
            if hasattr(set_, "dim"):
                return set_.dim
        raise AttributeError("no member of this intersection states its dimension")

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
        params = self.params if params is None else params
        xp = array_module(z)

        sets = self.sets

        # Each member's margins as a vector, a scalar margin as one entry
        margins = [
            xp.reshape(xp.asarray(set_.margin(z, t=t, params=params[key])), (-1,))
            for key, set_ in zip(member_keys(sets), sets)
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


def has_finite_face(box) -> bool:
    """Whether a set's bounding box has a finite face (``None``, no box, counts as having one: a general set)."""
    if box is None:
        return True

    return bool(np.any(np.isfinite(box.lower)) or np.any(np.isfinite(box.upper)))


###############################################################################
# Internal machinery
###############################################################################


def require_member_size(z, bound, what):
    """Refuse a member whose size is not its bound's: a set's ``margin`` takes its member first.

    The sizes are static shapes, so the check costs nothing under ``jit``; a
    scalar or a list member is left to broadcasting.
    """
    try:
        size, dim = z.shape[-1], bound.shape[-1]
    except (AttributeError, IndexError):
        return
    if size != dim:
        raise ValueError(
            f"{what}, of size {dim}; got a vector of size {size}: a set's margin "
            "takes its member first, margin(u, x, t) on an input set, margin(x, t) on "
            "a state set"
        )


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


def require_one_sample(n):
    """Refuse ``n`` draws from one counter key: it names one sample."""
    if n is not None:
        raise ValueError("a CounterKey names one sample: draw it with n=None")


def require_sample_count(n):
    """Refuse a sample count below one (``None`` draws a single point)."""
    if n is not None and int(n) < 1:
        raise ValueError("n must be greater than or equal to 1")


def require_nonnegative_radius(radius):
    """Refuse a negative radius."""
    if radius < 0.0:
        raise ValueError("radius must be nonnegative")


def require_margin_function(margin):
    """Refuse a callable set without a margin function: membership is derived from it."""
    if not callable(margin):
        raise TypeError(
            "CallableSet(margin) takes a function margin(z, t, params) -> array, "
            "nonnegative inside; membership is margin >= 0"
        )


def require_members(sets):
    """Refuse an intersection of no sets."""
    if not sets:
        raise ValueError("IntersectionSet requires at least one set")


def require_nonempty_intersection(lower, upper):
    """Refuse members whose boxes do not intersect: the intersection is empty."""
    if np.any(lower > upper):
        raise ValueError("the members' boxes do not intersect: the set is empty")
