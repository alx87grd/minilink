"""
Deterministic allowable sets for planning problems.

Sets model constraints such as ``x(t) in X(t)``, ``u(t) in U(x, t)``,
and terminal goals ``x(tf) in Xf``. They expose boolean membership for
search-style planners and nonnegative margins for optimization-style
transcriptions.

Construction, membership checks, and sampling are NumPy/Python boundary
utilities. The equation methods ``margin`` and ``residual`` are native-array
math paths: with NumPy input they return NumPy arrays, and with JAX input they
return JAX arrays when the set formula is traceable.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module


class Set(ABC):
    """
    Base class for deterministic vector-valued allowable sets.

    A point is feasible when all components of :meth:`margin` are
    nonnegative. The set may depend on time and optional parameters, so
    the generic notation is ``z in Z(t)``.
    """

    @abstractmethod
    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return nonnegative feasibility margins for ``z``."""
        ...

    def contains(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Return ``True`` when ``z`` belongs to the set."""
        z_arr = np.asarray(z, dtype=float).reshape(-1)
        margin = np.asarray(self.margin(z_arr, t=t, params=params), dtype=float)
        return bool(np.all(margin >= 0.0))

    def sample(
        self,
        rng: np.random.Generator | None = None,
        n: int = 1,
        params=None,
    ) -> np.ndarray:
        """
        Draw samples from the set when supported.

        Search planners can use this optional method for samplable sets
        such as boxes. Complex sets may leave it unimplemented and rely
        on solver-provided samplers.
        """
        raise NotImplementedError("Sampling is not available for this set")


class InputSet(ABC):
    """
    Base class for admissible input sets ``u in U(x, t)``.

    Input feasibility often depends on the current state, for example
    actuator envelopes or contact-dependent controls. The state argument
    is optional so constant input boxes stay simple.
    """

    @abstractmethod
    def margin(
        self,
        u: np.ndarray,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return nonnegative feasibility margins for ``u``."""
        ...

    def contains(
        self,
        u: np.ndarray,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Return ``True`` when ``u`` is admissible at ``(x, t)``."""
        u_arr = np.asarray(u, dtype=float).reshape(-1)
        x_arr = None if x is None else np.asarray(x, dtype=float).reshape(-1)
        margin = np.asarray(
            self.margin(u_arr, x=x_arr, t=t, params=params),
            dtype=float,
        )
        return bool(np.all(margin >= 0.0))

    def sample(
        self,
        rng: np.random.Generator | None = None,
        n: int = 1,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Draw input samples when supported."""
        raise NotImplementedError("Sampling is not available for this input set")


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
        lower = np.asarray(self.lower, dtype=float).reshape(-1).copy()
        upper = np.asarray(self.upper, dtype=float).reshape(-1).copy()
        if lower.shape != upper.shape:
            raise ValueError("lower and upper must have the same shape")
        if np.any(lower > upper):
            raise ValueError("lower must be less than or equal to upper")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def dim(self) -> int:
        """Dimension of the boxed vector."""
        return int(self.lower.size)

    @classmethod
    def from_system_state(cls, sys) -> "BoxSet":
        """Create a state box from ``sys.state`` metadata."""
        return cls(sys.state.lower_bound, sys.state.upper_bound)

    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return lower and upper bound margins."""
        xp = array_module(z)
        return xp.concatenate((z - self.lower, self.upper - z))

    def sample(
        self,
        rng: np.random.Generator | None = None,
        n: int = 1,
        params=None,
    ) -> np.ndarray:
        """Draw uniformly from a finite box."""
        if n < 1:
            raise ValueError("n must be greater than or equal to 1")
        if not (np.all(np.isfinite(self.lower)) and np.all(np.isfinite(self.upper))):
            raise ValueError("Cannot sample from a box with infinite bounds")
        generator = np.random.default_rng() if rng is None else rng
        return generator.uniform(self.lower, self.upper, size=(int(n), self.dim))


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

    def margin(
        self,
        u: np.ndarray,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return lower and upper input-bound margins."""
        return self.box.margin(u, t=t, params=params)

    def sample(
        self,
        rng: np.random.Generator | None = None,
        n: int = 1,
        x: np.ndarray | None = None,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Draw uniformly from the input box."""
        return self.box.sample(rng=rng, n=n, params=params)


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

    def residual(self, z: np.ndarray) -> np.ndarray:
        """Return equality residual ``z - point``."""
        return z - self.point

    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return zero only when ``z`` equals the singleton point."""
        xp = array_module(z)
        return -xp.abs(self.residual(z))


@dataclass(frozen=True)
class BallSet(Set):
    """
    Euclidean ball ``||z - center|| <= radius``.
    """

    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float).reshape(-1).copy()
        radius = float(self.radius)
        if radius < 0.0:
            raise ValueError("radius must be nonnegative")
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "radius", radius)

    @property
    def dim(self) -> int:
        """Dimension of the ball center."""
        return int(self.center.size)

    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Return the signed distance margin to the ball boundary."""
        xp = array_module(z)
        return xp.reshape(self.radius - xp.linalg.norm(z - self.center), (1,))


class CallableSet(Set):
    """
    Set backed by user callables.

    Parameters
    ----------
    margin_fn : callable, optional
        Function ``margin_fn(z, t, params) -> np.ndarray``.
    contains_fn : callable, optional
        Function ``contains_fn(z, t, params) -> bool``.
    """

    def __init__(
        self,
        margin_fn: Callable[[np.ndarray, float, object | None], np.ndarray]
        | None = None,
        contains_fn: Callable[[np.ndarray, float, object | None], bool] | None = None,
    ) -> None:
        self.margin_fn = margin_fn
        self.contains_fn = contains_fn
        if self.margin_fn is None and self.contains_fn is None:
            raise ValueError("Provide at least one of margin_fn or contains_fn")

    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Evaluate the user-supplied margin function."""
        if self.margin_fn is None:
            raise NotImplementedError("This CallableSet has no margin function")
        return self.margin_fn(z, t, params)

    def contains(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> bool:
        """Evaluate membership through the supplied callable when present."""
        if self.contains_fn is not None:
            return bool(self.contains_fn(np.asarray(z, dtype=float), t, params))
        return super().contains(z, t=t, params=params)


class IntersectionSet(Set):
    """
    Intersection of multiple sets.

    This is a small convenience for combining boxes, goal regions, and
    collision-free domains without introducing a separate constraint layer.
    """

    def __init__(self, sets: tuple[Set, ...] | list[Set]) -> None:
        if not sets:
            raise ValueError("IntersectionSet requires at least one set")
        self.sets = tuple(sets)

    def margin(
        self,
        z: np.ndarray,
        t: float = 0.0,
        params=None,
    ) -> np.ndarray:
        """Concatenate margins from all member sets."""
        xp = array_module(z)
        return xp.concatenate(
            [set_.margin(z, t=t, params=params).reshape(-1) for set_ in self.sets]
        )
