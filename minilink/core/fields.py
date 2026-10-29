"""Fields: a scalar function ``V(x, u, t)``, exported as a set, an input set or a cost."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_triangular

from minilink.core.backends import (
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.compile.evaluators.jacobian import central_difference
from minilink.core.costs import CostFunction, param_view
from minilink.core.distributions import is_counter_key
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import BoxSet, InputSet, Set

# The step of the central difference that a NumPy gradient takes
GRADIENT_EPS = 1e-6


###############################################################################
# Public API
###############################################################################


class Field(ABC):
    """
    A scalar function of the state, the input and the time: ``V(x, u, t)``.

    :meth:`value` is a native-array equation path that reads the arguments it
    depends on and ignores the rest, as ``f`` ignores ``t``. Four conversions,
    named by what they return: :meth:`as_set` (a :class:`~minilink.core.sets.Set`
    over ``x``), :meth:`as_input_set` (an :class:`~minilink.core.sets.InputSet`
    over ``u`` at a given ``x``), :meth:`as_cost` (the running cost ``g``) and
    :meth:`as_terminal_cost` (the terminal cost ``h``). ``V.as_set(upper=level)``
    is the sublevel set ``{x : V(x) ≤ level}``. ``params`` is the field's own
    numbers (``{}`` for a field with none).
    """

    params = {}

    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None):
        """The field's scalar value at ``(x, u, t)``."""

    def __call__(self, x, u=None, t=0.0, params=None):
        """``V(x, u, t)``, the same as :meth:`value`."""
        return self.value(x, u, t, params)

    def gradient(self, x, u=None, t=0.0, params=None):
        """``∂V/∂x`` at ``(x, u, t)``: ``jax.grad`` on JAX arrays, central differences on NumPy."""
        if array_module(x) is not np:
            jax = require_jax()
            return jax.grad(lambda z: self.value(z, u, t, params))(x)

        dV_dx = central_difference(
            lambda z: self.value(z, u, t, params), x, GRADIENT_EPS
        )

        return dV_dx.reshape(-1)

    def as_set(self, *, lower: float | None = 0.0, upper: float | None = None) -> Set:
        """The set of states where ``lower ≤ V(x) ≤ upper``."""
        return FieldSet(self, lower, upper)

    def as_input_set(
        self, *, lower: float | None = 0.0, upper: float | None = None
    ) -> InputSet:
        """The inputs where ``lower ≤ V(x, u) ≤ upper`` at the state ``x``."""
        return FieldInputSet(self, lower, upper)

    def as_cost(
        self, *, weight: float = 1.0, shaping: Callable | None = None
    ) -> CostFunction:
        """The running cost ``g = weight · shaping(V(x, u, t))``, no terminal cost."""
        return FieldCost(self, float(weight), shaping)

    def as_terminal_cost(
        self, *, weight: float = 1.0, shaping: Callable | None = None
    ) -> CostFunction:
        """The terminal cost ``h = weight · shaping(V(x, t))``, no running cost."""
        return FieldTerminalCost(self, float(weight), shaping)

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


@dataclass(frozen=True)
class FieldSet(Set):
    """Feasible where ``lower <= V(x) <= upper``; the field's own numbers are not yet leaves of the set's tree."""

    field: Field
    lower: float | None = 0.0
    upper: float | None = None

    def __post_init__(self) -> None:
        require_a_bound(self.lower, self.upper)

    def margin(self, z, t=0.0, params=None):
        xp = array_module(z)

        field = self.field
        lower = self.lower
        upper = self.upper

        v = field.value(z, None, t=t)

        # margin = [v − lower; upper − v], a row for each bound that is set
        rows = []
        if lower is not None:
            rows.append(v - lower)
        if upper is not None:
            rows.append(upper - v)
        margin = xp.stack(rows)

        return margin


@dataclass(frozen=True)
class FieldInputSet(InputSet):
    """Admissible where ``lower <= V(x, u) <= upper`` at the state ``x``."""

    field: Field
    lower: float | None = 0.0
    upper: float | None = None

    def __post_init__(self) -> None:
        require_a_bound(self.lower, self.upper)

    def margin(self, u, x=None, t=0.0, params=None):
        xp = array_module(u)

        field = self.field
        lower = self.lower
        upper = self.upper

        v = field.value(x, u, t=t)

        # margin = [v − lower; upper − v], a row for each bound that is set
        rows = []
        if lower is not None:
            rows.append(v - lower)
        if upper is not None:
            rows.append(upper - v)
        margin = xp.stack(rows)

        return margin


class FieldCost(CostFunction):
    """
    ``g = weight * shaping(V)``; ``shaping=None`` uses ``V`` directly.

    The weight is :attr:`params` (``{"weight": w}``, the System pattern); the
    field's own numbers are not yet leaves of the cost's tree.
    """

    def __init__(self, field: Field, weight: float = 1.0, shaping=None):
        self.field = field
        self.shaping = shaping
        self.params = {"weight": float(weight)}

    weight = param_view("weight")

    def g(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        field = self.field
        shaping = self.shaping
        weight = params["weight"]

        v = field.value(x, u, t=t)
        shaped = v if shaping is None else shaping(v)

        # g = w φ(V), with φ the identity when no shaping is given
        g = weight * shaped

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


class FieldTerminalCost(CostFunction):
    """``h = weight * shaping(V(x, t))``, no running cost; the weight is :attr:`params`."""

    def __init__(self, field: Field, weight: float = 1.0, shaping=None):
        self.field = field
        self.shaping = shaping
        self.params = {"weight": float(weight)}

    weight = param_view("weight")

    def g(self, x, u, t=0.0, params=None):
        return 0.0

    def h(self, x, t=0.0, params=None):
        params = self.params if params is None else params
        field = self.field
        shaping = self.shaping
        weight = params["weight"]

        v = field.value(x, None, t=t)
        shaped = v if shaping is None else shaping(v)

        # h = w φ(V(x)), with φ the identity when no shaping is given
        h = weight * shaped

        return h


@dataclass(frozen=True)
class MarginField(Field):
    """A set's least margin as a field: ``V(x) = minᵢ mᵢ(x)``, nonnegative inside the set."""

    set_: Set

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        set_ = self.set_

        # V(x) = minᵢ mᵢ(x): the face nearest to leaving decides
        least = xp.min(set_.margin(x, t=t, params=params))

        return least


@dataclass(frozen=True)
class QuadraticField(Field):
    """
    The quadratic field ``V(x, t) = (x − c)ᵀ P(t) (x − c)``: a Lyapunov function, an LQR value.

    ``P`` is one symmetric matrix ``(n, n)``, or a schedule ``(N, n, n)`` on the
    times ``t`` ``(N,)``, ``N ≥ 2``, read linearly in ``t`` and held past its
    ends, as a time-varying gain is read: ``value(x, t)`` is then the
    finite-horizon LQR value ``(x − c)ᵀ S(t) (x − c)``. ``params`` holds
    ``center`` and ``P``; the gradient is exact, ``2 P(t) (x − c)``. A stack of
    states ``(N, n)`` gives one value per row. The sublevel set of a constant
    ``P``, ``as_set(upper=level)``, is an :class:`EllipsoidSet`, which draws and
    bounds itself.
    """

    center: np.ndarray
    P: np.ndarray
    t: np.ndarray | None = None

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float).reshape(-1)
        P = np.asarray(self.P, dtype=float)
        t = None if self.t is None else np.asarray(self.t, dtype=float).reshape(-1)
        require_quadratic_shapes(center, P, t)
        require_symmetric(P)

        object.__setattr__(self, "center", center)
        object.__setattr__(self, "P", P)
        object.__setattr__(self, "t", t)

    @property
    def params(self) -> dict:
        """``{"center", "P"}``, the field's numbers."""
        return {"center": self.center, "P": self.P}

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, t, params["center"], params["P"])

        center = params["center"]
        P_t = self.matrix_at(t, params)

        # V = dᵀ P(t) d, d = x − c, one value per row of a stack
        d = xp.atleast_2d(x - center)
        v_rows = xp.einsum("ij,jk,ik->i", d, P_t, d)

        V = v_rows if xp.ndim(x) > 1 else v_rows[0]

        return V

    def gradient(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params

        center = params["center"]
        P_t = self.matrix_at(t, params)

        # ∂V/∂x = 2 P(t) (x − c), P symmetric
        d = x - center
        dV_dx = 2.0 * (d @ P_t)

        return dV_dx

    def matrix_at(self, t=0.0, params=None):
        """``P(t)``: the matrix, or the schedule read linearly in ``t`` and held past its ends."""
        params = self.params if params is None else params
        xp = array_module(t, params["P"])

        P = xp.asarray(params["P"])
        times = self.t

        # A constant P is its own value at every t
        if times is None:
            return P
        times = xp.asarray(times)

        # The bracket t_k ≤ t ≤ t_{k+1} of the schedule, t held inside its ends
        t_held = xp.clip(t, times[0], times[-1])
        k = xp.clip(xp.searchsorted(times, t_held, side="right") - 1, 0, times.size - 2)
        w = (t_held - times[k]) / (times[k + 1] - times[k])

        # P(t) = (1 − w) P_k + w P_{k+1}
        P_t = (1.0 - w) * P[k] + w * P[k + 1]

        return P_t

    def as_set(self, *, lower: float | None = 0.0, upper: float | None = None) -> Set:
        """``{x : lower ≤ V(x) ≤ upper}``; the sublevel set of a constant ``P`` (``upper`` alone, ``lower`` 0 or ``None``) is an :class:`EllipsoidSet`."""
        sublevel = upper is not None and lower in (None, 0.0) and self.t is None
        if sublevel:
            return EllipsoidSet(self, None, upper)

        return FieldSet(self, lower, upper)


@dataclass(frozen=True)
class EllipsoidSet(FieldSet):
    """
    The sublevel set of a quadratic field, ``{x : (x − c)ᵀ P (x − c) ≤ level}``, which draws and bounds itself.

    ``params`` is ``{"center", "P", "level"}``, and :meth:`with_params` the
    moved copy. :meth:`sample` draws uniformly inside (a Gaussian direction, a
    radius ``r^{1/n}``, mapped through ``P = L Lᵀ``) from a NumPy generator, an
    integer seed or a JAX key, and gives the center for ``key=None``;
    :meth:`bounding_box` is ``c ± √(level · diag(P⁻¹))``, the ellipsoid's
    shadow on each axis.
    """

    def __post_init__(self) -> None:
        require_sublevel(self.lower, self.upper)

    @property
    def params(self) -> dict:
        """``{"center", "P", "level"}``, the ellipsoid's numbers."""
        field = self.field
        return {"center": field.center, "P": field.P, "level": self.upper}

    @property
    def level(self) -> float:
        """The sublevel ``c`` of ``V``."""
        return float(self.upper)

    def with_params(self, params):
        """The ellipsoid moved to ``params`` (any of ``center``, ``P``, ``level``)."""
        moved = {**self.params, **params}
        field = QuadraticField(moved["center"], moved["P"])

        return EllipsoidSet(field, None, moved["level"])

    def margin(self, z, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(z)

        field = self.field
        level = params["level"]
        quadratic = {"center": params["center"], "P": params["P"]}

        v = field.value(z, None, t, quadratic)

        # margin = c − V(z), nonnegative inside
        margin = xp.stack([level - v])

        return margin

    def sample(self, key, n=None, params=None):
        """Draw uniformly inside the ellipsoid: ``(n_x,)`` for one point, ``(n, n_x)`` with ``n``; ``key=None`` the center."""
        params = self.params if params is None else params

        center = params["center"]
        P = params["P"]
        level = params["level"]
        dim = int(center.shape[-1])
        count = 1 if n is None else int(n)

        # No key: the center, the design point
        if key is None:
            return center_points(center, n)

        require_ellipsoid_key(key)
        xp, solve = triangular_solver_of(key)
        g, s = gaussian_and_uniform(key, count, dim)

        # z uniform in the unit ball: a Gaussian direction g/‖g‖ scaled by r = s^(1/dim)
        r = s ** (1.0 / dim)
        z = g * (r / xp.linalg.norm(g, axis=1, keepdims=True))

        # P = L Lᵀ, so d = √c L⁻ᵀ z satisfies dᵀ P d ≤ c
        L = xp.linalg.cholesky(P)
        d = xp.sqrt(level) * solve(L.T, z.T, lower=False).T
        points = center + d

        sample = points[0] if n is None else points

        return sample

    @property
    def half_width(self) -> np.ndarray:
        """``√(level · diag(P⁻¹))``: the ellipsoid's half-width along each axis."""
        params = self.params

        P = params["P"]
        level = params["level"]

        # The shadow on axis i: √(c (P⁻¹)ᵢᵢ)
        half_width = np.sqrt(level * np.diag(np.linalg.inv(P)))

        return half_width

    def bounding_box(self):
        """``c ± half_width``: the ellipsoid's shadow on each axis."""
        center = self.params["center"]
        half_width = self.half_width

        box = BoxSet(center - half_width, center + half_width)

        return box


###############################################################################
# Internal machinery
###############################################################################


def require_a_bound(lower, upper):
    """Refuse a field set with neither a lower nor an upper bound."""
    if lower is None and upper is None:
        raise ValueError("a field's set requires at least one of lower or upper")


def require_quadratic_shapes(center, P, t):
    """Refuse a quadratic field whose matrix, or schedule, does not match its center."""
    n = center.size
    if t is None and P.shape != (n, n):
        raise ValueError(
            f"P must be ({n}, {n}) for a center of size {n}, got {P.shape}"
        )
    if t is not None and (t.size < 2 or P.shape != (t.size, n, n)):
        raise ValueError(
            f"a schedule P must be ({t.size}, {n}, {n}) on {t.size} times, at least two, "
            f"got {P.shape}"
        )


def center_points(center, n):
    """The center as the draw with no key: ``(n_x,)``, or ``n`` rows of it."""
    if n is None:
        return center

    return np.tile(center, (int(n), 1))


def require_symmetric(P):
    """Refuse a quadratic field whose matrix, or a matrix of its schedule, is not symmetric."""
    P_T = np.swapaxes(P, -1, -2)
    if not np.allclose(P, P_T, rtol=1e-9, atol=1e-12):
        raise ValueError(
            "P must be symmetric: (x − c)ᵀ P (x − c) reads only ½(P + Pᵀ), so pass "
            "that matrix"
        )


def require_sublevel(lower, upper):
    """Refuse an ellipsoid set that is not a sublevel set ``{x : V(x) ≤ level}``, ``level ≥ 0``."""
    if upper is None or lower not in (None, 0.0) or upper < 0.0:
        raise ValueError(
            "an ellipsoid set is {x : V(x) <= level}: give upper=level >= 0 and "
            f"lower 0 or None, got lower={lower}, upper={upper}"
        )


def require_ellipsoid_key(key):
    """Refuse a counter key: an ellipsoid's draw takes many numbers per point."""
    if is_counter_key(key):
        raise TypeError(
            "an ellipsoid draws from a NumPy generator, an integer seed or a JAX key, "
            "not a counter key"
        )


def triangular_solver_of(key):
    """The array module and the triangular solve of a draw on ``key``'s backend."""
    if is_jax_key(key):
        jax = require_jax()
        return jax.numpy, jax.scipy.linalg.solve_triangular

    return np, solve_triangular


def gaussian_and_uniform(key, count, dim):
    """``g ~ N(0, I)`` of shape ``(count, dim)``, then ``s ~ U(0, 1)`` of shape ``(count, 1)``, on the key's backend."""
    if is_jax_key(key):
        jax = require_jax()
        key_g, key_s = jax.random.split(key)
        g = jax.random.normal(key_g, (count, dim))
        s = jax.random.uniform(key_s, (count, 1))
        return g, s

    rng = numpy_generator(key)
    g = rng.standard_normal((count, dim))
    s = rng.random((count, 1))

    return g, s
