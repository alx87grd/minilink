"""Fields: a scalar function ``V(x, u, t)``, exported as a set, an input set or a cost."""

from __future__ import annotations

import itertools
from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.linalg import solve_triangular

from minilink.core.backends import (
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.compile.evaluators.jacobian import central_difference
from minilink.core.costs import CostFunction
from minilink.core.distributions import is_counter_key
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.params import (
    delegated_params,
    held_params,
    own_params,
    param_view,
)
from minilink.core.sets import BoxSet, InputSet, Set
from minilink.core.shaping import shaping_of

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
    numbers (``{}`` for a field with none), and every set and cost built from
    the field carries them under ``"field"`` in its own tree.
    """

    params = own_params()

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


class FieldSet(Set):
    """
    Feasible where ``lower <= V(x) <= upper``.

    ``params`` holds ``{"field": the field's own numbers}`` and each bound that
    is set (``"lower"``, ``"upper"``): an edit of the field's numbers moves the
    set, and which bounds are set is the set's structure.
    """

    params = held_params(
        '``{"field", "lower", "upper"}``: the field\'s numbers and the bounds that are set.'
    )

    def __init__(
        self, field: Field, lower: float | None = 0.0, upper: float | None = None
    ):
        require_a_bound(lower, upper)

        self.field = field
        self.params = bounds_tree(field, lower, upper)

    def param_parts(self) -> dict:
        """The field: the part an assignment routes to."""
        return {"field": self.field}

    @property
    def lower(self):
        """The lower bound on ``V``, ``None`` when it is not set."""
        return self.params.get("lower")

    @property
    def upper(self):
        """The upper bound on ``V``, ``None`` when it is not set."""
        return self.params.get("upper")

    def margin(self, z, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(z)

        field = self.field
        field_params = params["field"]
        lower = params["lower"] if self.lower is not None else None
        upper = params["upper"] if self.upper is not None else None

        v = field.value(z, None, t, field_params)

        # margin = [v − lower; upper − v], a row for each bound that is set
        rows = []
        if lower is not None:
            rows.append(v - lower)
        if upper is not None:
            rows.append(upper - v)
        margin = xp.stack(rows)

        return margin


class FieldInputSet(InputSet):
    """
    Admissible where ``lower <= V(x, u) <= upper`` at the state ``x``.

    ``params`` holds ``{"field": the field's own numbers}`` and each bound that
    is set, as on :class:`FieldSet`.
    """

    params = held_params(
        '``{"field", "lower", "upper"}``: the field\'s numbers and the bounds that are set.'
    )

    def __init__(
        self, field: Field, lower: float | None = 0.0, upper: float | None = None
    ):
        require_a_bound(lower, upper)

        self.field = field
        self.params = bounds_tree(field, lower, upper)

    def param_parts(self) -> dict:
        """The field: the part an assignment routes to."""
        return {"field": self.field}

    @property
    def lower(self):
        """The lower bound on ``V``, ``None`` when it is not set."""
        return self.params.get("lower")

    @property
    def upper(self):
        """The upper bound on ``V``, ``None`` when it is not set."""
        return self.params.get("upper")

    def margin(self, u, x=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        field = self.field
        field_params = params["field"]
        lower = params["lower"] if self.lower is not None else None
        upper = params["upper"] if self.upper is not None else None

        v = field.value(x, u, t, field_params)

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

    :attr:`params` is ``{"weight": w, "field": the field's numbers}`` and, with a
    shaping, ``"shaping"``: its numbers (the System pattern), so the field's and
    the shaping's numbers are leaves of the cost's tree.
    """

    params = held_params(
        '``{"weight", "field"}``, and ``"shaping"`` when shaped: the field\'s and the shaping\'s own numbers.'
    )

    def __init__(self, field: Field, weight: float = 1.0, shaping=None):
        self.field = field
        self.shaping = shaping_of(shaping)
        self.params = cost_tree(field, weight, self.shaping)

    weight = param_view("weight")

    def param_parts(self) -> dict:
        """The field, and the shaping when there is one: the parts an assignment routes to."""
        return field_cost_parts(self)

    def g(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        field = self.field
        shaping = self.shaping
        weight = params["weight"]
        field_params = params["field"]

        v = field.value(x, u, t, field_params)
        shaped = v if shaping is None else shaping(v, params["shaping"])

        # g = w φ(V), with φ the identity when no shaping is given
        g = weight * shaped

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


class FieldTerminalCost(CostFunction):
    """``h = weight * shaping(V(x, t))``, no running cost; :attr:`params` is ``{"weight", "field", "shaping"}``, as on :class:`FieldCost`."""

    params = held_params(
        '``{"weight", "field"}``, and ``"shaping"`` when shaped: the field\'s and the shaping\'s own numbers.'
    )

    def __init__(self, field: Field, weight: float = 1.0, shaping=None):
        self.field = field
        self.shaping = shaping_of(shaping)
        self.params = cost_tree(field, weight, self.shaping)

    weight = param_view("weight")

    def param_parts(self) -> dict:
        """The field, and the shaping when there is one: the parts an assignment routes to."""
        return field_cost_parts(self)

    def g(self, x, u, t=0.0, params=None):
        return 0.0

    def h(self, x, t=0.0, params=None):
        params = self.params if params is None else params
        field = self.field
        shaping = self.shaping
        weight = params["weight"]
        field_params = params["field"]

        v = field.value(x, None, t, field_params)
        shaped = v if shaping is None else shaping(v, params["shaping"])

        # h = w φ(V(x)), with φ the identity when no shaping is given
        h = weight * shaped

        return h


class MarginField(Field):
    """A set's least margin as a field: ``V(x) = minᵢ mᵢ(x)``, nonnegative inside the set; ``params`` is the set's."""

    params = delegated_params("set_", "The set's numbers: the field moves with them.")

    def __init__(self, set_: Set):
        self.set_ = set_

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        set_ = self.set_

        # V(x) = minᵢ mᵢ(x): the face nearest to leaving decides
        least = xp.min(set_.margin(x, t=t, params=params))

        return least


class QuadraticField(Field):
    """
    The quadratic field ``V(x, t) = (x − c)ᵀ P(t) (x − c)``: a Lyapunov function, an LQR value.

    ``P`` is one symmetric matrix ``(n, n)``, or a schedule ``(N, n, n)`` on the
    times ``t`` ``(N,)``, ``N ≥ 2``, read linearly in ``t`` and held past its
    ends, as a time-varying gain is read: ``value(x, t)`` is then the
    finite-horizon LQR value ``(x − c)ᵀ S(t) (x − c)``. ``params`` holds
    ``center``, ``P`` and, for a schedule, its times ``t``; the gradient is exact, ``2 P(t) (x − c)``. A stack of
    states ``(N, n)`` gives one value per row. The sublevel set of a constant
    ``P``, ``as_set(upper=level)``, is an :class:`EllipsoidSet`, which draws and
    bounds itself.
    """

    def __init__(self, center, P, t=None):
        center = np.asarray(center, dtype=float).reshape(-1)
        P = np.asarray(P, dtype=float)
        t = None if t is None else np.asarray(t, dtype=float).reshape(-1)
        require_quadratic_shapes(center, P, t)
        require_symmetric(P)

        # The numbers: a schedule's times are among them, a constant P has none
        self.params = {"center": center, "P": P}
        if t is not None:
            self.params["t"] = t

    center = param_view("center")
    P = param_view("P")

    @property
    def t(self):
        """The schedule's times ``(N,)``, ``None`` for a constant ``P``."""
        return self.params.get("t")

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

        # A constant P is its own value at every t
        if self.t is None:
            return P
        times = xp.asarray(params["t"])

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


class EllipsoidSet(FieldSet):
    """
    The sublevel set of a quadratic field, ``{x : (x − c)ᵀ P (x − c) ≤ level}``, which draws and bounds itself.

    ``params`` is ``{"center", "P", "level"}``, the ellipsoid's own numbers;
    its :attr:`field` is the quadratic field at them. :meth:`sample` draws uniformly inside (a Gaussian direction, a
    radius ``r^{1/n}``, mapped through ``P = L Lᵀ``) from a NumPy generator, an
    integer seed or a JAX key, and gives the center for ``key=None``;
    :meth:`bounding_box` is ``c ± √(level · diag(P⁻¹))``, the ellipsoid's
    shadow on each axis.
    """

    params = own_params('``{"center", "P", "level"}``, the ellipsoid\'s numbers.')

    def __init__(self, field: QuadraticField, lower=None, upper=None):
        require_sublevel(lower, upper)

        self.params = {"center": field.center, "P": field.P, "level": upper}

    @property
    def field(self) -> QuadraticField:
        """The quadratic field ``V`` at the ellipsoid's numbers."""
        return QuadraticField(self.params["center"], self.params["P"])

    @property
    def lower(self):
        """An ellipsoid has no lower bound on ``V``."""
        return None

    @property
    def upper(self):
        """The sublevel ``c`` of ``V``, as given."""
        return self.params["level"]

    @property
    def level(self) -> float:
        """The sublevel ``c`` of ``V``."""
        return float(self.upper)

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


class GridField(Field):
    """
    A field tabulated on a regular grid: ``V(x)``, or ``V(x, t)``, read multilinearly between the nodes.

    ``levels`` holds each axis's increasing node coordinates and ``values`` the
    table on them, shape ``(n_0, …, n_{d−1})``; with times ``t`` ``(T,)``,
    ``T ≥ 2``, a leading time axis ``(T, n_0, …)`` read linearly in ``t`` and
    held past its ends, the rule of :class:`QuadraticField`. ``method`` is
    ``"linear"`` (multilinear in the cell, the default) or ``"nearest"``, both
    traced; ``"cubic"`` and ``"quintic"`` read the table by SciPy's splines, on
    NumPy only. Outside the grid the edge is held (``x`` clamped to the grid on
    each axis), or ``V = outside`` when a number is given: a value function's
    planner passes its failure cost there. ``params`` holds ``levels`` (by axis,
    ``"0"``, ``"1"``, …), ``values`` and, as given, ``t`` and ``outside``. A
    stack of states ``(M, n)`` gives one value per row; the gradient is the
    exact one of the cell's multilinear form.
    """

    METHODS = ("linear", "nearest", "cubic", "quintic")

    def __init__(self, levels, values, t=None, *, method="linear", outside=None):
        levels = [np.asarray(level, dtype=float).reshape(-1) for level in levels]
        values = np.asarray(values, dtype=float)
        t = None if t is None else np.asarray(t, dtype=float).reshape(-1)
        require_grid_shapes(levels, values, t)
        require_grid_method(method, self.METHODS)

        # The reading rule is structure; the table, its axes, its times and the value
        # outside the grid are numbers
        self.method = method
        self.params = {
            "levels": {str(k): level for k, level in enumerate(levels)},
            "values": values,
        }
        if t is not None:
            self.params["t"] = t
        if outside is not None:
            self.params["outside"] = float(outside)

    @property
    def t(self):
        """The table's times ``(T,)``, ``None`` for a table that does not vary in time."""
        return self.params.get("t")

    @property
    def n(self) -> int:
        """The dimension of the grid."""
        return len(self.params["levels"])

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, t, params["values"])

        levels = levels_of(params, self.n, xp)
        table = xp.asarray(self.table_at(t, params))  # On x's module, x traced or not
        X = xp.asarray(x, dtype=float)

        # x held inside the grid on each axis: the edge's value past it
        lower = xp.stack([level[0] for level in levels])
        upper = xp.stack([level[-1] for level in levels])
        X_held = xp.clip(X, lower, upper)

        # V(x) read on the grid: multilinear (or nearest) in the cell holding x
        if self.method in ("linear", "nearest"):
            V = grid_value(levels, table, X_held, nearest=self.method == "nearest")
        else:
            V = spline_value(levels, table, X_held, self.method)

        # A number outside the grid, when one is given
        if "outside" in params:
            inside = xp.all((X >= lower) & (X <= upper), axis=-1)
            V = xp.where(inside, V, params["outside"])

        return V

    def gradient(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        if self.method not in ("linear", "nearest"):
            return super().gradient(x, u, t, params)
        xp = array_module(x, t, params["values"])

        levels = levels_of(params, self.n, xp)
        table = xp.asarray(self.table_at(t, params))  # On x's module, x traced or not
        X = xp.asarray(x, dtype=float)

        # Zero along an axis where x is held, and everywhere outside with a number
        lower = xp.stack([level[0] for level in levels])
        upper = xp.stack([level[-1] for level in levels])
        inside = (X >= lower) & (X <= upper)
        if "outside" in params:
            inside = inside & xp.all(inside, axis=-1, keepdims=True)

        # ∂V/∂x_k of the cell's multilinear form; zero for the nearest node
        if self.method == "nearest":
            dV_dx = xp.zeros_like(X)
        else:
            dV_dx = grid_gradient(levels, table, xp.clip(X, lower, upper))
        dV_dx = xp.where(inside, dV_dx, 0.0)

        return dV_dx

    def table_at(self, t=0.0, params=None):
        """The table at ``t``: the table itself, or its time axis read linearly in ``t`` and held past its ends."""
        params = self.params if params is None else params
        xp = array_module(t, params["values"])

        values = xp.asarray(params["values"])

        # A table that does not vary in time is its own value at every t
        if self.t is None:
            return values
        times = xp.asarray(params["t"])

        # The bracket t_k ≤ t ≤ t_{k+1}, t held inside the times
        t_held = xp.clip(t, times[0], times[-1])
        k = xp.clip(xp.searchsorted(times, t_held, side="right") - 1, 0, times.size - 2)
        w = (t_held - times[k]) / (times[k + 1] - times[k])

        # V(t) = (1 − w) V_k + w V_{k+1}
        table = (1.0 - w) * values[k] + w * values[k + 1]

        return table


###############################################################################
# Internal machinery
###############################################################################


def cost_tree(field, weight, shaping) -> dict:
    """``{"weight", "field"}``, and ``"shaping"`` when the cost is shaped: a field cost's numbers."""
    tree = {"weight": float(weight), "field": field.params}
    if shaping is not None:
        tree["shaping"] = shaping.params

    return tree


def bounds_tree(field, lower, upper) -> dict:
    """``{"field": field.params}`` and each bound that is set, by name."""
    tree = {"field": field.params}
    if lower is not None:
        tree["lower"] = lower
    if upper is not None:
        tree["upper"] = upper

    return tree


def field_cost_parts(cost) -> dict:
    """A field cost's parts: its field, and its shaping when it has one."""
    parts = {"field": cost.field}
    if cost.shaping is not None:
        parts["shaping"] = cost.shaping

    return parts


def grid_value(levels, table, X, *, nearest=False):
    """
    The table read at ``X`` on the grid's ``levels``: multilinear in the cell holding each point, or its nearest node.

    ``X`` is one point ``(n,)`` or a stack ``(M, n)``. A point past the grid
    reads the edge cell's form extended (callers clamp it to hold the edge
    instead). ``table`` has the grid's shape, then any trailing axes (a vector
    per node); the result is ``(…)`` or ``(M, …)``. NumPy or JAX through the
    array module.
    """
    xp = array_module(X, table)
    n = len(levels)

    # A stack's weights take one axis per trailing axis of the table, to multiply its
    # vectors; a point's are numbers
    trailing = (1,) * (xp.ndim(table) - n) if xp.ndim(X) > 1 else ()

    # On each axis the cell l_{i_k} ≤ x_k < l_{i_k + 1} (the edge cell past the grid)
    # and the position in it, y_k = (x_k − l_{i_k}) / (l_{i_k + 1} − l_{i_k})
    cells = []
    for k in range(n):
        level = levels[k]
        i = xp.clip(xp.searchsorted(level, X[..., k]) - 1, 0, level.shape[0] - 2)
        y = (X[..., k] - level[i]) / (level[i + 1] - level[i])
        cells.append((i, y))

    # Nearest: the node of each axis's closer end, the lower one on a tie
    if nearest:
        index = tuple(xp.where(y <= 0.5, i, i + 1) for i, y in cells)
        return table[index]

    # Each axis's two weights, 1 − y_k toward its lower node and y_k toward its upper,
    # shaped to multiply a vector per node
    weights = [(1.0 - y, y) for _, y in cells]
    if trailing:
        weights = [
            tuple(xp.reshape(w, w.shape + trailing) for w in pair) for pair in weights
        ]

    # Linear: V = Σ_corners V_corner Πₖ (y_k or 1 − y_k) over the cell's 2ⁿ corners
    V = 0.0
    for corner in itertools.product((0, 1), repeat=n):
        index = tuple(i + upper for (i, _), upper in zip(cells, corner))
        term = table[index]
        for w, upper in zip(weights, corner):
            term = term * w[upper]
        V = V + term

    return V


def grid_gradient(levels, table, X):
    """``∂V/∂x`` of the multilinear form of the cell holding each point of ``X`` (``(n,)`` or ``(M, n)``): one entry per axis, last."""
    xp = array_module(X, table)
    n = len(levels)

    # The cell and the position in it on each axis, and the cell's width
    cells = []
    for k in range(n):
        level = levels[k]
        i = xp.clip(xp.searchsorted(level, X[..., k]) - 1, 0, level.shape[0] - 2)
        width = level[i + 1] - level[i]
        y = (X[..., k] - level[i]) / width
        cells.append((i, y, width))

    # ∂V/∂x_k = Σ_corners V_corner (±1 / Δ_k) Π_{j ≠ k} (y_j or 1 − y_j)
    columns = []
    for k in range(n):
        dV = 0.0
        for corner in itertools.product((0, 1), repeat=n):
            index = tuple(i + upper for (i, _, _), upper in zip(cells, corner))
            term = table[index]
            for j, ((_, y, width), upper) in enumerate(zip(cells, corner)):
                if j == k:
                    term = term * ((1.0 if upper else -1.0) / width)
                else:
                    term = term * (y if upper else 1.0 - y)
            dV = dV + term
        columns.append(dV)

    return xp.stack(columns, axis=-1)


def spline_value(levels, table, X, method):
    """The table read by SciPy's splines at ``X`` (one point ``(n,)`` or a stack ``(M, n)``; NumPy only)."""
    X = np.asarray(X, dtype=float)
    interp = RegularGridInterpolator(
        tuple(np.asarray(level) for level in levels), np.asarray(table), method=method
    )

    V = interp(np.atleast_2d(X))

    return V if X.ndim > 1 else V[0]


def levels_of(params, n, xp):
    """Each axis's levels from a grid field's tree, in order, as arrays of ``xp``."""
    return [xp.asarray(params["levels"][str(k)]) for k in range(n)]


def require_grid_shapes(levels, values, t):
    """Refuse levels that do not increase or a table that does not match them (with its time axis when timed)."""
    for k, level in enumerate(levels):
        if level.size < 2 or np.any(np.diff(level) <= 0.0):
            raise ValueError(
                f"axis {k}'s levels must be at least two increasing values"
            )
    shape = tuple(level.size for level in levels)
    expected = shape if t is None else (t.size, *shape)
    axes = "the levels' sizes" if t is None else "the levels' sizes, the times first"
    if values.shape != expected:
        raise ValueError(
            f"values must have shape {expected} ({axes}), got {values.shape}"
        )
    if t is not None and (t.size < 2 or np.any(np.diff(t) <= 0.0)):
        raise ValueError("the times t must be at least two increasing values")


def require_grid_method(method, supported):
    """Refuse a reading rule the grid field does not have."""
    if method not in supported:
        raise ValueError(f"method must be one of {supported}, got {method!r}")


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
