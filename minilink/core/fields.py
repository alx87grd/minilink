"""Fields: a scalar function ``V(x, u, t)``, exported as a set, an input set or a cost."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module, require_jax
from minilink.core.compile.evaluators.jacobian import central_difference
from minilink.core.costs import CostFunction, param_view
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import InputSet, Set

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
    numbers (``{}`` for a field with none, R140).
    """

    params = {}

    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None):
        """The field's scalar value at ``(x, u, t)``."""

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
    """Feasible where ``lower <= V(x) <= upper``; the field reads its own parameters."""

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
    field reads its own parameters, never the cost's.
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


###############################################################################
# Internal machinery
###############################################################################


def require_a_bound(lower, upper):
    """Refuse a field set with neither a lower nor an upper bound."""
    if lower is None and upper is None:
        raise ValueError("a field's set requires at least one of lower or upper")
