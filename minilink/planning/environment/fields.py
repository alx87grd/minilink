"""
State fields and their constraint/cost adapters.

A :class:`StateField` is the fused result of placing a robot in an
environment: a native-array vector function of state that has not yet
committed to being a hard constraint or a soft cost. The two generic
adapters do that last step — :meth:`StateField.as_constraint` wraps it as a
:class:`~minilink.core.sets.Set` (feasible where the value is in range) and
:meth:`StateField.as_cost` wraps it as a
:class:`~minilink.core.costs.CostFunction` (a weighted, optionally shaped
penalty). The same field therefore drives obstacle avoidance as a constraint
*or* a barrier cost from one definition.

These adapters are the only new ``Set``/``CostFunction`` types; the base
contracts are unchanged. ``value`` carries an optional ``u`` so cost contexts
can pass the control while state constraints pass ``None``.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

from minilink.core.backends import array_module
from minilink.core.costs import CostFunction
from minilink.core.sets import Set


# Public API


class StateField(ABC):
    """
    A native-array vector function of state, viewable as a set or a cost.

    Subclasses implement :meth:`value`; the adapters below turn it into a
    feasibility :class:`~minilink.core.sets.Set` or a penalty
    :class:`~minilink.core.costs.CostFunction`.
    """

    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None):
        """Return the field vector at state ``x`` (native-array)."""
        ...

    def as_constraint(self, *, lower: float | None = 0.0, upper: float | None = None) -> Set:
        """Return a set feasible where ``lower <= value <= upper``."""
        return FieldSet(self, lower, upper)

    def as_cost(
        self,
        *,
        weight: float = 1.0,
        shaping: Callable | None = None,
    ) -> CostFunction:
        """Return a running cost ``weight * sum(shaping(value))``."""
        return FieldCost(self, float(weight), shaping)


@dataclass(frozen=True)
class FieldSet(Set):
    """
    Feasible set of a :class:`StateField`: ``lower <= value <= upper``.

    At least one bound must be set. With only ``lower`` (the default ``0``)
    the set is "value nonnegative", e.g. obstacle-free space; ``upper``
    alone gives a keep-out band such as a traversability ceiling.
    """

    field: StateField
    lower: float | None = 0.0
    upper: float | None = None

    def __post_init__(self) -> None:
        if self.lower is None and self.upper is None:
            raise ValueError("FieldSet requires at least one of lower or upper")

    def margin(self, z, t=0.0, params=None):
        xp = array_module(z)
        v = self.field.value(z, None, t=t, params=params)

        parts = []
        if self.lower is not None:
            parts.append(v - self.lower)
        if self.upper is not None:
            parts.append(self.upper - v)
        return xp.concatenate(parts)


@dataclass(frozen=True)
class FieldCost(CostFunction):
    """
    Running cost of a :class:`StateField`: ``weight * sum(shaping(value))``.

    With ``shaping=None`` the field value is summed directly (a
    traversability penalty). A barrier ``shaping`` turns a clearance field
    into a soft obstacle potential.
    """

    field: StateField
    weight: float = 1.0
    shaping: Callable | None = None

    def g(self, x, u, t=0.0, params=None):
        xp = array_module(x)
        v = self.field.value(x, u, t=t, params=params)
        shaped = v if self.shaping is None else self.shaping(v)

        return self.weight * xp.sum(shaped)

    def h(self, x, t=0.0, params=None):
        return 0.0
