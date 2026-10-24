"""State fields: a scalar value of the state, exported as a set or a cost."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.costs import CostFunction, param_view
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import Set
from minilink.planning.spatial.collision import CollisionBody, iter_probes
from minilink.planning.spatial.scene import Scene

if TYPE_CHECKING:
    from minilink.planning.spatial.track import ReferenceTrack


###############################################################################
# Public API
###############################################################################


class StateField(ABC):
    """Scalar ``value(x)``; export with :meth:`as_constraint` or :meth:`as_cost`."""

    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None):
        """The field's scalar value at the state ``x``."""

    def as_constraint(
        self, *, lower: float | None = 0.0, upper: float | None = None
    ) -> Set:
        """The set of states where ``lower ≤ value(x) ≤ upper``."""
        return FieldSet(self, lower, upper)

    def as_cost(
        self,
        *,
        weight: float = 1.0,
        shaping: Callable | None = None,
    ) -> CostFunction:
        """The running cost ``g = weight · shaping(value(x))``, no terminal cost."""
        return FieldCost(self, float(weight), shaping)

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


@dataclass(frozen=True)
class ClearanceField(StateField):
    """``value(x) = minᵢ (clearance(pᵢ) − rᵢ)`` over the body's probes; nonnegative when free."""

    scene: Scene
    body: CollisionBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        scene = self.scene
        body = self.body

        # cᵢ = clearance(pᵢ) − rᵢ, the room around each probe disc
        c = []
        for world, radius in iter_probes(body, x, u, t, params):
            c.append(scene.clearance(world, t=t, params=params) - radius)

        # value(x) = minᵢ cᵢ: the probe closest to an obstacle decides
        clearance = xp.min(xp.stack(c))

        return clearance


@dataclass(frozen=True)
class CostDensityField(StateField):
    """``value(x) = maxᵢ density(pᵢ)`` over the body's probes."""

    scene: Scene
    body: CollisionBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        scene = self.scene
        body = self.body

        # dᵢ = density(pᵢ), at the center of each probe disc
        d = []
        for world, _ in iter_probes(body, x, u, t, params):
            d.append(scene.cost_density(world, t=t, params=params))

        # value(x) = maxᵢ dᵢ: the costliest probe decides
        density = xp.max(xp.stack(d))

        return density


@dataclass(frozen=True)
class PathDistanceField(StateField):
    """``value(x) = minᵢ (distance(pᵢ) − rᵢ)`` to the reference centerline."""

    track: ReferenceTrack
    body: CollisionBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        track = self.track
        body = self.body

        # dᵢ = distance(pᵢ) − rᵢ, from each probe disc to the centerline
        d = []
        for world, radius in iter_probes(body, x, u, t, params):
            d.append(track.distance(world, t=t, params=params) - radius)

        # value(x) = minᵢ dᵢ: the closest probe decides
        distance = xp.min(xp.stack(d))

        return distance


@dataclass(frozen=True)
class CorridorMarginField(StateField):
    """``value(x) = minᵢ (w − distance(pᵢ) − rᵢ)``; nonnegative in the tube."""

    track: ReferenceTrack
    body: CollisionBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        track = self.track
        body = self.body
        half_width = track.half_width

        # mᵢ = w − distance(pᵢ) − rᵢ, the room each probe disc has to the wall
        m = []
        for world, radius in iter_probes(body, x, u, t, params):
            m.append(half_width - track.distance(world, t=t, params=params) - radius)

        # value(x) = minᵢ mᵢ: the probe nearest the corridor wall decides
        margin = xp.min(xp.stack(m))

        return margin


@dataclass(frozen=True)
class FieldSet(Set):
    """Feasible where ``lower <= value <= upper``."""

    field: StateField
    lower: float | None = 0.0
    upper: float | None = None

    def __post_init__(self) -> None:
        require_a_bound(self.lower, self.upper)

    def margin(self, z, t=0.0, params=None):
        xp = array_module(z)

        field = self.field
        lower = self.lower
        upper = self.upper

        v = field.value(z, None, t=t, params=params)

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
    ``g = weight * shaping(value)``; ``shaping=None`` uses ``value`` directly.

    The weight is :attr:`params` (``{"weight": w}``, the System pattern); the
    field reads its own parameters, never the cost's.
    """

    def __init__(self, field: StateField, weight: float = 1.0, shaping=None):
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

        # g = w φ(v), with φ the identity when no shaping is given
        g = weight * shaped

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


###############################################################################
# Internal machinery
###############################################################################


def require_a_bound(lower, upper):
    """Refuse a field set with neither a lower nor an upper bound."""
    if lower is None and upper is None:
        raise ValueError("FieldSet requires at least one of lower or upper")
