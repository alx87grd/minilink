"""The workspace fields: clearance, cost density and the track's distances, each a :class:`~minilink.core.fields.Field` of the state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.fields import Field
from minilink.planning.spatial.collision import CollisionBody, iter_probes
from minilink.planning.spatial.scene import Scene

if TYPE_CHECKING:
    from minilink.planning.spatial.track import ReferenceTrack


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class ClearanceField(Field):
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
class CostDensityField(Field):
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
class PathDistanceField(Field):
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
class CorridorMarginField(Field):
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
