"""The workspace fields: clearance, cost density and the track's distances, each a :class:`~minilink.core.fields.Field` of the state."""

from __future__ import annotations

from dataclasses import dataclass, replace
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
    """
    ``value(x) = minᵢ (clearance(pᵢ) − rᵢ)`` over the body's probes; nonnegative when free.

    ``params`` is the scene's numbers (``{"scene": …}``), as on every workspace
    field (a track's for the track's fields); the body's probes are placed by
    the plant's own kinematics, on the plant's numbers.
    """

    scene: Scene
    body: CollisionBody

    @property
    def params(self) -> dict:
        """``{"scene"}``: the scene's numbers."""
        return {"scene": self.scene.params}

    def with_params(self, params):
        """The field on its scene moved to ``params``."""
        return replace(self, scene=self.scene.with_params(params["scene"]))

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        scene = self.scene
        scene_params = params["scene"]
        body = self.body

        # cᵢ = clearance(pᵢ) − rᵢ, the room around each probe disc
        c = []
        for world, radius in iter_probes(body, x, u, t):
            c.append(scene.clearance(world, t=t, params=scene_params) - radius)

        # value(x) = minᵢ cᵢ: the probe closest to an obstacle decides
        clearance = xp.min(xp.stack(c))

        return clearance


@dataclass(frozen=True)
class CostDensityField(Field):
    """``value(x) = maxᵢ density(pᵢ)`` over the body's probes."""

    scene: Scene
    body: CollisionBody

    @property
    def params(self) -> dict:
        """``{"scene"}``: the scene's numbers."""
        return {"scene": self.scene.params}

    def with_params(self, params):
        """The field on its scene moved to ``params``."""
        return replace(self, scene=self.scene.with_params(params["scene"]))

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        scene = self.scene
        scene_params = params["scene"]
        body = self.body

        # dᵢ = density(pᵢ), at the center of each probe disc
        d = []
        for world, _ in iter_probes(body, x, u, t):
            d.append(scene.cost_density(world, t=t, params=scene_params))

        # value(x) = maxᵢ dᵢ: the costliest probe decides
        density = xp.max(xp.stack(d))

        return density


@dataclass(frozen=True)
class PathDistanceField(Field):
    """``value(x) = minᵢ (distance(pᵢ) − rᵢ)`` to the reference centerline."""

    track: ReferenceTrack
    body: CollisionBody

    @property
    def params(self) -> dict:
        """``{"track"}``: the track's numbers."""
        return {"track": self.track.params}

    def with_params(self, params):
        """The field on its track moved to ``params``."""
        return replace(self, track=self.track.with_params(params["track"]))

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        track = self.track
        track_params = params["track"]
        body = self.body

        # dᵢ = distance(pᵢ) − rᵢ, from each probe disc to the centerline
        d = []
        for world, radius in iter_probes(body, x, u, t):
            d.append(track.distance(world, t=t, params=track_params) - radius)

        # value(x) = minᵢ dᵢ: the closest probe decides
        distance = xp.min(xp.stack(d))

        return distance


@dataclass(frozen=True)
class CorridorMarginField(Field):
    """``value(x) = minᵢ (w − distance(pᵢ) − rᵢ)``; nonnegative in the tube."""

    track: ReferenceTrack
    body: CollisionBody

    @property
    def params(self) -> dict:
        """``{"track"}``: the track's numbers, its corridor's half-width among them."""
        return {"track": self.track.params}

    def with_params(self, params):
        """The field on its track moved to ``params``."""
        return replace(self, track=self.track.with_params(params["track"]))

    def value(self, x, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        track = self.track
        body = self.body
        track_params = params["track"]
        half_width = track_params["half_width"]

        # mᵢ = w − distance(pᵢ) − rᵢ, the room each probe disc has to the wall
        m = []
        for world, radius in iter_probes(body, x, u, t):
            m.append(
                half_width - track.distance(world, t=t, params=track_params) - radius
            )

        # value(x) = minᵢ mᵢ: the probe nearest the corridor wall decides
        margin = xp.min(xp.stack(m))

        return margin
