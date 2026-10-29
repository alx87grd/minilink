"""Reference tracks: a centerline path with a constant-width corridor around it."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.planning.spatial.collision import CollisionBody
from minilink.planning.spatial.paths import ReferencePath

if TYPE_CHECKING:
    from minilink.core.fields import Field


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class ReferenceTrack:
    """
    Target path with a constant half-width corridor in workspace units.

    A track composes into a :class:`~minilink.planning.problems.PlanningProblem`
    through its state fields::

        track = ReferenceTrack(from_waypoints(pts), half_width=1.0)
        X = bounds & track.corridor_field(body).as_set()
        cost = base + track.distance_field(body).as_cost(weight=5.0)

    Parameters
    ----------
    path : ReferencePath
        Workspace centerline.
    half_width : float
        Corridor radius around the path (same units as ``path.distance``).

    ``params`` is ``{"path": the path's numbers, "half_width"}``.
    """

    path: ReferencePath
    half_width: float = 1.0

    def __post_init__(self) -> None:
        require_positive_half_width(self.half_width)

    @property
    def params(self) -> dict:
        """``{"path", "half_width"}``: the centerline's numbers and the corridor's radius."""
        return {"path": self.path.params, "half_width": self.half_width}

    def with_params(self, params):
        """The track with its centerline and half-width moved to ``params``."""
        path = self.path.with_params(params["path"])

        return replace(self, path=path, half_width=params["half_width"])

    def distance(self, p, t=0.0, params=None):
        """Distance from ``p`` to the centerline."""
        params = self.params if params is None else params
        return self.path.distance(p, t=t, params=params["path"])

    def corridor_margin(self, p, t=0.0, params=None):
        """Positive when ``p`` lies inside the corridor: ``half_width - distance``."""
        params = self.params if params is None else params
        xp = array_module(p)

        half_width = params["half_width"]
        path = self.path
        path_params = params["path"]

        # m(p) = w − d(p), the room left to the corridor's edge
        margin = xp.asarray(half_width) - path.distance(p, t=t, params=path_params)

        return margin

    def distance_field(self, body: CollisionBody) -> Field:
        """The least distance from ``body``'s probes to the centerline, as a field."""
        # Lazy import: the state fields load on first use, not with the track
        from minilink.planning.spatial.state_fields import PathDistanceField

        return PathDistanceField(self, body)

    def corridor_field(self, body: CollisionBody) -> Field:
        """The smallest corridor margin over ``body``'s probes, as a field."""
        # Lazy import: the state fields load on first use, not with the track
        from minilink.planning.spatial.state_fields import CorridorMarginField

        return CorridorMarginField(self, body)

    def plot(self, **kwargs):
        """Plot the centerline and corridor (lazy matplotlib import)."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.spatial.plotting import plot_track

        return plot_track(self, **kwargs)


###############################################################################
# Internal machinery
###############################################################################


def require_positive_half_width(half_width):
    """Refuse a corridor of zero or negative half-width."""
    if half_width <= 0.0:
        raise ValueError("half_width must be positive")
