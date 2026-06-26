"""
Reference tracks: a path plus an optional corridor for planning exports.

Compose at :class:`~minilink.planning.problems.PlanningProblem`::

    track = ReferenceTrack(from_waypoints(pts), half_width=1.0)
    X = bounds & track.corridor_field(robot).as_constraint()
    cost = base + track.distance_field(robot).as_cost(weight=5.0)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.backends import array_module
from minilink.planning.spatial.paths import ReferencePath
from minilink.planning.spatial.robot import RobotBody

if TYPE_CHECKING:
    from minilink.planning.spatial.state_fields import StateField

# Public API


@dataclass(frozen=True)
class ReferenceTrack:
    """
    Target path with a constant half-width corridor in workspace units.

    Parameters
    ----------
    path : ReferencePath
        Workspace centerline.
    half_width : float
        Corridor radius around the path (same units as ``path.distance``).
    """

    path: ReferencePath
    half_width: float = 1.0

    def __post_init__(self) -> None:
        if self.half_width <= 0.0:
            raise ValueError("half_width must be positive")

    def distance(self, p, t=0.0, params=None):
        """Distance from ``p`` to the centerline."""
        return self.path.distance(p, t=t, params=params)

    def corridor_margin(self, p, t=0.0, params=None):
        """Positive when ``p`` lies inside the corridor: ``half_width - distance``."""
        xp = array_module(p)
        return xp.asarray(self.half_width) - self.path.distance(p, t=t, params=params)

    def sample_boundaries(self, n_samples=200):
        """
        Sample the centerline and corridor edges along the path.

        Returns ``(center, upper, lower)``, each an ``(n_samples, 2)`` array.
        This is the single source for the corridor geometry drawn by both
        :func:`~minilink.planning.spatial.plotting.plot_track` and the animation
        ``SceneOverlay``, so the plot and the animation never drift apart.
        """
        if self.path.workspace_dim != 2:
            raise ValueError("sample_boundaries supports 2-D paths only")

        ss = np.linspace(0.0, self.path.total_length, n_samples)
        center = np.array([self.path.sample(s) for s in ss])
        tangents = np.array([self.path.tangent(s) for s in ss])
        normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)

        half = float(self.half_width)
        return center, center + half * normals, center - half * normals

    def distance_field(self, robot: RobotBody) -> StateField:
        from minilink.planning.spatial.state_fields import PathDistanceField

        return PathDistanceField(self, robot)

    def corridor_field(self, robot: RobotBody) -> StateField:
        from minilink.planning.spatial.state_fields import CorridorMarginField

        return CorridorMarginField(self, robot)

    def plot(self, **kwargs):
        """Plot the centerline and corridor (lazy matplotlib import)."""
        from minilink.planning.spatial.plotting import plot_track

        return plot_track(self, **kwargs)
