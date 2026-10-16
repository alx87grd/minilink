"""Track overlays: a reference track's corridor drawn into a 2-D animation."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.drawables import Overlay
from minilink.graphical.animation.primitives import CustomLine
from minilink.graphical.animation.visualization import WORLD

###############################################################################
# Public API
###############################################################################


class TrackCorridorOverlay(Overlay):
    """Corridor edge and dashed centerline polylines for 2-D animation."""

    def __init__(
        self,
        track,
        *,
        n_samples: int = 200,
        corridor_edge: str = "#9aa3af",
        centerline: str = "#5c6570",
    ):
        # The centerline and both corridor edges, sampled along the track
        center, upper, lower = track_boundaries(track, n_samples=n_samples)

        # Graphics: the edges solid, the centerline dashed, in the world frame
        self._geometry = {
            WORLD: [
                CustomLine(
                    line3(upper),
                    color=corridor_edge,
                    linewidth=1.6,
                    style="-",
                ),
                CustomLine(
                    line3(lower),
                    color=corridor_edge,
                    linewidth=1.6,
                    style="-",
                ),
                CustomLine(
                    line3(center),
                    color=centerline,
                    linewidth=1.1,
                    style=(0, (5, 4)),
                ),
            ]
        }

    def tf(self, t=0.0, params=None):
        return {}

    def get_kinematic_geometry(self):
        return dict(self._geometry)


###############################################################################
# Internal machinery
###############################################################################


def track_boundaries(track, n_samples: int = 200):
    """Return centerline and corridor upper/lower edges for a 2-D track."""
    path = track.path
    require_planar_path(path)

    # The centerline c(s) and its unit tangent τ(s), sampled along the arc length
    ss = np.linspace(0.0, path.total_length, n_samples)
    center = np.array([path.sample(s) for s in ss])
    tangents = np.array([path.tangent(s) for s in ss])

    # The left normal n = (−τ_y, τ_x), and the edges c ± w n at the half-width w
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)
    half = float(track.half_width)
    upper = center + half * normals
    lower = center - half * normals

    return center, upper, lower


def require_planar_path(path):
    """Refuse a track whose path does not lie in the plane."""
    if path.workspace_dim != 2:
        raise ValueError("TrackCorridorOverlay supports 2-D paths only")


def line3(xy: np.ndarray) -> np.ndarray:
    """The planar points ``xy`` as 3-D points on the plane ``z = 0``."""
    z = np.zeros((xy.shape[0], 1))
    xyz = np.hstack([xy, z])

    return xyz
