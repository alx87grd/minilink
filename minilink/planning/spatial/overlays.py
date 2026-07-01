"""Animation overlays for planning spatial geometry."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.drawables import Overlay
from minilink.graphical.animation.primitives import CustomLine
from minilink.graphical.animation.visualization import WORLD


def _track_boundaries(track, n_samples: int = 200):
    """Return centerline and corridor upper/lower edges for a 2-D track."""
    path = track.path
    if path.workspace_dim != 2:
        raise ValueError("TrackCorridorOverlay supports 2-D paths only")

    ss = np.linspace(0.0, path.total_length, n_samples)
    center = np.array([path.sample(s) for s in ss])
    tangents = np.array([path.tangent(s) for s in ss])
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)
    half = float(track.half_width)
    upper = center + half * normals
    lower = center - half * normals
    return center, upper, lower


def _line3(xy: np.ndarray) -> np.ndarray:
    return np.hstack([xy, np.zeros((xy.shape[0], 1))])


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
        center, upper, lower = _track_boundaries(track, n_samples=n_samples)
        self._geometry = {
            WORLD: [
                CustomLine(
                    _line3(upper),
                    color=corridor_edge,
                    linewidth=1.6,
                    style="-",
                ),
                CustomLine(
                    _line3(lower),
                    color=corridor_edge,
                    linewidth=1.6,
                    style="-",
                ),
                CustomLine(
                    _line3(center),
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
