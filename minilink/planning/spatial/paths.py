"""
Workspace reference paths for path tracking and corridor constraints.

Build a continuous primitive from waypoints with :func:`from_waypoints` (default
``kind="polyline"``) and pass it to :class:`~minilink.planning.spatial.track.ReferenceTrack`.
:func:`circuit_waypoints` writes a closed test loop to track.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module

# Public API


class ReferencePath(ABC):
    """Curve in workspace; distance is always nonnegative length units."""

    @property
    @abstractmethod
    def workspace_dim(self) -> int: ...

    @property
    @abstractmethod
    def total_length(self) -> float:
        """Arc length of the path."""
        ...

    @abstractmethod
    def distance(self, p, t=0.0, params=None):
        """Shortest distance from workspace point ``p`` to the path."""
        ...

    @abstractmethod
    def project(self, p, t=0.0, params=None):
        """Return ``(arc_length, closest_point)`` on the path."""
        ...

    @abstractmethod
    def sample(self, s, t=0.0, params=None):
        """Point at arc length ``s`` along the path."""
        ...

    @abstractmethod
    def tangent(self, s, t=0.0, params=None):
        """Unit tangent at arc length ``s``."""
        ...


@dataclass(frozen=True)
class PolylinePath(ReferencePath):
    """
    Piecewise-linear path through waypoints (C⁰).

    Parameters
    ----------
    waypoints : array_like, shape (N, d)
        Vertices with ``N >= 2``.
    """

    waypoints: np.ndarray

    def __post_init__(self) -> None:
        wp = np.asarray(self.waypoints, dtype=float)
        if wp.ndim != 2 or wp.shape[0] < 2:
            raise ValueError("PolylinePath requires at least two waypoints")
        object.__setattr__(self, "waypoints", wp)
        seg = np.linalg.norm(np.diff(wp, axis=0), axis=1)
        s_knots = np.concatenate([[0.0], np.cumsum(seg)])
        object.__setattr__(self, "_seg_lengths", seg)
        object.__setattr__(self, "_s_knots", s_knots)
        object.__setattr__(self, "_total_length", float(s_knots[-1]))

    @property
    def workspace_dim(self) -> int:
        return int(self.waypoints.shape[1])

    @property
    def total_length(self) -> float:
        return self._total_length

    def distance(self, p, t=0.0, params=None):
        xp = array_module(p)
        p = xp.asarray(p, dtype=float).reshape(-1)
        verts = xp.asarray(self.waypoints, dtype=float)
        a = verts[:-1]
        b = verts[1:]
        ab = b - a
        ap = p - a
        denom = xp.sum(ab * ab, axis=-1)
        tau = xp.clip(xp.sum(ap * ab, axis=-1) / xp.maximum(denom, 1e-12), 0.0, 1.0)
        closest = a + ab * tau[:, xp.newaxis]
        dist = xp.linalg.norm(p - closest, axis=-1)
        return xp.min(dist)

    def project(self, p, t=0.0, params=None):
        dist, s, closest = self._closest_numpy(p)
        return s, closest

    def sample(self, s, t=0.0, params=None):
        xp = array_module(s)
        s_val = float(xp.asarray(s).reshape(-1)[0])
        return self._point_at_arc_length(s_val)

    def tangent(self, s, t=0.0, params=None):
        xp = array_module(s)
        s_val = float(xp.asarray(s).reshape(-1)[0])
        seg_idx = self._segment_index(s_val)
        a = self.waypoints[seg_idx]
        b = self.waypoints[seg_idx + 1]
        direction = b - a
        length = float(np.linalg.norm(direction))
        if length <= 0.0:
            return np.ones(self.workspace_dim) / np.sqrt(self.workspace_dim)
        return direction / length

    def _closest_numpy(self, p):
        p = np.asarray(p, dtype=float).reshape(-1)
        verts = self.waypoints
        a = verts[:-1]
        b = verts[1:]
        ab = b - a
        ap = p - a
        denom = np.sum(ab * ab, axis=-1)
        tau = np.clip(np.sum(ap * ab, axis=-1) / np.maximum(denom, 1e-12), 0.0, 1.0)
        closest = a + ab * tau[:, np.newaxis]
        dist = np.linalg.norm(p - closest, axis=-1)
        idx = int(np.argmin(dist))
        seg_len = float(self._seg_lengths[idx])
        s = float(self._s_knots[idx] + float(tau[idx]) * seg_len)
        return float(dist[idx]), s, closest[idx].copy()

    def _segment_index(self, s: float) -> int:
        s = float(np.clip(s, 0.0, self._total_length))
        idx = int(np.searchsorted(self._s_knots, s, side="right") - 1)
        return min(max(idx, 0), len(self._seg_lengths) - 1)

    def _point_at_arc_length(self, s: float) -> np.ndarray:
        if self._total_length <= 0.0:
            return self.waypoints[0].copy()
        s = float(np.clip(s, 0.0, self._total_length))
        idx = self._segment_index(s)
        seg_len = float(self._seg_lengths[idx])
        if seg_len <= 0.0:
            return self.waypoints[idx].copy()
        tau = (s - float(self._s_knots[idx])) / seg_len
        a = self.waypoints[idx]
        b = self.waypoints[idx + 1]
        return a + tau * (b - a)


def from_waypoints(waypoints, *, kind: str = "polyline") -> ReferencePath:
    """
    Build a reference path from a waypoint polyline.

    Parameters
    ----------
    waypoints : array_like, shape (N, d)
        Path vertices with ``N >= 2``.
    kind : {"polyline"}
        Path primitive; default is the robust piecewise-linear polyline.
    """
    if kind == "polyline":
        return PolylinePath(np.asarray(waypoints, dtype=float))
    raise ValueError(f"unknown path kind {kind!r}; supported: 'polyline'")


def circuit_waypoints(length=14.0, width=9.0, radius=2.5, spacing=0.25):
    """Closed test circuit: a rounded rectangle, sampled counter-clockwise.

    Four quarter turns joined by four straights, then resampled at a constant arc-length
    spacing so that a tracker's lookahead always lands between two waypoints.

    Parameters
    ----------
    length, width : float
        Outer dimensions of the loop [m], corner to corner.
    radius : float
        Corner radius [m].
    spacing : float
        Distance between consecutive waypoints [m].

    Returns
    -------
    waypoints : array of shape (N, 2)
        Polyline [m] going once around the loop. The first point is not repeated at
        the end: a zero-length closing segment has no tangent, and the path objects
        that consume waypoints divide by it.
    """
    half_l, half_w = 0.5 * length - radius, 0.5 * width - radius
    centers = [
        (half_l, half_w, 0.0),
        (-half_l, half_w, 0.5),
        (-half_l, -half_w, 1.0),
        (half_l, -half_w, 1.5),
    ]

    corners = []
    for cx, cy, quarter in centers:
        angles = np.pi * (quarter + np.linspace(0.0, 0.5, 16))
        corners.append(
            np.column_stack(
                [cx + radius * np.cos(angles), cy + radius * np.sin(angles)]
            )
        )
    loop = np.vstack([np.vstack(corners), np.vstack(corners)[:1]])

    # constant spacing: walk the closed polyline by arc length, and stretch the step so
    # that a whole number of them goes around (the closing gap is then one step too)
    steps = np.linalg.norm(np.diff(loop, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(steps)])
    n_samples = max(int(round(s[-1] / spacing)), 8)
    s_uniform = np.linspace(0.0, s[-1], n_samples, endpoint=False)
    waypoints = np.column_stack(
        [np.interp(s_uniform, s, loop[:, 0]), np.interp(s_uniform, s, loop[:, 1])]
    )

    return waypoints
