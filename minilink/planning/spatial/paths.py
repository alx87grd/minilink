"""
Workspace reference paths for path tracking and corridor constraints.

Build a continuous primitive from waypoints with :func:`from_waypoints` (default
``kind="polyline"``) and pass it to :class:`~minilink.planning.spatial.track.ReferenceTrack`.
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
    def workspace_dim(self) -> int:
        ...

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
