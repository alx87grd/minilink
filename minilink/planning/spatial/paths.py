"""Reference paths: workspace curves for path tracking and corridor constraints."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module

###############################################################################
# Public API
###############################################################################


class ReferencePath(ABC):
    """
    Curve in workspace; distance is always nonnegative length units.

    Build one from waypoints with :func:`from_waypoints` (default
    ``kind="polyline"``) and pass it to
    :class:`~minilink.planning.spatial.track.ReferenceTrack`.
    """

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
        # The vertices, as an (N, d) float array of at least two points
        wp = np.asarray(self.waypoints, dtype=float)
        require_polyline(wp)
        object.__setattr__(self, "waypoints", wp)

        # Arc length at each vertex: s₀ = 0, s_{i+1} = sᵢ + ℓᵢ, ℓᵢ = |w_{i+1} − wᵢ|
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

        verts = xp.asarray(self.waypoints, dtype=float)

        p = xp.asarray(p, dtype=float).reshape(-1)
        a = verts[:-1]
        b = verts[1:]
        ab = b - a
        ap = p - a

        # τᵢ = clip((p − aᵢ)·(bᵢ − aᵢ) / max(|bᵢ − aᵢ|², 10⁻¹²), 0, 1)
        # cᵢ = aᵢ + τᵢ (bᵢ − aᵢ), the closest point of each segment
        denom = xp.sum(ab * ab, axis=-1)
        tau = xp.clip(xp.sum(ap * ab, axis=-1) / xp.maximum(denom, 1e-12), 0.0, 1.0)
        closest = a + ab * tau[:, xp.newaxis]

        # d(p) = minᵢ |p − cᵢ|
        dist = xp.linalg.norm(p - closest, axis=-1)
        d = xp.min(dist)

        return d

    def project(self, p, t=0.0, params=None):
        xp = array_module(p)

        verts = xp.asarray(self.waypoints, dtype=float)
        s_knots = xp.asarray(self._s_knots, dtype=float)
        seg = xp.asarray(self._seg_lengths, dtype=float)

        p = xp.asarray(p, dtype=float).reshape(-1)
        a = verts[:-1]
        b = verts[1:]
        ab = b - a
        ap = p - a

        # τᵢ = clip((p − aᵢ)·(bᵢ − aᵢ) / max(|bᵢ − aᵢ|², 10⁻¹²), 0, 1)
        # cᵢ = aᵢ + τᵢ (bᵢ − aᵢ), the closest point of each segment
        denom = xp.sum(ab * ab, axis=-1)
        tau = xp.clip(xp.sum(ap * ab, axis=-1) / xp.maximum(denom, 1e-12), 0.0, 1.0)
        closest = a + ab * tau[:, xp.newaxis]

        # The nearest segment i = argminᵢ |p − cᵢ|, at arc length s = sᵢ + τᵢ ℓᵢ
        dist = xp.linalg.norm(p - closest, axis=-1)
        i = xp.argmin(dist)
        s = s_knots[i] + tau[i] * seg[i]
        c = closest[i]

        return s, c

    def sample(self, s, t=0.0, params=None):
        xp = array_module(s)

        total_length = self._total_length
        s_knots = xp.asarray(self._s_knots, dtype=float)
        seg = xp.asarray(self._seg_lengths, dtype=float)
        verts = xp.asarray(self.waypoints, dtype=float)

        s = xp.clip(xp.asarray(s, dtype=float), 0.0, total_length)
        n_seg = verts.shape[0] - 1

        # The segment i with sᵢ ≤ s < s_{i+1}, and τ = (s − sᵢ) / max(ℓᵢ, 10⁻¹²)
        idx = xp.clip(xp.searchsorted(s_knots, s, side="right") - 1, 0, n_seg - 1)
        tau = (s - s_knots[idx]) / xp.maximum(seg[idx], 1e-12)

        # w(s) = wᵢ + τ (w_{i+1} − wᵢ)
        w = verts[idx] + tau * (verts[idx + 1] - verts[idx])

        return w

    def tangent(self, s, t=0.0, params=None):
        xp = array_module(s)

        total_length = self._total_length
        s_knots = xp.asarray(self._s_knots, dtype=float)
        verts = xp.asarray(self.waypoints, dtype=float)
        dim = self.workspace_dim

        s = xp.clip(xp.asarray(s, dtype=float), 0.0, total_length)
        n_seg = verts.shape[0] - 1

        # The segment i with sᵢ ≤ s < s_{i+1}
        idx = xp.clip(xp.searchsorted(s_knots, s, side="right") - 1, 0, n_seg - 1)

        # T = (w_{i+1} − wᵢ) / |w_{i+1} − wᵢ|, or 1/√d on each axis when ℓᵢ = 0
        direction = verts[idx + 1] - verts[idx]
        length = xp.linalg.norm(direction)
        fallback = xp.ones(direction.shape) / xp.sqrt(xp.asarray(dim))
        T = xp.where(length > 0.0, direction / xp.maximum(length, 1e-12), fallback)

        return T


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
    require_path_kind(kind)

    return PolylinePath(np.asarray(waypoints, dtype=float))


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
    # The corner centres, inset by the radius, and the quarter turn each arc starts at
    half_l = 0.5 * length - radius
    half_w = 0.5 * width - radius
    centers = [
        (half_l, half_w, 0.0),
        (-half_l, half_w, 0.5),
        (-half_l, -half_w, 1.0),
        (half_l, -half_w, 1.5),
    ]

    # Each corner arc: (cx + r cos φ, cy + r sin φ), φ = π (q + [0, ½]) on 16 points
    corners = []
    for cx, cy, quarter in centers:
        angles = np.pi * (quarter + np.linspace(0.0, 0.5, 16))
        x = cx + radius * np.cos(angles)
        y = cy + radius * np.sin(angles)
        corners.append(np.column_stack([x, y]))

    # The closed loop: the arcs, joined by the straights, back to the first point
    loop = np.vstack([np.vstack(corners), np.vstack(corners)[:1]])

    # Constant spacing: walk the closed polyline by arc length, and stretch the step so
    # that a whole number of them goes around (the closing gap is then one step too)
    steps = np.linalg.norm(np.diff(loop, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(steps)])
    n_samples = max(int(round(s[-1] / spacing)), 8)
    s_uniform = np.linspace(0.0, s[-1], n_samples, endpoint=False)
    x_uniform = np.interp(s_uniform, s, loop[:, 0])
    y_uniform = np.interp(s_uniform, s, loop[:, 1])
    waypoints = np.column_stack([x_uniform, y_uniform])

    return waypoints


###############################################################################
# Internal machinery
###############################################################################


def require_path_kind(kind):
    """Refuse a path kind other than the polyline."""
    if kind != "polyline":
        raise ValueError(f"unknown path kind {kind!r}; supported: 'polyline'")


def require_polyline(wp):
    """Refuse waypoints that are not an ``(N, d)`` array of at least two points."""
    if wp.ndim != 2 or wp.shape[0] < 2:
        raise ValueError("PolylinePath requires at least two waypoints")
