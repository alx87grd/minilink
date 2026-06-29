"""Public shape primitives — the curated surface students and demos import.

Mirrors how ``dynamics.catalog`` re-exports curated plants: the classes live in
the internal ``graphical/animation/`` band; this module is the friendly, stable
re-export. Two things to note:

- ``Line`` is the public name for the internal ``CustomLine``.
- ``Arrow`` / ``TorqueArrow`` are the honest, frame-keyed primitives whose
  geometry is baked at construction (no column-norm side-channel scaling).

Placement helpers (``spring_between``, ``link_pose_3d``, …) live here — not in
``core/kinematics`` (rigid math only).
"""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    ExtrudedPolygon,
    HorizonPolyline,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    TrajectoryPolyline,
    ground_line,
    spring_line,
    vehicle_body,
    wheel_box,
)

Line = CustomLine  # public alias


def _as3(point):
    p = np.asarray(point, dtype=float).reshape(-1)
    if p.size == 2:
        return np.array([p[0], p[1], 0.0])
    return p[:3]


def _spring_coil_template(coils=6, amplitude=0.12):
    """Normalized coil samples ``(u along span, v lateral)`` in ``u ∈ [0, 1]``."""
    samples = [(0.0, 0.0), (0.15, 0.0)]
    xs = np.linspace(0.2, 0.8, 2 * coils + 1)
    for i, u in enumerate(xs):
        v = amplitude if i % 2 else -amplitude
        samples.append((float(u), float(v)))
    samples.append((0.85, 0.0))
    samples.append((1.0, 0.0))
    return samples


def _perp_axis(axis):
    axis = np.asarray(axis, dtype=float)
    length = np.linalg.norm(axis)
    if length < 1e-12:
        return np.array([0.0, 1.0, 0.0])
    axis = axis / length
    if axis.size == 2 or abs(axis[2]) < 1e-9:
        xy = axis[:2]
        n = np.linalg.norm(xy)
        if n < 1e-12:
            return np.array([0.0, 1.0, 0.0])
        return np.array([-xy[1] / n, xy[0] / n, 0.0])
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(axis, reference)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    perp = np.cross(reference, axis)
    return perp / (np.linalg.norm(perp) + 1e-12)


def spring_between(p0, p1, *, coils=6, amplitude=0.12, color="black", linewidth=1):
    """World-space spring coil from *p0* to *p1* (dynamic-tier geometry).

    Coil count and lateral amplitude stay fixed in world units; only span along
    the axis changes with compression/extension.
    """
    p0 = _as3(p0)
    p1 = _as3(p1)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    if length < 1e-12:
        return CustomLine(np.array([p0, p0]), color=color, linewidth=linewidth)
    perp = _perp_axis(axis)
    pts = []
    for u, v in _spring_coil_template(coils=coils, amplitude=amplitude):
        position = p0 + u * axis
        pts.append(position + v * perp)
    pts[0] = p0
    pts[-1] = p1
    return CustomLine(np.asarray(pts), color=color, linewidth=linewidth)


def line_segment(p0, p1, **style):
    """Honest world-space line segment (dynamic-tier geometry)."""
    p0 = _as3(p0)
    p1 = _as3(p1)
    return CustomLine(np.array([p0, p1]), **style)


def segment_pose_2d(p0, p1):
    """Pose a unit 2-D segment primitive to span *p0* → *p1* (static ``tf`` tier).

    Used for fixed-length chord lines where endpoints are known each frame.
    """
    p0 = np.asarray(p0, dtype=float).reshape(-1)[:2]
    p1 = np.asarray(p1, dtype=float).reshape(-1)[:2]
    delta = p1 - p0
    length = np.hypot(delta[0], delta[1])
    theta = np.arctan2(delta[1], delta[0])
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = length * c
    T[0, 1] = length * (-s)
    T[1, 0] = length * s
    T[1, 1] = length * c
    T[0, 3] = p0[0]
    T[1, 3] = p0[1]
    return T


def link_pose_3d(p0, p1):
    """Rigid pose for a unit ``Rod`` (length on primitive) from *p0* toward *p1*."""
    p0 = np.asarray(p0, dtype=float).reshape(-1)
    p1 = np.asarray(p1, dtype=float).reshape(-1)
    if p0.size == 2:
        p0 = np.array([p0[0], p0[1], 0.0])
    if p1.size == 2:
        p1 = np.array([p1[0], p1[1], 0.0])
    delta = p1 - p0
    length = np.linalg.norm(delta)
    T = np.eye(4)
    T[:3, 3] = p0
    if length < 1e-12:
        return T

    y_axis = -delta / length
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(y_axis, reference)) > 0.95:
        reference = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(reference, y_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, y_axis)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis
    return T


def point_pose(p):
    """Pure translation placing a primitive at *point*."""
    p = _as3(p)
    T = np.eye(4)
    T[:3, 3] = p
    return T


__all__ = [
    "Arrow",
    "Box",
    "Circle",
    "ExtrudedPolygon",
    "HorizonPolyline",
    "Line",
    "Plane",
    "Point",
    "Rod",
    "Sphere",
    "TorqueArrow",
    "TrajectoryPolyline",
    "ground_line",
    "spring_line",
    "spring_between",
    "line_segment",
    "segment_pose_2d",
    "link_pose_3d",
    "point_pose",
    "vehicle_body",
    "wheel_box",
]
