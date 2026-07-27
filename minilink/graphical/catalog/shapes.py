"""Public shape primitives — the curated surface students and demos import.

Mirrors how ``dynamics.catalog`` re-exports curated plants: the classes live in
the internal ``graphical/animation/`` band; this module is the friendly, stable
re-export. Two things to note:

- ``Line`` is the public name for the internal ``CustomLine``.
- ``Arrow`` / ``TorqueArrow`` bake full geometry at construction in the
  primitive's local frame.

Placement helpers (``spring_between``, ``link_pose_3d``, …) live here — not in
``core/kinematics`` (rigid math only).
"""

from __future__ import annotations

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE3
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
    """Rigid pose for a unit ``Rod`` (length on primitive) from *p0* toward *p1*.

    Native-array: NumPy or JAX depending on the input backend. Near-zero span
    collapses to a pure translation at *p0*.
    """
    xp = array_module(p0, p1)
    p0 = xp.asarray(p0, dtype=float).reshape(-1)
    p1 = xp.asarray(p1, dtype=float).reshape(-1)
    if p0.shape[0] == 2:
        p0 = xp.concatenate([p0, xp.asarray([0.0])])
    if p1.shape[0] == 2:
        p1 = xp.concatenate([p1, xp.asarray([0.0])])

    delta = p1 - p0
    length = xp.linalg.norm(delta)
    safe = xp.maximum(length, 1e-12)
    y_axis = -delta / safe
    ref_z = xp.asarray([0.0, 0.0, 1.0])
    ref_x = xp.asarray([1.0, 0.0, 0.0])
    reference = xp.where(xp.abs(xp.dot(y_axis, ref_z)) > 0.95, ref_x, ref_z)
    x_axis = xp.cross(reference, y_axis)
    x_axis = x_axis / xp.linalg.norm(x_axis)
    z_axis = xp.cross(x_axis, y_axis)
    R = xp.stack([x_axis, y_axis, z_axis], axis=1)
    posed = SE3(R, p0)
    collapsed = SE3(xp.eye(3), p0)
    return xp.where(length < 1e-12, collapsed, posed)


def point_pose(p):
    """Pure translation placing a primitive at *point*.

    Native-array: NumPy or JAX depending on the input backend. Length-2 points
    are lifted to the ``z = 0`` plane.
    """
    xp = array_module(p)
    p = xp.asarray(p, dtype=float).reshape(-1)
    if p.shape[0] == 2:
        p = xp.concatenate([p, xp.asarray([0.0])])
    else:
        p = p[:3]
    return SE3(xp.eye(3), p)


def plane_airframe_3d(
    length=2.0,
    width=0.2,
    l_cg=None,
    span=2.0,
    chord_w=0.35,
    chord_t=0.2,
    span_t=0.7,
    fin_height=0.35,
    l_w=0.0,
    l_t=None,
    color="#2f6fed",
    wing_color="#4c8cff",
    tail_color="#3a7af0",
):
    """Body-frame solid airframe for meshcat (c.g. at the origin).

    Axes match a standard aircraft body frame: ``+x`` forward, ``+y`` right,
    ``+z`` down. The fuselage is an :class:`ExtrudedPolygon` side silhouette
    (local XY = body ``x``–up, extruded along body ``y``); wing / horizontal
    tail / vertical fin are thin :class:`Box` solids.
    """
    from minilink.core.kinematics import Rx

    length = float(length)
    width = float(width)
    l_cg = 0.6 * length if l_cg is None else float(l_cg)
    l_t = 0.45 * length if l_t is None else float(l_t)
    span = float(span)
    chord_w = float(chord_w)
    chord_t = float(chord_t)
    span_t = float(span_t)
    fin_height = float(fin_height)
    l_w = float(l_w)
    half_h = 0.55 * width

    nose = length - l_cg
    tail = -l_cg
    # Side silhouette in ExtrudedPolygon XY: +X forward, +Y toward body -Z (up).
    fuselage_pts = np.array(
        [
            [nose, 0.0],
            [0.72 * nose, half_h],
            [0.15 * nose, half_h],
            [tail + 0.12 * length, 0.45 * half_h],
            [tail, 0.2 * half_h],
            [tail, -0.25 * half_h],
            [0.15 * nose, -0.65 * half_h],
            [0.72 * nose, -0.45 * half_h],
        ]
    )
    fuselage = ExtrudedPolygon(fuselage_pts, height=width, color=color, opacity=1.0)
    # Map ExtrudedPolygon (X, Y, Z_extrude) → body (X, Y_right, Z_down).
    fuselage.local_transform = SE3(Rx(-0.5 * np.pi), 0.0)

    wing = Box(
        length_x=chord_w,
        length_y=span,
        length_z=max(0.06 * width, 0.03),
        center=(0.0, 0.0, 0.0),
        color=wing_color,
        opacity=1.0,
    )
    wing.local_transform = SE3(np.eye(3), np.array([-l_w, 0.0, 0.0]))

    h_tail = Box(
        length_x=chord_t,
        length_y=span_t,
        length_z=max(0.05 * width, 0.025),
        center=(0.0, 0.0, 0.0),
        color=tail_color,
        opacity=1.0,
    )
    h_tail.local_transform = SE3(np.eye(3), np.array([-l_t, 0.0, 0.0]))

    v_fin = Box(
        length_x=chord_t,
        length_y=0.03 * width,
        length_z=fin_height,
        center=(0.0, 0.0, 0.0),
        color=tail_color,
        opacity=1.0,
    )
    # Fin sits above the boom (body -Z); box center at half height.
    v_fin.local_transform = SE3(np.eye(3), np.array([-l_t, 0.0, -0.5 * fin_height]))

    return [fuselage, wing, h_tail, v_fin]


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
    "plane_airframe_3d",
    "vehicle_body",
    "wheel_box",
]
