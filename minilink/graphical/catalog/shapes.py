"""Shape primitives and the placement helpers that pose them in a frame."""

from __future__ import annotations

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE3, Rx
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
    point3,
    spring_line,
    vehicle_body,
    wheel_box,
)

# The curated surface students and demos import, as ``dynamics.catalog`` re-exports
# its plants: the primitives live in the internal ``graphical/animation/`` band, and
# this module is their friendly, stable re-export.
Line = CustomLine  # The public name of the internal ``CustomLine``


###############################################################################
# Public API
###############################################################################


# The placement helpers pose a primitive in a frame; they live here, not in
# ``core/kinematics``, which keeps the rigid-body math only


def spring_between(p0, p1, *, coils=6, amplitude=0.12, color="black", linewidth=1):
    """World-space spring coil from *p0* to *p1* (dynamic-tier geometry).

    Coil count and lateral amplitude stay fixed in world units; only span along
    the axis changes with compression/extension.
    """
    p0 = point3(p0)
    p1 = point3(p1)

    # The spring's axis; a zero-length spring is drawn as a point
    axis = p1 - p0
    length = np.linalg.norm(axis)
    if length < 1e-12:
        points = np.array([p0, p0])
        return CustomLine(points, color=color, linewidth=linewidth)

    # p = p0 + u (p1 − p0) + v n, with n ⟂ (p1 − p0): the coil template along the axis
    perp = perp_axis(axis)
    pts = []
    for u, v in spring_coil_template(coils=coils, amplitude=amplitude):
        position = p0 + u * axis
        pts.append(position + v * perp)

    # The end points pinned exactly on p0 and p1
    pts[0] = p0
    pts[-1] = p1
    points = np.asarray(pts)

    return CustomLine(points, color=color, linewidth=linewidth)


def segment_pose_2d(p0, p1):
    """Pose a unit 2-D segment primitive to span *p0* → *p1* (static ``tf`` tier).

    Used for fixed-length chord lines where endpoints are known each frame.
    Native-array: NumPy or JAX depending on the input backend.
    """
    xp = array_module(p0, p1)

    p0 = xp.asarray(p0, dtype=float).reshape(-1)[:2]
    p1 = xp.asarray(p1, dtype=float).reshape(-1)[:2]

    # The segment's length ℓ and heading θ
    delta = p1 - p0
    length = xp.hypot(delta[0], delta[1])
    theta = xp.arctan2(delta[1], delta[0])
    c = xp.cos(theta)
    s = xp.sin(theta)

    # T = [ℓ R(θ), p0; 0, 1]: the unit segment stretched to ℓ, turned by θ, placed at p0
    T11 = length * c
    T12 = length * (-s)
    T21 = length * s
    T22 = length * c

    # fmt: off
    T = xp.array([
        [T11, T12, 0.0, p0[0]],
        [T21, T22, 0.0, p0[1]],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    # fmt: on

    return T


def link_pose_3d(p0, p1):
    """Rigid pose for a unit ``Rod`` (length on primitive) from *p0* toward *p1*.

    Native-array: NumPy or JAX depending on the input backend. Near-zero span
    collapses to a pure translation at *p0*.
    """
    xp = array_module(p0, p1)

    # The end points, planar ones lifted to z = 0
    p0 = xp.asarray(p0, dtype=float).reshape(-1)
    p1 = xp.asarray(p1, dtype=float).reshape(-1)
    if p0.shape[0] == 2:
        p0 = xp.concatenate([p0, xp.asarray([0.0])])
    if p1.shape[0] == 2:
        p1 = xp.concatenate([p1, xp.asarray([0.0])])

    # y = −(p1 − p0) / ℓ: the rod lies along its local −y, from p0 toward p1
    delta = p1 - p0
    length = xp.linalg.norm(delta)
    safe = xp.maximum(length, 1e-12)
    y_axis = -delta / safe

    # x = r × y / ‖r × y‖ and z = x × y, the reference r = ẑ, or x̂ when y is near ẑ
    ref_z = xp.asarray([0.0, 0.0, 1.0])
    ref_x = xp.asarray([1.0, 0.0, 0.0])
    reference = xp.where(xp.abs(xp.dot(y_axis, ref_z)) > 0.95, ref_x, ref_z)
    x_axis = xp.cross(reference, y_axis)
    x_axis = x_axis / xp.linalg.norm(x_axis)
    z_axis = xp.cross(x_axis, y_axis)
    R = xp.stack([x_axis, y_axis, z_axis], axis=1)

    # T = (R, p0), or the pure translation (I, p0) when the span collapses
    posed = SE3(R, p0)
    collapsed = SE3(xp.eye(3), p0)
    T = xp.where(length < 1e-12, collapsed, posed)

    return T


def point_pose(p):
    """Pure translation placing a primitive at *p*.

    Native-array: NumPy or JAX depending on the input backend. Length-2 points
    are lifted to the ``z = 0`` plane.
    """
    xp = array_module(p)

    # The point, a planar one lifted to z = 0
    p = xp.asarray(p, dtype=float).reshape(-1)
    if p.shape[0] == 2:
        p = xp.concatenate([p, xp.asarray([0.0])])
    else:
        p = p[:3]

    # T = (I, p)
    T = SE3(xp.eye(3), p)

    return T


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
    # Dimensions, the c.g. and tail arms defaulting to fractions of the length
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

    # Fuselage: its side silhouette in the ExtrudedPolygon's XY plane, +X forward
    # and +Y toward body −Z (up)
    half_h = 0.55 * width
    nose = length - l_cg
    tail = -l_cg
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

    # The extrusion frame (X, Y, Z_extrude) mapped onto the body (X, Y_right, Z_down)
    fuselage.local_transform = SE3(Rx(-0.5 * np.pi), 0.0)

    # Wing, centered l_w behind the c.g.
    wing = Box(
        length_x=chord_w,
        length_y=span,
        length_z=max(0.06 * width, 0.03),
        center=(0.0, 0.0, 0.0),
        color=wing_color,
        opacity=1.0,
    )
    wing.local_transform = SE3(np.eye(3), np.array([-l_w, 0.0, 0.0]))

    # Horizontal tail, centered l_t behind the c.g.
    h_tail = Box(
        length_x=chord_t,
        length_y=span_t,
        length_z=max(0.05 * width, 0.025),
        center=(0.0, 0.0, 0.0),
        color=tail_color,
        opacity=1.0,
    )
    h_tail.local_transform = SE3(np.eye(3), np.array([-l_t, 0.0, 0.0]))

    # Vertical fin above the boom (body −Z), its box centered at half height
    v_fin = Box(
        length_x=chord_t,
        length_y=0.03 * width,
        length_z=fin_height,
        center=(0.0, 0.0, 0.0),
        color=tail_color,
        opacity=1.0,
    )
    v_fin.local_transform = SE3(np.eye(3), np.array([-l_t, 0.0, -0.5 * fin_height]))

    return [fuselage, wing, h_tail, v_fin]


###############################################################################
# Internal machinery
###############################################################################


def perp_axis(axis):
    """A unit vector normal to *axis*, in the XY plane when *axis* lies in it."""
    axis = np.asarray(axis, dtype=float)
    length = np.linalg.norm(axis)

    # A zero axis has no direction: ŷ by default
    if length < 1e-12:
        perp = np.array([0.0, 1.0, 0.0])
        return perp

    axis = axis / length

    # An axis in the XY plane: its in-plane normal n = (−a_y, a_x, 0) / ‖(a_x, a_y)‖
    if axis.size == 2 or abs(axis[2]) < 1e-9:
        xy = axis[:2]
        length_xy = np.linalg.norm(xy)
        if length_xy < 1e-12:
            perp = np.array([0.0, 1.0, 0.0])
            return perp

        perp = np.array([-xy[1] / length_xy, xy[0] / length_xy, 0.0])
        return perp

    # Any other axis: n = r × a / ‖r × a‖, the reference r = ẑ, or ŷ when a is near ẑ
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(axis, reference)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    cross = np.cross(reference, axis)
    perp = cross / (np.linalg.norm(cross) + 1e-12)

    return perp


def spring_coil_template(coils=6, amplitude=0.12):
    """Normalized coil samples ``(u along span, v lateral)`` in ``u ∈ [0, 1]``."""
    # The lead in
    samples = [(0.0, 0.0), (0.15, 0.0)]

    # The coils: 2 coils + 1 vertices over u ∈ [0.2, 0.8], alternating −v and +v
    xs = np.linspace(0.2, 0.8, 2 * coils + 1)
    for i, u in enumerate(xs):
        v = amplitude if i % 2 else -amplitude
        samples.append((float(u), float(v)))

    # The lead out
    samples.append((0.85, 0.0))
    samples.append((1.0, 0.0))

    return samples


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
    "segment_pose_2d",
    "link_pose_3d",
    "point_pose",
    "plane_airframe_3d",
    "vehicle_body",
    "wheel_box",
]
