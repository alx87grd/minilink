"""Meshcat WebGL backend."""

from __future__ import annotations

import time
from pathlib import Path

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    ExtrudedPolygon,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
)
from minilink.graphical.animation.renderers.renderer import AnimationRenderer
from minilink.graphical.common.environment import is_blocking_needed


def html_export_path(file_name) -> Path:
    """Destination of ``animate(save=True, renderer="meshcat")``.

    ``Animation`` becomes ``Animation.html``. A name that already ends in
    ``.html`` is left as written, so a project can pass a full page path.
    """
    path = Path(file_name)
    if path.suffix.lower() != ".html":
        path = Path(str(file_name) + ".html")
    return path


def _import_meshcat():
    try:
        import meshcat
    except ImportError as e:
        raise ImportError(
            "meshcat is required for renderer='meshcat'. "
            "Install with: pip install 'minilink[visualization]'"
        ) from e
    return meshcat


def _color_to_meshcat_hex(color) -> int:
    r, g, b = mcolors.to_rgb(color)
    return (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255)


# The camera 4x4 (``camera_matrix``) in the viewer. The mouse orbit is anchored
# at the viewer origin, so the look-at target is honoured by sliding the drawn
# world, with the viewer's grid and axes, by minus the target; the eye sits on
# the view-out side of the origin at the ``T[3, 3]`` distance, as the position
# of the viewer's own camera under ``/Cameras/default/rotated``. Orbit, pan and
# zoom stay with the mouse.
_CAMERA_EYE = "/Cameras/default/rotated/<object>"
_VIEWER_FURNITURE = ("/Grid", "/Axes")
# A top-down hint would put the eye on the orbit's pole, where the view is
# undefined; the elevation is capped at the viewer's default eye ``(3, 1, 0)``.
_EYE_ELEVATION_MAX = float(np.arctan2(1.0, 3.0))


def camera_world_shift(camera) -> list[float]:
    """Position of the drawn world that brings the camera target to the origin."""
    target = np.asarray(camera, dtype=float)[:3, 3]
    return [float(v) for v in -target]


def camera_eye_position(camera) -> list[float]:
    """Eye offset from the orbit origin, in the viewer's Y-up camera frame.

    The eye lies on the view-out side (``camera[:3, 2]``) at the distance
    ``camera[3, 3]``. Its elevation above the ground plane is capped so a
    top-down hint lands on the viewer's default inclination, on the +X side.
    """
    camera = np.asarray(camera, dtype=float)
    view_out = camera[:3, 2]
    distance = float(camera[3, 3])
    norm = float(np.linalg.norm(view_out))
    if norm < 1e-12:
        view_out, norm = np.array([0.0, 0.0, 1.0]), 1.0
    azimuth = float(np.arctan2(view_out[1], view_out[0]))
    elevation = float(np.arcsin(np.clip(view_out[2] / norm, -1.0, 1.0)))
    elevation = float(np.clip(elevation, -_EYE_ELEVATION_MAX, _EYE_ELEVATION_MAX))
    x = distance * np.cos(elevation) * np.cos(azimuth)
    y = distance * np.cos(elevation) * np.sin(azimuth)
    z = distance * np.sin(elevation)
    # The viewer turns its camera frame by +90 deg about X: world (x, y, z)
    # reads (x, z, -y) there.
    return [float(x), float(z), float(-y)]


# WebGL LineBasicMaterial ignores dash and (on most GPUs) linewidth. Overlay
# polylines become a thin ribbon so Meshcat 3-D matches matplotlib's dashed
# plans and corridor edges.
_LINE_WIDTH_M = 0.016
_LINE_DASH_M = 0.05
_LINE_LIFT_M = 0.01
_LINE_THICKNESS_M = 0.003
_GROUND_Z = 0.02


def _as_xyz(pts) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return np.zeros((0, 3))
    if pts.shape[1] == 2:
        return np.column_stack([pts, np.zeros(len(pts))])
    return np.asarray(pts[:, :3], dtype=float)


def _linestyle_dashes(style):
    """Return the on/off cycle in pattern units, or ``None`` for a solid line.

    Mirrors matplotlib: ``"-"`` / ``"--"`` / ``":"`` / ``"-."``, plus
    ``(offset, (on, off, ...))``.
    """
    if style in (None, "-", "solid"):
        return None
    if isinstance(style, str):
        return {
            "--": (2.0, 1.25),
            "dashed": (2.0, 1.25),
            ":": (0.4, 0.55),
            "dotted": (0.4, 0.55),
            "-.": (2.0, 0.7, 0.4, 0.7),
            "dashdot": (2.0, 0.7, 0.4, 0.7),
        }.get(style)
    if isinstance(style, tuple) and len(style) == 2:
        seq = style[1]
        if seq is None:
            return None
        return tuple(float(v) for v in seq)
    return None


def _polyline_point_at(pts: np.ndarray, cum: np.ndarray, s: float) -> np.ndarray:
    if s <= 0.0:
        return pts[0]
    total = float(cum[-1])
    if s >= total:
        return pts[-1]
    i = int(np.searchsorted(cum, s, side="right") - 1)
    i = max(0, min(i, len(pts) - 2))
    span = cum[i + 1] - cum[i]
    alpha = 0.0 if span < 1e-15 else (s - cum[i]) / span
    return pts[i] + alpha * (pts[i + 1] - pts[i])


def _horizontal_normal(tangent: np.ndarray) -> np.ndarray:
    n = np.array([-tangent[1], tangent[0], 0.0], dtype=float)
    length = float(np.linalg.norm(n))
    if length > 1e-9:
        return n / length
    return np.array([1.0, 0.0, 0.0], dtype=float)


def _dash_windows(length: float, dashes, unit: float, offset: float = 0.0):
    """Yield ``(s0, s1)`` intervals that should be drawn along ``[0, length]``."""
    if length <= 1e-12:
        return
    if dashes is None:
        yield 0.0, length
        return
    pattern = [max(float(v), 0.0) * unit for v in dashes]
    if not pattern or all(v <= 1e-15 for v in pattern):
        yield 0.0, length
        return
    s = -float(offset) * unit
    phase = 0
    while s < length:
        seg = pattern[phase % len(pattern)]
        if seg <= 1e-15:
            phase += 1
            if phase > 8 * len(pattern) and s < 0.0:
                s = 0.0
            continue
        a, b = max(s, 0.0), min(s + seg, length)
        if phase % 2 == 0 and b > a + 1e-12:
            yield a, b
        s += seg
        phase += 1
        if phase > 10_000:
            break


def _strip_from_stations(stations: np.ndarray, half_width: float, thickness: float):
    """Box-strip mesh through ``stations`` (K×3), offset in the floor plane."""
    if len(stations) < 2:
        return None
    tangents = np.diff(stations, axis=0)
    tangents = np.vstack([tangents[:1], tangents])
    for i in range(1, len(stations) - 1):
        tangents[i] = stations[i + 1] - stations[i - 1]
    normals = np.stack([_horizontal_normal(t) for t in tangents])
    left = stations - half_width * normals
    right = stations + half_width * normals
    up = np.array([0.0, 0.0, thickness], dtype=float)
    k = len(stations)
    vertices = np.vstack([left, right, left + up, right + up])
    faces = []
    for i in range(k - 1):
        a, b = i, i + 1
        c, d = k + i, k + i + 1
        e, f = 2 * k + i, 2 * k + i + 1
        g, h = 3 * k + i, 3 * k + i + 1
        faces.extend(
            [
                (a, b, d),
                (a, d, c),
                (e, h, f),
                (e, g, h),
                (a, c, g),
                (a, g, e),
                (b, f, h),
                (b, h, d),
            ]
        )
    return vertices, np.asarray(faces, dtype=np.uint32)


def polyline_strip_mesh(
    pts,
    *,
    linewidth=1.0,
    style="-",
    width=None,
    dash_unit=None,
    lift=_LINE_LIFT_M,
    thickness=_LINE_THICKNESS_M,
):
    """Ribbon mesh for a polyline: visible width, matplotlib dash, floor lift.

    Returns ``(vertices, faces)`` or ``None`` when the polyline is too short.
    Ground-plane lines (all ``|z|`` small) are lifted so they do not z-fight a
    tiled floor.
    """
    pts = _as_xyz(pts)
    if len(pts) < 2:
        return None
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = step > 1e-10
    pts = pts[keep]
    if len(pts) < 2:
        return None
    if float(np.max(np.abs(pts[:, 2]))) < _GROUND_Z:
        pts = pts.copy()
        pts[:, 2] = lift
    cum = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    )
    length = float(cum[-1])
    if length < 1e-10:
        return None
    lw = max(float(linewidth), 0.8)
    half = 0.5 * (float(width) if width is not None else _LINE_WIDTH_M * lw)
    unit = float(dash_unit) if dash_unit is not None else _LINE_DASH_M * lw
    offset = 0.0
    dashes = style
    if isinstance(style, tuple) and len(style) == 2:
        offset = float(style[0])
        dashes = style
    dashes = _linestyle_dashes(dashes)
    pieces = []
    for s0, s1 in _dash_windows(length, dashes, unit, offset=offset):
        samples = [s0]
        for s in cum:
            if s0 < s < s1:
                samples.append(float(s))
        samples.append(s1)
        stations = np.stack([_polyline_point_at(pts, cum, s) for s in samples])
        mesh = _strip_from_stations(stations, half, float(thickness))
        if mesh is not None:
            pieces.append(mesh)
    if not pieces:
        return None
    if len(pieces) == 1:
        return pieces[0]
    from minilink.graphical.meshes import merge_meshes

    return merge_meshes(*pieces)


def flipbook_pages(frames, i: int, key) -> list:
    """Runs of slot ``i`` over the frames, as ``(start, stop, primitive)``.

    A run covers the frames ``start <= k < stop``; consecutive frames whose
    primitives share one ``key`` (the geometry) share a run, and a frame
    without slot ``i`` is in none. A slot drawn the same way throughout is the
    single run ``(0, len(frames), primitive)``.
    """
    pages = []
    previous, previous_key = None, None
    for k, frame in enumerate(frames):
        if i >= len(frame["primitives"]):
            previous, previous_key = None, None
            continue
        primitive = frame["primitives"][i]
        # the kinematic geometry is one object for every frame: key it once
        primitive_key = previous_key if primitive is previous else key(primitive)
        if previous is not None and primitive_key == previous_key:
            start, _, first = pages[-1]
            pages[-1] = (start, k + 1, first)
        else:
            pages.append((k, k + 1, primitive))
        previous, previous_key = primitive, primitive_key
    return pages


def fold_growing_prefix(pages) -> list:
    """Flipbook pages of a growing polyline (a trail), each drawing only what it adds.

    Where a page's line extends the previous page's, frame after frame and
    drawn alike, the later page keeps just its new piece and every page of
    the chain stays shown to the chain's last frame: the pieces add up to the
    line of each frame, at a size linear in the frame count.
    """
    folded = []
    k = 0
    while k < len(pages):
        end = k
        while end + 1 < len(pages) and extends_polyline(pages[end], pages[end + 1]):
            end += 1
        chain_stop = pages[end][1]
        start, _, primitive = pages[k]
        folded.append((start, chain_stop, primitive))
        for j in range(k + 1, end + 1):
            start, _, primitive = pages[j]
            folded.append(
                (start, chain_stop, polyline_tail(pages[j - 1][2], primitive))
            )
        k = end + 1
    return folded


def extends_polyline(page, later) -> bool:
    """True when ``later`` follows ``page`` in time with the same line, grown at its end."""
    _, stop, before = page
    start, _, after = later
    if stop != start or type(before) is not CustomLine or type(after) is not CustomLine:
        return False
    look = (str(before.color), float(before.linewidth), repr(before.style))
    if look != (str(after.color), float(after.linewidth), repr(after.style)):
        return False
    n = len(before.pts)
    return 2 <= n < len(after.pts) and np.array_equal(after.pts[:n], before.pts)


def polyline_tail(before, after) -> CustomLine:
    """The piece of ``after`` past ``before``, from its last point, dashes in phase."""
    n = len(before.pts)
    style = before.style
    dashes = _linestyle_dashes(style)
    if dashes is not None:
        offset = float(style[0]) if isinstance(style, tuple) else 0.0
        unit = _LINE_DASH_M * max(float(before.linewidth), 0.8)
        length = float(
            np.sum(np.linalg.norm(np.diff(_as_xyz(before.pts), axis=0), axis=1))
        )
        style = (offset + length / unit, dashes)
    return CustomLine(
        after.pts[n - 1 :], color=after.color, linewidth=after.linewidth, style=style
    )


def page_visibility(start: int, stop: int, n_frames: int) -> dict:
    """Keyframes ``{frame: shown}`` of a page shown over ``start <= k < stop``.

    The last frame is always keyed, so every clip lasts the whole animation.
    """
    keys = {0: start == 0, start: True, n_frames - 1: stop >= n_frames}
    if stop < n_frames:
        keys[stop] = False
    return dict(sorted(keys.items()))


def _rotation_from_a_to_b(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return 3x3 rotation matrix mapping unit vector a to unit vector b."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    s = np.linalg.norm(v)
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        # 180deg: pick any orthogonal axis
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / (np.linalg.norm(v) + 1e-12)
        K = np.array(
            [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
            dtype=float,
        )
        return np.eye(3) + 2.0 * (K @ K)
    K = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]], dtype=float
    )
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s * s))


class MeshcatCanvas:
    """Maps primitives to meshcat geometry under a scene path."""

    def __init__(self, vis, scene_path="minilink_scene", is_3d=False):
        _import_meshcat()
        import meshcat.geometry as g
        import meshcat.transformations as tf

        self.vis = vis
        self.scene_path = scene_path
        self.scene = vis[scene_path]
        self.is_3d = is_3d
        self._g = g
        self._tf = tf
        self._geom_keys = []
        self._n_slots = 0

    def clear(self):
        self.scene.delete()
        self._geom_keys = []
        self._n_slots = 0

    def _base_path(self, i: int):
        return self.scene[f"p{i}"]

    def _primitive_key(self, primitive):
        # Conservative key: rebuild when any meaningful visual parameter changes.
        if isinstance(primitive, Point):
            return ("Point", float(primitive.size), str(primitive.color))
        if isinstance(primitive, CustomLine):
            return (
                "CustomLine",
                primitive.pts.shape,
                tuple(np.asarray(primitive.pts).reshape(-1).tolist()),
                str(primitive.color),
                float(primitive.linewidth),
                repr(primitive.style),
                bool(self.is_3d),
            )
        if isinstance(primitive, (Arrow, TorqueArrow)):
            # Honest arrows are baked polylines; key on the points so a reshaped
            # arc/arrow triggers a rebuild (geometry is in ``pts``).
            return (
                primitive.__class__.__name__,
                primitive.pts.shape,
                tuple(np.asarray(primitive.pts).reshape(-1).tolist()),
                str(primitive.color),
                float(primitive.linewidth),
                repr(primitive.style),
                bool(self.is_3d),
            )
        if isinstance(primitive, Circle):
            return (
                "Circle",
                float(primitive.radius),
                str(primitive.color),
                float(primitive.linewidth),
            )
        if isinstance(primitive, Sphere):
            return (
                "Sphere",
                float(primitive.radius),
                str(primitive.color),
                float(primitive.opacity),
            )
        if isinstance(primitive, Rod):
            return (
                "Rod",
                float(primitive.length),
                float(primitive.radius),
                str(primitive.color),
                float(primitive.opacity),
            )
        if isinstance(primitive, Plane):
            n = tuple(np.asarray(primitive.normal, dtype=float).tolist())
            return (
                "Plane",
                n,
                float(primitive.offset),
                float(primitive.size),
                float(primitive.thickness),
                str(primitive.color),
                float(primitive.opacity),
            )
        if isinstance(primitive, Box):
            c = tuple(np.asarray(primitive.center, dtype=float).tolist())
            return (
                "Box",
                float(primitive.length_x),
                float(primitive.length_y),
                float(primitive.length_z),
                c,
                str(primitive.color),
                float(primitive.opacity),
            )
        if isinstance(primitive, ExtrudedPolygon):
            return (
                "ExtrudedPolygon",
                primitive.pts_xy.shape,
                tuple(np.asarray(primitive.pts_xy).reshape(-1).tolist()),
                float(primitive.height),
                tuple(np.asarray(primitive.center, dtype=float).tolist()),
                str(primitive.color),
                float(primitive.opacity),
            )
        return (primitive.__class__.__name__,)

    def set_geometry(self, path, primitive) -> bool:
        """Draw ``primitive`` at ``path`` in its local frame; ``False`` when it draws nothing."""
        g = self._g
        hex_color = _color_to_meshcat_hex(primitive.color)

        if isinstance(primitive, Point):
            radius = max(0.02, 0.04 * float(primitive.size))
            path.set_object(
                g.Mesh(g.Sphere(radius), g.MeshLambertMaterial(color=hex_color))
            )
            return True

        if isinstance(primitive, Sphere):
            path.set_object(
                g.Mesh(
                    g.Sphere(float(primitive.radius)),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=primitive.opacity < 0.999,
                        opacity=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    ),
                )
            )
            return True

        if isinstance(primitive, Rod):
            path.set_object(
                g.Mesh(
                    g.Cylinder(float(primitive.length), float(primitive.radius)),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=primitive.opacity < 0.999,
                        opacity=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    ),
                )
            )
            return True

        if isinstance(primitive, Plane):
            path.set_object(
                g.Mesh(
                    g.Box(
                        [
                            float(primitive.size),
                            float(primitive.size),
                            float(max(primitive.thickness, 1e-3)),
                        ]
                    ),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=primitive.opacity < 0.999,
                        opacity=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    ),
                )
            )
            return True

        if isinstance(primitive, Box):
            path.set_object(
                g.Mesh(
                    g.Box(
                        [
                            float(max(primitive.length_x, 1e-6)),
                            float(max(primitive.length_y, 1e-6)),
                            float(max(primitive.length_z, 1e-6)),
                        ]
                    ),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=primitive.opacity < 0.999,
                        opacity=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    ),
                )
            )
            return True

        if isinstance(primitive, ExtrudedPolygon):
            vertices, faces = primitive.mesh_data()
            path.set_object(
                g.Mesh(
                    g.TriangularMeshGeometry(
                        np.asarray(vertices, dtype=np.float32),
                        np.asarray(faces, dtype=np.uint32),
                    ),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=primitive.opacity < 0.999,
                        opacity=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    ),
                )
            )
            return True

        if isinstance(primitive, Circle):
            # Interpret 2D circle primitives as volumetric 3D spheres in meshcat.
            # This makes bodies like pendulum tips and floating masses visible.
            path.set_object(
                g.Mesh(
                    g.Sphere(float(primitive.radius)),
                    g.MeshLambertMaterial(
                        color=hex_color,
                        transparent=not bool(primitive.fill),
                        opacity=1.0 if bool(primitive.fill) else 0.6,
                    ),
                )
            )
            return True

        if isinstance(primitive, (CustomLine, Arrow, TorqueArrow)):
            style = getattr(primitive, "style", "-")
            if self.is_3d:
                mesh = polyline_strip_mesh(
                    primitive.pts,
                    linewidth=float(primitive.linewidth),
                    style=style,
                )
                if mesh is None:
                    return False
                vertices, faces = mesh
                path.set_object(
                    g.Mesh(
                        g.TriangularMeshGeometry(
                            np.asarray(vertices, dtype=np.float32),
                            np.asarray(faces, dtype=np.uint32),
                        ),
                        g.MeshLambertMaterial(color=hex_color),
                    )
                )
                return True
            pts = primitive.pts
            if pts.shape[1] == 2:
                pts = np.hstack((pts, np.zeros((pts.shape[0], 1))))
            vertices = np.asarray(pts[:, :3], dtype=np.float32).T
            path.set_object(
                g.Line(
                    g.PointsGeometry(vertices),
                    g.LineBasicMaterial(
                        color=hex_color, linewidth=float(primitive.linewidth)
                    ),
                )
            )
            return True
        return False

    def _set_static_geometry(self, i: int, primitive):
        if not self.set_geometry(self._base_path(i), primitive):
            self._base_path(i).delete()

    def page_path(self, i: int, start: int):
        """Flipbook page of slot ``i`` first shown at frame ``start``."""
        return self._base_path(i)[f"f{start}"]

    def set_page(self, i: int, start: int, primitive) -> bool:
        """Draw one flipbook page, hidden unless it is the first frame's."""
        page = self.page_path(i, start)
        if not self.set_geometry(page, primitive):
            return False
        # after set_object: the meshcat server drops a node's properties on it
        if start > 0:
            page.set_property("visible", False)
        return True

    def ensure_objects(self, primitives):
        self.reserve_slots(len(primitives))
        for i, primitive in enumerate(primitives):
            self.ensure_object(i, primitive)

    def reserve_slots(self, n: int):
        """One scene slot per primitive; a new count starts from an empty scene."""
        if n != self._n_slots:
            self.clear()
            self._n_slots = n
            self._geom_keys = [None] * n

    def ensure_object(self, i: int, primitive):
        """(Re)build the geometry of slot ``i`` when the primitive's shape changed."""
        key = self._primitive_key(primitive)
        if self._geom_keys[i] != key:
            self._set_static_geometry(i, primitive)
            self._geom_keys[i] = key

    def update_primitive(self, i: int, primitive, transform_matrix):
        path = self._base_path(i)
        # Meshcat/umsgpack cannot serialize JAX arrays — convert at this boundary.
        transform_matrix = np.asarray(transform_matrix, dtype=float)

        if isinstance(primitive, Point):
            local_pt = np.append(primitive.pt, 1.0)
            world_pt = transform_matrix @ local_pt
            path.set_transform(self._tf.translation_matrix(world_pt[:3].tolist()))
            return

        if isinstance(primitive, (CustomLine, Arrow, TorqueArrow, Circle)):
            if isinstance(primitive, Circle):
                local_center = np.zeros(3)
                local_center[: len(primitive.center)] = primitive.center
                world_center = (transform_matrix @ np.append(local_center, 1.0))[:3]
                path.set_transform(self._tf.translation_matrix(world_center.tolist()))
            else:
                path.set_transform(transform_matrix)
            return

        if isinstance(primitive, Sphere):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            world_center = (transform_matrix @ np.append(local_center, 1.0))[:3]
            path.set_transform(self._tf.translation_matrix(world_center.tolist()))
            return

        if isinstance(primitive, Rod):
            # Cylinder is centered at origin and aligned with local Y in meshcat.
            # Place rod from hinge (0,0,0) to tip (0,-L,0) via center offset.
            T_local = self._tf.translation_matrix([0.0, -0.5 * primitive.length, 0.0])
            path.set_transform(transform_matrix @ T_local)
            return

        if isinstance(primitive, Plane):
            n = np.asarray(primitive.normal, dtype=float)
            n = n / (np.linalg.norm(n) + 1e-12)
            center = n * float(primitive.offset)
            R = _rotation_from_a_to_b(np.array([0.0, 0.0, 1.0]), n)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = center
            path.set_transform(transform_matrix @ T)
            return

        if isinstance(primitive, Box):
            c = np.asarray(primitive.center, dtype=float).reshape(3)
            T_c = self._tf.translation_matrix(c.tolist())
            path.set_transform(transform_matrix @ T_c)
            return

        if isinstance(primitive, ExtrudedPolygon):
            path.set_transform(transform_matrix)
            return


def _rigid_effective_transform(primitive, transform_matrix, tf):
    """
    Return the 4x4 transform that ``MeshcatCanvas.update_primitive`` would apply
    to the primitive's slot, or ``None`` for a primitive type it does not place.
    Lines and arrows are placed by their frame alone: their points, baked in
    that frame, are the geometry.
    """
    transform_matrix = np.asarray(transform_matrix, dtype=float)
    if isinstance(primitive, Point):
        local_pt = np.append(primitive.pt, 1.0)
        world_pt = transform_matrix @ local_pt
        return tf.translation_matrix(world_pt[:3].tolist())

    if isinstance(primitive, Circle):
        local_center = np.zeros(3)
        local_center[: len(primitive.center)] = primitive.center
        world_center = (transform_matrix @ np.append(local_center, 1.0))[:3]
        return tf.translation_matrix(world_center.tolist())

    if isinstance(primitive, Sphere):
        local_center = np.zeros(3)
        local_center[: len(primitive.center)] = primitive.center
        world_center = (transform_matrix @ np.append(local_center, 1.0))[:3]
        return tf.translation_matrix(world_center.tolist())

    if isinstance(primitive, Rod):
        T_local = tf.translation_matrix([0.0, -0.5 * primitive.length, 0.0])
        return transform_matrix @ T_local

    if isinstance(primitive, Plane):
        n = np.asarray(primitive.normal, dtype=float)
        n = n / (np.linalg.norm(n) + 1e-12)
        center = n * float(primitive.offset)
        R = _rotation_from_a_to_b(np.array([0.0, 0.0, 1.0]), n)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = center
        return transform_matrix @ T

    if isinstance(primitive, Box):
        c = np.asarray(primitive.center, dtype=float).reshape(3)
        T_c = tf.translation_matrix(c.tolist())
        return transform_matrix @ T_c

    if isinstance(primitive, (CustomLine, Arrow, TorqueArrow, ExtrudedPolygon)):
        return transform_matrix

    return None


class MeshcatRenderer(AnimationRenderer):
    """Browser-based playback and static-HTML snapshots.

    The viewer is always 3-D, so the ``is_3d`` argument of the renderer
    interface is accepted and ignored: lines are always drawn as ribbons with a
    visible width and dash pattern.
    """

    def __init__(self, animator):
        super().__init__(animator)
        self.vis = None
        self.canvas = None
        self.show = True

    def open_scene(
        self,
        *,
        is_3d: bool,
        show: bool,
        camera,
        title: str | None = None,
    ) -> None:
        meshcat = _import_meshcat()
        self.show = show
        self.vis = meshcat.Visualizer()
        # the viewer is always 3-D: ``is_3d`` only picks the axes of flat renderers
        self.canvas = MeshcatCanvas(self.vis, is_3d=True)
        if show:
            import sys

            if "google.colab" in sys.modules:
                from google.colab import output

                port = int(self.vis.url().split(":")[-1].split("/")[0])
                print(f"[Colab] Rendering live Meshcat on Port {port}.")
                print(
                    "If you see a blank white box, you MUST allow Third-Party Cookies in your browser to view Colab iframes."
                )
                output.serve_kernel_port_as_iframe(port, path="/static/", height=500)
            else:
                self.vis.open()
                self.vis.wait()
        # Only after the browser handshake: the meshcat server stops answering
        # commands when a browser connects through ``wait()`` while its scene
        # tree already holds commands, so nothing is sent before it.
        self._place_eye(camera)
        self._set_backdrop()

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        # The camera target stays at the orbit origin, so everything is drawn
        # shifted by -target. Here the shift rides in each primitive's own
        # transform. One scene-level slide sent after the primitives would let the
        # browser render in between: a followed body would show at its new pose
        # in the old view, then snap back, and flicker between the two.
        shift = self.canvas._tf.translation_matrix(camera_world_shift(camera))
        self.canvas.reserve_slots(len(primitives))
        for i, (prim, T) in enumerate(zip(primitives, transforms)):
            # geometry then pose, back to back: a rebuilt polyline (plan, trail)
            # never waits at the previous frame's shift
            self.canvas.ensure_object(i, prim)
            self.canvas.update_primitive(i, prim, shift @ np.asarray(T, dtype=float))
        self._slide_furniture(camera)

    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        if block:
            import sys

            if "google.colab" in sys.modules:
                from IPython.display import display

                print("Meshcat static frame built. Displaying offline standalone HTML.")
                display(self.vis.render_static(height=500))
                return
            print("Meshcat static frame ready.")
            # Only gate on `input()` in bare scripts; IPython REPL, Jupyter,
            # and Colab keep the process / kernel alive so the browser tab
            # stays reachable without a blocking prompt.
            if is_blocking_needed():
                input("Press Enter to exit meshcat viewer...")
        elif self.show and interval_s is not None:
            time.sleep(interval_s)

    def close_scene(self) -> None:
        self.canvas = None
        self.vis = None

    def _world_nodes(self):
        """The drawn world and the viewer's grid and axes: what slides with the target."""
        vis = self.canvas.vis
        return (vis, *(vis[path] for path in _VIEWER_FURNITURE))

    def _place_eye(self, camera) -> None:
        """Eye at the camera distance on the view-out side, its clip planes scaled to that distance."""
        eye = self.canvas.vis[_CAMERA_EYE]
        distance = float(np.asarray(camera, dtype=float)[3, 3])
        eye.set_property("position", camera_eye_position(camera))
        # the viewer's default planes (0.01, 100) clip a scene seen from far away
        eye.set_property("near", max(0.01, 1.0e-4 * distance))
        eye.set_property("far", max(100.0, 50.0 * distance))

    def _set_backdrop(self) -> None:
        """The system's backdrop hint: the viewer's grid and axes hidden when ``scene_grid`` is off."""
        if not getattr(self.sys, "scene_grid", True):
            for path in _VIEWER_FURNITURE:
                self.canvas.vis[path].set_property("visible", False)

    def _place_camera(self, camera) -> None:
        """Eye placed; world slid to the target (the native clip keyframes that slide)."""
        self._place_eye(camera)
        self._slide_world(camera)

    def _slide_furniture(self, camera) -> None:
        """Slide the viewer's grid and axes with the target (live frames)."""
        shift = camera_world_shift(camera)
        for path in _VIEWER_FURNITURE:
            self.canvas.vis[path].set_property("position", shift)

    def _slide_world(self, camera) -> None:
        shift = camera_world_shift(camera)
        for node in self._world_nodes():
            node.set_property("position", shift)

    def _build_meshcat_animation(self, frames, schedule):
        """
        Compile the frame list into a ``meshcat.animation.Animation``.

        A slot drawn the same way in every frame is one object with a pose
        track (none when it never moves). A slot whose geometry changes (an
        ``Arrow``, a ``TorqueArrow``, a growing trail, a receding horizon) is a
        flipbook: one page per run of frames with the same geometry, under the
        slot's pose, each shown by a ``visible`` track over its own run. The
        world slide keeps the per-frame camera target under the eye (the eye
        distance is the first frame's).
        """
        import meshcat.animation as mcanim

        canvas = self.canvas
        n_frames = len(frames)
        n_slots = max(len(frame["primitives"]) for frame in frames)
        canvas.reserve_slots(n_slots)
        self._place_camera(frames[0]["camera"])
        self._set_backdrop()
        animation_obj = mcanim.Animation(default_framerate=schedule.target_fps)

        for i in range(n_slots):
            pages = fold_growing_prefix(
                flipbook_pages(frames, i, canvas._primitive_key)
            )
            if len(pages) == 1 and pages[0][:2] == (0, n_frames):
                canvas.ensure_object(i, pages[0][2])
            else:
                for start, stop, primitive in pages:
                    if not canvas.set_page(i, start, primitive):
                        continue
                    page = canvas.page_path(i, start)
                    for k, shown in page_visibility(start, stop, n_frames).items():
                        with animation_obj.at_frame(page, k) as frame_vis:
                            frame_vis.set_property("visible", "boolean", shown)
            self._keyframe_slot_pose(animation_obj, frames, i)

        for k, frame in enumerate(frames):
            shift = camera_world_shift(frame["camera"])
            for node in self._world_nodes():
                with animation_obj.at_frame(node, k) as frame_vis:
                    frame_vis.set_property("position", "vector3", shift)

        return animation_obj

    def _keyframe_slot_pose(self, animation_obj, frames, i: int) -> None:
        """Pose of slot ``i``: placed at its first frame, keyframed only if it moves."""
        tf = self.canvas._tf
        poses = []
        for k, frame in enumerate(frames):
            if i >= len(frame["primitives"]):
                continue
            T = _rigid_effective_transform(
                frame["primitives"][i], frame["transforms"][i], tf
            )
            if T is not None:
                poses.append((k, np.asarray(T, dtype=float)))
        if not poses:
            return
        path = self.canvas._base_path(i)
        path.set_transform(poses[0][1])
        if all(np.allclose(T, poses[0][1], rtol=0.0, atol=1e-12) for _, T in poses):
            return
        for k, T in poses:
            with animation_obj.at_frame(path, k) as frame_vis:
                frame_vis.set_transform(T)

    def _set_native_animation(self, frames, schedule):
        """Build a Visualizer, keyframe the native Meshcat animation, and play it."""
        meshcat = _import_meshcat()
        self.vis = meshcat.Visualizer()
        # the viewer is always 3-D: ``is_3d`` only picks the axes of flat renderers
        self.canvas = MeshcatCanvas(self.vis, is_3d=True)
        animation_obj = self._build_meshcat_animation(frames, schedule)
        self.vis.set_animation(animation_obj, play=True, repetitions=1)
        return animation_obj

    def export_animation(
        self, primitives, frames, schedule, file_name: str, *, is_3d: bool = False
    ) -> None:
        """Write a standalone HTML page of the native Meshcat animation."""
        self.show = False
        self._set_native_animation(frames, schedule)
        path = html_export_path(file_name)
        path.write_text(self.vis.static_html(), encoding="utf-8")
        print(f"Saving animation to {path} ...")

    def play_native(
        self,
        primitives,
        frames,
        schedule,
        *,
        is_3d: bool,
        scene_title: str | None = None,
    ):
        """
        Drive playback through ``meshcat.animation.Animation`` +
        ``Visualizer.set_animation`` instead of a Python frame loop. The browser
        plays keyframes natively; no ``time.sleep`` in Python.
        """
        self.show = True
        animation_obj = self._set_native_animation(frames, schedule)
        self.vis.open()
        self.vis.wait()
        return animation_obj

    def render_inline_animation(self, primitives, frames, schedule, *, is_3d: bool):
        """
        Return an ``IPython.display.HTML`` iframe snapshot of the meshcat scene
        with the animation embedded. Uses ``Visualizer.render_static`` under the
        hood, which wraps ``static_html()`` in a self-contained ``srcdoc=``.
        Ideal for Colab: no zmq client, no port forwarding.
        """
        self.show = False
        self._set_native_animation(frames, schedule)

        try:
            return self.vis.render_static(height=480)
        except Exception:
            # Fallback: return the raw static HTML blob wrapped for notebooks.
            try:
                from IPython.display import HTML as IPythonHTML

                return IPythonHTML(self.vis.static_html())
            except ImportError:
                return self.vis.static_html()
