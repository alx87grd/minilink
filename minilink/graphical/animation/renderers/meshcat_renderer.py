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


def _frames_have_changing_polylines(frames) -> bool:
    """True when a line/arrow primitive changes vertices after the first frame."""
    if len(frames) < 2:
        return False
    first = frames[0]["primitives"]
    for frame in frames[1:]:
        for a, b in zip(first, frame["primitives"]):
            if not isinstance(a, (CustomLine, Arrow, TorqueArrow)):
                continue
            if type(a) is not type(b):
                return True
            pa = np.asarray(a.pts, dtype=float)
            pb = np.asarray(b.pts, dtype=float)
            if pa.shape != pb.shape or not np.allclose(pa, pb, atol=1e-9, rtol=0.0):
                return True
    return False


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
        self._has_head = []
        self._n_slots = 0

    def clear(self):
        self.scene.delete()
        self._geom_keys = []
        self._has_head = []
        self._n_slots = 0

    def _base_path(self, i: int):
        return self.scene[f"p{i}"]

    def _head_path(self, i: int):
        return self.scene[f"p{i}_head"]

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

    def _set_static_geometry(self, i: int, primitive):
        g = self._g
        path = self._base_path(i)
        hex_color = _color_to_meshcat_hex(primitive.color)

        if isinstance(primitive, Point):
            radius = max(0.02, 0.04 * float(primitive.size))
            path.set_object(
                g.Mesh(g.Sphere(radius), g.MeshLambertMaterial(color=hex_color))
            )
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

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
            self._has_head[i] = False
            return

        if isinstance(primitive, (CustomLine, Arrow, TorqueArrow)):
            style = getattr(primitive, "style", "-")
            if self.is_3d:
                mesh = polyline_strip_mesh(
                    primitive.pts,
                    linewidth=float(primitive.linewidth),
                    style=style,
                )
                if mesh is None:
                    path.delete()
                    self._has_head[i] = False
                    return
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
                self._has_head[i] = False
                return
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
            self._has_head[i] = False
            return

    def ensure_objects(self, primitives):
        n = len(primitives)
        if n != self._n_slots:
            self.clear()
            self._n_slots = n
            self._geom_keys = [None] * n
            self._has_head = [False] * n

        for i, primitive in enumerate(primitives):
            key = self._primitive_key(primitive)
            if self._geom_keys[i] != key:
                if i < len(self._has_head) and self._has_head[i]:
                    self._head_path(i).delete()
                    self._has_head[i] = False
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
    for the given rigid primitive, or ``None`` for primitives whose geometry is
    rebuilt each frame (honest ``Arrow`` / ``TorqueArrow`` overlays) — in native
    meshcat keyframing only the transform animates, so per-frame geometry is
    frozen at ``t=0`` (use ``native=False`` for frame-accurate dynamic geometry).
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
    """Browser-based playback and static-HTML snapshots."""

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
        self.canvas = MeshcatCanvas(self.vis, is_3d=is_3d)
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

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        self.canvas.ensure_objects(primitives)
        for i, (prim, T) in enumerate(zip(primitives, transforms)):
            self.canvas.update_primitive(i, prim, T)
        # Meshcat uses the viewer default camera; ``camera`` is ignored.

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

    def _build_meshcat_animation(self, primitives, frames, schedule):
        """
        Compile the frame list into a ``meshcat.animation.Animation`` keyframe
        track per rigid primitive path. Dynamic polylines (``Arrow``,
        ``TorqueArrow``, changing ``CustomLine`` trails/horizons) are left
        frozen at ``t=0`` and a one-line notice is printed if any are present.
        """
        import meshcat.animation as mcanim

        self.canvas.ensure_objects(primitives)

        # Draw t=0 once: this sets the (frozen) geometry of dynamic polylines
        # and gives every rigid primitive a sane starting pose before keyframes
        # kick in.
        t0_transforms = frames[0]["transforms"]
        has_dynamic = _frames_have_changing_polylines(frames)
        for i, (prim, T0) in enumerate(zip(primitives, t0_transforms)):
            self.canvas.update_primitive(i, prim, T0)
            if isinstance(prim, (Arrow, TorqueArrow)):
                has_dynamic = True

        animation_obj = mcanim.Animation(default_framerate=schedule.target_fps)
        tf = self.canvas._tf

        for frame_idx, frame in enumerate(frames):
            for i, (prim, T) in enumerate(zip(primitives, frame["transforms"])):
                T_eff = _rigid_effective_transform(prim, T, tf)
                if T_eff is None:
                    continue
                path_vis = self.canvas._base_path(i)
                with animation_obj.at_frame(path_vis, frame_idx) as frame_vis:
                    frame_vis.set_transform(np.asarray(T_eff, dtype=float))

        if has_dynamic:
            print(
                "Note: meshcat native animation freezes per-frame dynamic "
                "geometry (e.g. Arrow length/direction, TorqueArrow sweep, "
                "CustomLine trails/horizons) at t=0; use native=False for "
                "frame-accurate playback of those primitives."
            )

        return animation_obj

    def _set_native_animation(self, primitives, frames, schedule, *, is_3d: bool):
        """Build a Visualizer, keyframe the native Meshcat animation, and play it."""
        meshcat = _import_meshcat()
        self.vis = meshcat.Visualizer()
        self.canvas = MeshcatCanvas(self.vis, is_3d=is_3d)
        animation_obj = self._build_meshcat_animation(primitives, frames, schedule)
        self.vis.set_animation(animation_obj, play=True, repetitions=1)
        return animation_obj

    def export_animation(
        self, primitives, frames, schedule, file_name: str, *, is_3d: bool = False
    ) -> None:
        """Write a standalone HTML page of the native Meshcat animation."""
        self.show = False
        self._set_native_animation(primitives, frames, schedule, is_3d=is_3d)
        path = html_export_path(file_name)
        path.write_text(self.vis.static_html())
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
        animation_obj = self._set_native_animation(
            primitives, frames, schedule, is_3d=is_3d
        )
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
        self._set_native_animation(primitives, frames, schedule, is_3d=is_3d)

        try:
            return self.vis.render_static(height=480)
        except Exception:
            # Fallback: return the raw static HTML blob wrapped for notebooks.
            try:
                from IPython.display import HTML as IPythonHTML

                return IPythonHTML(self.vis.static_html())
            except ImportError:
                return self.vis.static_html()
