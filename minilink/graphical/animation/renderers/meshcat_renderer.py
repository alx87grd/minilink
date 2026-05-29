"""Meshcat WebGL backend."""

from __future__ import annotations

import time

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.common.environment import is_blocking_needed
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
    extract_amplitude,
)
from minilink.graphical.animation.renderers.renderer import AnimationRenderer


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
            )
        if isinstance(primitive, Arrow):
            return (
                "Arrow",
                primitive.pts.shape,
                str(primitive.color),
                float(primitive.linewidth),
            )
        if isinstance(primitive, TorqueArrow):
            return ("TorqueArrow", str(primitive.color), float(primitive.linewidth))
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

        if isinstance(primitive, (CustomLine, Arrow)):
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

        if isinstance(primitive, TorqueArrow):
            # Dynamic geometry updated each frame; ensure secondary head path exists.
            path.set_object(
                g.Line(
                    g.PointsGeometry(np.zeros((3, 2), dtype=np.float32)),
                    g.LineBasicMaterial(
                        color=hex_color, linewidth=float(primitive.linewidth)
                    ),
                )
            )
            self._head_path(i).set_object(
                g.Line(
                    g.PointsGeometry(np.zeros((3, 2), dtype=np.float32)),
                    g.LineBasicMaterial(
                        color=hex_color, linewidth=float(primitive.linewidth)
                    ),
                )
            )
            self._has_head[i] = True
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
        g = self._g
        path = self._base_path(i)

        if isinstance(primitive, Point):
            local_pt = np.append(primitive.pt, 1.0)
            world_pt = transform_matrix @ local_pt
            path.set_transform(self._tf.translation_matrix(world_pt[:3].tolist()))
            return

        if isinstance(primitive, (CustomLine, Arrow, Circle)):
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

        if isinstance(primitive, TorqueArrow):
            sweep, T_rigid = extract_amplitude(transform_matrix)
            local_pts = primitive.compute_pts(sweep)
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (T_rigid @ local_pts_hom.T).T
            hex_color = _color_to_meshcat_hex(primitive.color)
            arc_n = local_pts.shape[0] - 3
            if arc_n >= 2 and hasattr(path, "set_object"):
                arc_verts = np.asarray(world_pts[:arc_n, :3], dtype=np.float32).T
                path.set_object(
                    g.Line(
                        g.PointsGeometry(arc_verts),
                        g.LineBasicMaterial(
                            color=hex_color, linewidth=float(primitive.linewidth)
                        ),
                    )
                )
                head_verts = np.asarray(world_pts[arc_n:, :3], dtype=np.float32).T
                self._head_path(i).set_object(
                    g.Line(
                        g.PointsGeometry(head_verts),
                        g.LineBasicMaterial(
                            color=hex_color, linewidth=float(primitive.linewidth)
                        ),
                    )
                )
            return


def _rigid_effective_transform(primitive, transform_matrix, tf):
    """
    Return the 4x4 transform that ``MeshcatCanvas.update_primitive`` would apply
    for the given rigid primitive, or ``None`` for primitives whose geometry is
    rebuilt each frame (``TorqueArrow``) and therefore cannot be rigid-keyframed.
    """
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

    if isinstance(primitive, (CustomLine, Arrow, ExtrudedPolygon)):
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
        track per rigid primitive path. Dynamic primitives (``TorqueArrow``) are
        left frozen at ``t=0`` and a one-line notice is printed if any are
        present.
        """
        import meshcat.animation as mcanim

        self.canvas.ensure_objects(primitives)

        # Draw t=0 once: this sets the (frozen) geometry of TorqueArrow and gives
        # every rigid primitive a sane starting pose before keyframes kick in.
        t0_transforms = frames[0]["transforms"]
        has_dynamic = False
        for i, (prim, T0) in enumerate(zip(primitives, t0_transforms)):
            self.canvas.update_primitive(i, prim, T0)
            if isinstance(prim, TorqueArrow):
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
                "Note: meshcat native animation freezes dynamic-geometry "
                "primitives (e.g. TorqueArrow) at t=0; use native=False for "
                "frame-accurate playback of those primitives."
            )

        return animation_obj

    def play_native(self, primitives, frames, schedule, *, is_3d: bool):
        """
        Drive playback through ``meshcat.animation.Animation`` +
        ``Visualizer.set_animation`` instead of a Python frame loop. The browser
        plays keyframes natively; no ``time.sleep`` in Python.
        """
        meshcat = _import_meshcat()
        self.show = True
        self.vis = meshcat.Visualizer()
        self.canvas = MeshcatCanvas(self.vis, is_3d=is_3d)
        self.vis.open()
        self.vis.wait()

        animation_obj = self._build_meshcat_animation(primitives, frames, schedule)
        self.vis.set_animation(animation_obj, play=True, repetitions=1)
        return animation_obj

    def render_inline_animation(self, primitives, frames, schedule, *, is_3d: bool):
        """
        Return an ``IPython.display.HTML`` iframe snapshot of the meshcat scene
        with the animation embedded. Uses ``Visualizer.render_static`` under the
        hood, which wraps ``static_html()`` in a self-contained ``srcdoc=``.
        Ideal for Colab: no zmq client, no port forwarding.
        """
        meshcat = _import_meshcat()
        self.show = False
        self.vis = meshcat.Visualizer()
        self.canvas = MeshcatCanvas(self.vis, is_3d=is_3d)

        animation_obj = self._build_meshcat_animation(primitives, frames, schedule)
        self.vis.set_animation(animation_obj, play=True, repetitions=1)

        try:
            return self.vis.render_static(height=480)
        except Exception:
            # Fallback: return the raw static HTML blob wrapped for notebooks.
            try:
                from IPython.display import HTML as IPythonHTML

                return IPythonHTML(self.vis.static_html())
            except ImportError:
                return self.vis.static_html()
