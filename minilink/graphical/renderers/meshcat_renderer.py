"""Meshcat WebGL backend."""

from __future__ import annotations

import time

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.primitives import (
    Arrow,
    Circle,
    CustomLine,
    Plane,
    Point,
    Sphere,
    TorqueArrow,
    extract_amplitude,
)
from minilink.graphical.renderers.base import AnimationRenderer


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
        self._idx = 0

    def clear(self):
        self.scene.delete()
        self._idx = 0

    def draw_primitive(self, primitive, transform_matrix):
        g = self._g
        path = self.scene[f"p{self._idx}"]
        self._idx += 1

        if isinstance(primitive, Point):
            local_pt = np.append(primitive.pt, 1.0)
            world_pt = transform_matrix @ local_pt
            x, y, z = world_pt[0], world_pt[1], world_pt[2]
            radius = max(0.02, 0.04 * float(primitive.size))
            hex_color = _color_to_meshcat_hex(primitive.color)
            path.set_object(
                g.Mesh(g.Sphere(radius), g.MeshLambertMaterial(color=hex_color))
            )
            path.set_transform(self._tf.translation_matrix([x, y, z]))

        elif isinstance(primitive, CustomLine):
            local_pts = primitive.pts
            if local_pts.shape[1] == 2:
                local_pts_hom = np.hstack(
                    (local_pts, np.zeros((local_pts.shape[0], 1)))
                )
            else:
                local_pts_hom = local_pts
            local_pts_hom = np.hstack((local_pts_hom, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            vertices = np.asarray(world_pts[:, :3], dtype=np.float32).T
            hex_color = _color_to_meshcat_hex(primitive.color)
            path.set_object(
                g.Line(
                    g.PointsGeometry(vertices),
                    g.LineBasicMaterial(
                        color=hex_color,
                        linewidth=float(primitive.linewidth),
                    ),
                )
            )

        elif isinstance(primitive, Arrow):
            local_pts = primitive.pts
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            vertices = np.asarray(world_pts[:, :3], dtype=np.float32).T
            hex_color = _color_to_meshcat_hex(primitive.color)
            path.set_object(
                g.Line(
                    g.PointsGeometry(vertices),
                    g.LineBasicMaterial(
                        color=hex_color,
                        linewidth=float(primitive.linewidth),
                    ),
                )
            )

        elif isinstance(primitive, TorqueArrow):
            sweep, T_rigid = extract_amplitude(transform_matrix)
            local_pts = primitive.compute_pts(sweep)
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (T_rigid @ local_pts_hom.T).T
            hex_color = _color_to_meshcat_hex(primitive.color)
            arc_n = local_pts.shape[0] - 3
            if arc_n >= 2:
                arc_verts = np.asarray(world_pts[:arc_n, :3], dtype=np.float32).T
                path.set_object(
                    g.Line(
                        g.PointsGeometry(arc_verts),
                        g.LineBasicMaterial(
                            color=hex_color,
                            linewidth=float(primitive.linewidth),
                        ),
                    )
                )
                head_path = self.scene[f"p{self._idx}"]
                self._idx += 1
                head_verts = np.asarray(world_pts[arc_n:, :3], dtype=np.float32).T
                head_path.set_object(
                    g.Line(
                        g.PointsGeometry(head_verts),
                        g.LineBasicMaterial(
                            color=hex_color,
                            linewidth=float(primitive.linewidth),
                        ),
                    )
                )

        elif isinstance(primitive, Circle):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            local_center = np.append(local_center, 1.0)
            world_center = transform_matrix @ local_center
            x, y, z = world_center[0], world_center[1], world_center[2]
            hex_color = _color_to_meshcat_hex(primitive.color)
            lw = float(primitive.linewidth)

            if self.is_3d:
                th = np.linspace(0, 2 * np.pi, 50)
                pts_z = primitive.center[2] if len(primitive.center) > 2 else 0.0
                pts = np.vstack(
                    (
                        primitive.center[0] + primitive.radius * np.cos(th),
                        primitive.center[1] + primitive.radius * np.sin(th),
                        np.full_like(th, pts_z),
                        np.ones_like(th),
                    )
                )
                world_pts = transform_matrix @ pts
                vertices = np.asarray(world_pts[:3, :], dtype=np.float32)
                path.set_object(
                    g.LineLoop(
                        g.PointsGeometry(vertices),
                        g.LineBasicMaterial(color=hex_color, linewidth=lw),
                    )
                )

        elif isinstance(primitive, Sphere):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            world_center = (transform_matrix @ np.append(local_center, 1.0))[:3]
            hex_color = _color_to_meshcat_hex(primitive.color)
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
            path.set_transform(self._tf.translation_matrix(world_center.tolist()))

        elif isinstance(primitive, Plane):
            n = np.asarray(primitive.normal, dtype=float)
            n = n / (np.linalg.norm(n) + 1e-12)
            center = n * float(primitive.offset)
            R = _rotation_from_a_to_b(np.array([0.0, 0.0, 1.0]), n)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = center
            hex_color = _color_to_meshcat_hex(primitive.color)
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
            path.set_transform(transform_matrix @ T)


class MeshcatRenderer(AnimationRenderer):
    """Browser-based playback; optional GIF/HTML not supported."""

    def __init__(self, animator):
        super().__init__(animator)
        self.vis = None
        self.canvas = None
        self.show = True

    def open_scene(self, *, is_3d: bool, show: bool, title: str | None = None) -> None:
        meshcat = _import_meshcat()
        self.show = show
        self.vis = meshcat.Visualizer()
        self.canvas = MeshcatCanvas(self.vis, is_3d=is_3d)
        if show:
            self.vis.open()
            self.vis.wait()

    def draw_frame(self, primitives, transforms, t: float) -> None:
        self.canvas.clear()
        for prim, T in zip(primitives, transforms):
            self.canvas.draw_primitive(prim, T)

    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        if block:
            print("Meshcat static frame ready.")
            input("Press Enter to exit meshcat viewer...")
        elif self.show and interval_s is not None:
            time.sleep(interval_s)

    def close_scene(self) -> None:
        self.canvas = None
        self.vis = None
