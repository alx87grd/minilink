"""Meshcat WebGL backend."""

from __future__ import annotations

import time

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.primitives import Circle, CustomLine, Point
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
            else:
                th = np.linspace(0, 2 * np.pi, 50)
                cx = x + primitive.radius * np.cos(th)
                cy = y + primitive.radius * np.sin(th)
                cz = np.full_like(th, z)
                vertices = np.vstack((cx, cy, cz)).astype(np.float32)
                path.set_object(
                    g.LineLoop(
                        g.PointsGeometry(vertices),
                        g.LineBasicMaterial(color=hex_color, linewidth=lw),
                    )
                )


class MeshcatRenderer(AnimationRenderer):
    """Browser-based playback; optional GIF/HTML not supported."""

    def render_static(self, x, u, t: float, is_3d: bool) -> None:
        meshcat = _import_meshcat()
        vis = meshcat.Visualizer()
        canvas = MeshcatCanvas(vis, is_3d=is_3d)

        primitives = self.sys.get_kinematic_geometry()
        transforms = self.sys.get_kinematic_transforms(x, u, t)

        if len(primitives) != len(transforms):
            raise ValueError(
                "System graphical error: Number of transforms must equal number of base geometric primitives."
            )

        canvas.clear()
        for prim, T in zip(primitives, transforms):
            canvas.draw_primitive(prim, T)

        vis.open()
        vis.wait()
        print(f"Static frame (t = {t:.2f} s).")
        input("Press Enter to exit meshcat viewer...")

    def render_animation(
        self,
        traj,
        *,
        time_factor_video: float,
        is_3d: bool,
        save: bool,
        file_name: str,
        show: bool,
        html: bool,
    ):
        if html:
            print(
                "html=True is only supported for renderer='matplotlib'; "
                "ignoring html for meshcat animation."
            )
        if save:
            print(
                f"save=True (GIF to {file_name!r}) is not supported for "
                "renderer='meshcat'; skipping export."
            )

        from minilink.graphical.renderers.timing import (
            sim_index_for_frame,
            trajectory_frame_schedule,
        )

        meshcat = _import_meshcat()
        vis = meshcat.Visualizer()
        canvas = MeshcatCanvas(vis, is_3d=is_3d)
        primitives = self.sys.get_kinematic_geometry()

        sched = trajectory_frame_schedule(traj, time_factor_video)
        n_frames = sched.n_frames
        nsteps = sched.nsteps
        interval_s = sched.interval_ms / 1000.0

        if n_frames <= 0:
            return None

        if show:
            vis.open()
            vis.wait()

        sim_idx = 0
        for frame_idx in range(n_frames):
            canvas.clear()
            sim_idx = sim_index_for_frame(frame_idx, sched)
            x = traj.x[:, sim_idx]
            if len(traj.u) > 0:
                u = traj.u[:, sim_idx]
            else:
                u = np.array([])

            transforms = self.sys.get_kinematic_transforms(x, u, traj.t[sim_idx])

            if len(primitives) != len(transforms):
                raise ValueError(
                    "System graphical error: Number of transforms must equal number of base geometric primitives."
                )

            for prim, T in zip(primitives, transforms):
                canvas.draw_primitive(prim, T)

            if show and frame_idx < n_frames - 1:
                time.sleep(interval_s)

        if show:
            print(
                f"Meshcat animation finished (last frame t = {traj.t[sim_idx]:.2f} s)."
            )
            input("Press Enter to exit meshcat viewer...")

        return None
