"""Matplotlib backend: static figure and FuncAnimation playback."""

from __future__ import annotations

from itertools import product

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from minilink.graphical.environment import is_blocking_needed
from minilink.graphical.matplotlib_style import (
    DPI_EXPORT,
    DPI_FIGURE,
    FIGSIZE_ANIMATION,
    FONT_SIZE,
    style_animation_axes,
)
from minilink.graphical.primitives import (
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
    world_to_camera,
)
from minilink.graphical.renderers.renderer import AnimationRenderer

_AXIS_LABEL_BY_INDEX = ("X", "Y", "Z")


def _axis_label_from_column(col):
    """Return ``"X" / "Y" / "Z"`` if *col* is (close to) an axis-aligned unit vector, else ``""``."""
    col = np.asarray(col, dtype=float).reshape(3)
    abs_col = np.abs(col)
    i = int(np.argmax(abs_col))
    if abs_col[i] < 0.999:
        return ""
    other = np.delete(abs_col, i)
    if np.any(other > 1e-3):
        return ""
    return ("-" if col[i] < 0 else "") + _AXIS_LABEL_BY_INDEX[i]


def _camera_3d_view_init(camera):
    """Decode matplotlib ``view_init(elev, azim)`` from camera-Z (view-out direction)."""
    view_out = np.asarray(camera[:3, 2], dtype=float).reshape(3)
    n = np.linalg.norm(view_out)
    if n < 1e-12:
        return None
    v = view_out / n
    elev = float(np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0))))
    azim = float(np.degrees(np.arctan2(v[1], v[0])))
    return elev, azim


class MatplotlibCanvas:
    """Maps primitives to matplotlib artists (internal drawing layer)."""

    def __init__(self, ax, is_3d=False):
        self.ax = ax
        self.is_3d = is_3d
        self.drawn_objects = []

    def draw_primitive(self, primitive, transform_matrix):
        if isinstance(primitive, Point):
            local_pt = np.append(primitive.pt, 1.0)
            world_pt = transform_matrix @ local_pt
            x, y, z = world_pt[0], world_pt[1], world_pt[2]

            if self.is_3d:
                (obj,) = self.ax.plot(
                    [x],
                    [y],
                    [z],
                    marker=primitive.marker,
                    color=primitive.color,
                    markersize=primitive.size,
                )
            else:
                (obj,) = self.ax.plot(
                    [x],
                    [y],
                    marker=primitive.marker,
                    color=primitive.color,
                    markersize=primitive.size,
                )

            self.drawn_objects.append(obj)

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

            x = world_pts[:, 0]
            y = world_pts[:, 1]
            z = world_pts[:, 2]

            if self.is_3d:
                (obj,) = self.ax.plot(
                    x,
                    y,
                    z,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            else:
                (obj,) = self.ax.plot(
                    x,
                    y,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, Arrow):
            local_pts = primitive.pts
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            x = world_pts[:, 0]
            y = world_pts[:, 1]

            if self.is_3d:
                z = world_pts[:, 2]
                (obj,) = self.ax.plot(
                    x,
                    y,
                    z,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            else:
                (obj,) = self.ax.plot(
                    x,
                    y,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, TorqueArrow):
            sweep, T_rigid = extract_amplitude(transform_matrix)
            local_pts = primitive.compute_pts(sweep)
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (T_rigid @ local_pts_hom.T).T

            arc_n = local_pts.shape[0] - 3
            if arc_n >= 2:
                (arc_obj,) = self.ax.plot(
                    world_pts[:arc_n, 0],
                    world_pts[:arc_n, 1],
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
                self.drawn_objects.append(arc_obj)
                (head_obj,) = self.ax.plot(
                    world_pts[arc_n:, 0],
                    world_pts[arc_n:, 1],
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle="-",
                )
                self.drawn_objects.append(head_obj)

        elif isinstance(primitive, Circle):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            local_center = np.append(local_center, 1.0)

            world_center = transform_matrix @ local_center

            x, y, z = world_center[0], world_center[1], world_center[2]

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

                (obj,) = self.ax.plot(
                    world_pts[0, :],
                    world_pts[1, :],
                    world_pts[2, :],
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
                self.drawn_objects.append(obj)
            else:
                circ = patches.Circle(
                    (x, y),
                    radius=primitive.radius,
                    ec=primitive.color,
                    fill=primitive.fill,
                    fc=primitive.color if primitive.fill else "none",
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
                obj = self.ax.add_patch(circ)
                self.drawn_objects.append(obj)

        elif isinstance(primitive, Sphere):
            # 2D/3D fallback: draw sphere as a circle in XY.
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            world_center = transform_matrix @ np.append(local_center, 1.0)
            x, y = world_center[0], world_center[1]
            circ = patches.Circle(
                (x, y),
                radius=primitive.radius,
                ec=primitive.color,
                fill=True,
                fc=primitive.color,
                alpha=float(np.clip(primitive.opacity, 0.0, 1.0)),
                linewidth=1.5,
            )
            obj = self.ax.add_patch(circ)
            self.drawn_objects.append(obj)

        elif isinstance(primitive, Plane):
            # 2D fallback: draw XY intersection line of n·x=offset.
            n = np.asarray(primitive.normal, dtype=float)
            off = float(primitive.offset)
            half = 0.5 * float(primitive.size)
            if abs(n[1]) > 1e-9:
                x0, x1 = -half, half
                y0 = (off - n[0] * x0) / n[1]
                y1 = (off - n[0] * x1) / n[1]
                local = np.array([[x0, y0, 0.0], [x1, y1, 0.0]])
            else:
                x = off / (n[0] + 1e-12)
                local = np.array([[x, -half, 0.0], [x, half, 0.0]])
            pts = np.hstack((local, np.ones((2, 1))))
            world = (transform_matrix @ pts.T).T
            (obj,) = self.ax.plot(
                world[:, 0],
                world[:, 1],
                color=primitive.color,
                linewidth=2.0,
                alpha=float(np.clip(primitive.opacity, 0.0, 1.0)),
            )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, Rod):
            local = np.array([[0.0, 0.0, 0.0], [0.0, -primitive.length, 0.0]])
            local_h = np.hstack((local, np.ones((2, 1))))
            world = (transform_matrix @ local_h.T).T
            lw = float(primitive.linewidth)
            if self.is_3d:
                (obj,) = self.ax.plot(
                    world[:, 0],
                    world[:, 1],
                    world[:, 2],
                    color=primitive.color,
                    linewidth=lw,
                    linestyle=primitive.style,
                )
            else:
                (obj,) = self.ax.plot(
                    world[:, 0],
                    world[:, 1],
                    color=primitive.color,
                    linewidth=lw,
                    linestyle=primitive.style,
                )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, Box):
            lx, ly, lz = primitive.length_x, primitive.length_y, primitive.length_z
            c = np.asarray(primitive.center, dtype=float).reshape(3)
            corners = []
            for sx, sy, sz in product((-1.0, 1.0), repeat=3):
                corners.append(
                    np.array([sx * lx / 2, sy * ly / 2, sz * lz / 2], dtype=float) + c
                )
            edges = (
                (0, 1),
                (0, 2),
                (0, 4),
                (1, 3),
                (1, 5),
                (2, 3),
                (2, 6),
                (3, 7),
                (4, 5),
                (4, 6),
                (5, 7),
                (6, 7),
            )
            corners_h = np.hstack((np.array(corners), np.ones((8, 1))))
            world_c = (transform_matrix @ corners_h.T).T[:, :3]
            for i, j in edges:
                p0, p1 = world_c[i], world_c[j]
                if self.is_3d:
                    (seg,) = self.ax.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        [p0[2], p1[2]],
                        color=primitive.color,
                        linewidth=1.5,
                        linestyle=primitive.style,
                    )
                else:
                    (seg,) = self.ax.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        color=primitive.color,
                        linewidth=1.5,
                        linestyle=primitive.style,
                    )
                self.drawn_objects.append(seg)

        elif isinstance(primitive, ExtrudedPolygon):
            vertices = primitive.vertices_local()
            vertices_h = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
            world_v = (transform_matrix @ vertices_h.T).T[:, :3]
            for i, j in primitive.edges():
                p0, p1 = world_v[i], world_v[j]
                if self.is_3d:
                    (seg,) = self.ax.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        [p0[2], p1[2]],
                        color=primitive.color,
                        linewidth=1.5,
                        alpha=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    )
                else:
                    (seg,) = self.ax.plot(
                        [p0[0], p1[0]],
                        [p0[1], p1[1]],
                        color=primitive.color,
                        linewidth=1.5,
                        alpha=float(np.clip(primitive.opacity, 0.0, 1.0)),
                    )
                self.drawn_objects.append(seg)

    def clear(self):
        for obj in self.drawn_objects:
            obj.remove()
        self.drawn_objects.clear()


class MatplotlibRenderer(AnimationRenderer):
    """Static matplotlib figure and GIF / notebook HTML animation."""

    def __init__(self, animator):
        super().__init__(animator)
        self.fig = None
        self.ax = None
        self.canvas = None

    def _apply_camera(self, ax, camera, is_3d):
        """Apply view limits, labels, and (3D) view orientation from *camera*."""
        target = np.asarray(camera[:3, 3], dtype=float).reshape(3)
        scale = float(camera[3, 3])
        if is_3d:
            ax.set_xlim3d(target[0] - scale, target[0] + scale)
            ax.set_ylim3d(target[1] - scale, target[1] + scale)
            ax.set_zlim3d(target[2] - scale, target[2] + scale)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            view = _camera_3d_view_init(camera)
            if view is not None:
                ax.view_init(elev=view[0], azim=view[1])
        else:
            ax.set_xlim(-scale, scale)
            ax.set_ylim(-scale, scale)
            ax.set_xlabel(_axis_label_from_column(camera[:3, 0]))
            ax.set_ylabel(_axis_label_from_column(camera[:3, 1]))
            ax.set_aspect("equal")

    def _create_figure_and_ax(self, is_3d, camera):
        fig = plt.figure(figsize=FIGSIZE_ANIMATION, dpi=DPI_FIGURE)
        fig.canvas.manager.set_window_title(f"Animation: {self.sys.name}")

        if is_3d:
            ax = fig.add_subplot(111, projection="3d")
        else:
            ax = fig.add_subplot(111)

        self._apply_camera(ax, camera, is_3d)
        style_animation_axes(ax, is_3d=is_3d)
        return fig, ax

    def open_scene(
        self,
        *,
        is_3d: bool,
        show: bool,
        camera,
        title: str | None = None,
    ) -> None:
        self.fig, self.ax = self._create_figure_and_ax(is_3d, camera)
        self.canvas = MatplotlibCanvas(self.ax, is_3d=is_3d)
        if title:
            self.ax.set_title(title, fontsize=FONT_SIZE)

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        self.canvas.clear()
        # 2D path uses an orthographic projection onto the camera plane: pre-multiply
        # body transforms by world-to-camera so primitive XY is already in camera frame.
        # 3D matplotlib keeps world coordinates; limits and ``view_init`` come from the
        # camera once at ``open_scene``, then the interactive UI owns the viewpoint.
        if self.canvas.is_3d:
            draw_transforms = transforms
        else:
            W = world_to_camera(camera)
            draw_transforms = [W @ T for T in transforms]
        for prim, T in zip(primitives, draw_transforms):
            self.canvas.draw_primitive(prim, T)
        # 2D: refresh orthographic limits and axis labels from *camera* each frame.
        # 3D: *camera* was applied once in ``open_scene``; keep mpl toolbars / mouse
        # orbit from being reset every frame.
        if not self.canvas.is_3d:
            self._apply_camera(self.ax, camera, False)
        self.ax.set_title(f"Time = {t:.2f} s", fontsize=FONT_SIZE)

    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        if block:
            # Only actually block in script mode; IPython REPL / Jupyter /
            # Colab keep the process alive independently.
            if is_blocking_needed():
                plt.show(block=True)
            return
        plt.pause(0.001 if interval_s is None else interval_s)

    def poll_events(self) -> dict[str, bool]:
        if self.fig is None:
            return {"quit": True}
        return {"quit": not plt.fignum_exists(self.fig.number)}

    def close_scene(self) -> None:
        if self.fig is not None:
            plt.close(self.fig)
        self.fig = None
        self.ax = None
        self.canvas = None

    def _build_animation(self, primitives, frames, schedule, *, is_3d: bool = False):
        fig, ax = self._create_figure_and_ax(is_3d=is_3d, camera=frames[0]["camera"])
        canvas = MatplotlibCanvas(ax, is_3d=is_3d)

        def update(frame_idx):
            frame = frames[frame_idx]
            canvas.clear()
            camera = frame["camera"]
            if is_3d:
                draw_transforms = frame["transforms"]
            else:
                W = world_to_camera(camera)
                draw_transforms = [W @ T for T in frame["transforms"]]
            for prim, T in zip(primitives, draw_transforms):
                canvas.draw_primitive(prim, T)
            if not is_3d:
                self._apply_camera(ax, camera, False)
            ax.set_title(f"Time = {frame['t']:.2f} s", fontsize=FONT_SIZE)
            return canvas.drawn_objects

        ani = animation.FuncAnimation(
            fig,
            update,
            frames=len(frames),
            interval=schedule.interval_ms,
            blit=False,
        )
        return fig, ani

    def render_inline_animation(self, primitives, frames, schedule):
        fig, ani = self._build_animation(primitives, frames, schedule)
        plt.close(fig)
        try:
            from IPython.display import HTML

            return HTML(ani.to_jshtml())
        except ImportError:
            return ani

    def export_animation(self, primitives, frames, schedule, file_name: str) -> None:
        fig, ani = self._build_animation(primitives, frames, schedule)
        print(f"Saving animation to {file_name}.gif ...")
        ani.save(
            file_name + ".gif",
            writer="imagemagick",
            fps=schedule.target_fps,
            dpi=DPI_EXPORT,
        )
        plt.close(fig)

    def play_native(self, primitives, frames, schedule, *, is_3d: bool):
        """
        Drive playback through ``matplotlib.animation.FuncAnimation`` instead
        of a Python frame loop.

        Only the window path lands here: Colab and Jupyter-with-non-interactive
        backend are routed to ``render_inline_animation`` upstream via
        :func:`minilink.graphical.environment.prefers_inline_animation`.

        - Bare script (``is_blocking_needed() is True``): block until the user
          closes the window, then clean up.
        - IPython terminal REPL or Jupyter with an interactive backend
          (``qt`` / ``widget`` / ``macosx`` / ``tk`` / ``nbagg``): show the
          window non-blocking and intentionally retain strong references to
          the figure and ``FuncAnimation`` on ``self`` so the backend event
          loop can drive playback without ``FuncAnimation`` being garbage
          collected.
        """
        fig, ani = self._build_animation(primitives, frames, schedule, is_3d=is_3d)
        self.fig = fig
        self.ax = fig.axes[0] if fig.axes else None
        self.canvas = None
        self._native_animation = ani

        if is_blocking_needed():
            plt.show(block=True)
            plt.close(fig)
            self.fig = None
            self.ax = None
            self._native_animation = None
        else:
            plt.show(block=False)

        return ani
