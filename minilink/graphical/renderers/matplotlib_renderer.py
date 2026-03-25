"""Matplotlib backend: static figure and FuncAnimation playback."""

from __future__ import annotations

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from minilink.graphical.primitives import Circle, CustomLine, Point
from minilink.graphical.renderers.base import AnimationRenderer


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

    def _create_figure_and_ax(self, is_3d):
        a = self.animator
        fig = plt.figure(figsize=a.figsize, dpi=a.dpi)
        fig.canvas.manager.set_window_title(f"Animation: {self.sys.name}")

        if is_3d:
            ax = fig.add_subplot(111, projection="3d")
            ax.set_xlim3d(a.domain[0])
            ax.set_ylim3d(a.domain[1])
            ax.set_zlim3d(a.domain[2])
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
        else:
            ax = fig.add_subplot(111)
            ax.set_xlim(a.domain[0])
            ax.set_ylim(a.domain[1])
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.grid(True)
            ax.set_aspect("equal")

        return fig, ax

    def open_scene(self, *, is_3d: bool, show: bool, title: str | None = None) -> None:
        self.fig, self.ax = self._create_figure_and_ax(is_3d)
        self.canvas = MatplotlibCanvas(self.ax, is_3d=is_3d)
        if title:
            self.ax.set_title(title)

    def draw_frame(self, primitives, transforms, t: float) -> None:
        self.canvas.clear()
        for prim, T in zip(primitives, transforms):
            self.canvas.draw_primitive(prim, T)
        self.ax.set_title(f"Time = {t:.2f} s")

    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        if block:
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

    def _build_animation(self, primitives, frames, schedule):
        fig, ax = self._create_figure_and_ax(is_3d=False)
        canvas = MatplotlibCanvas(ax, is_3d=False)

        def update(frame_idx):
            frame = frames[frame_idx]
            canvas.clear()
            for prim, T in zip(primitives, frame["transforms"]):
                canvas.draw_primitive(prim, T)
            ax.set_title(f"Time = {frame['t']:.2f} s")
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
        ani.save(file_name + ".gif", writer="imagemagick", fps=schedule.target_fps)
        plt.close(fig)
