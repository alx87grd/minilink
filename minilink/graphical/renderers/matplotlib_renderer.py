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

    def render_static(self, x, u, t: float, is_3d: bool) -> None:
        fig, ax = self._create_figure_and_ax(is_3d)
        canvas = MatplotlibCanvas(ax, is_3d=is_3d)

        primitives = self.sys.get_kinematic_geometry()
        transforms = self.sys.get_kinematic_transforms(x, u, t)

        if len(primitives) != len(transforms):
            raise ValueError(
                "System graphical error: Number of transforms must equal number of base geometric primitives."
            )

        for prim, T in zip(primitives, transforms):
            canvas.draw_primitive(prim, T)

        ax.set_title(f"Time = {t:.2f} s")
        plt.show(block=True)

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
        fig, ax = self._create_figure_and_ax(is_3d)
        canvas = MatplotlibCanvas(ax, is_3d=is_3d)
        primitives = self.sys.get_kinematic_geometry()

        from minilink.graphical.renderers.timing import (
            sim_index_for_frame,
            trajectory_frame_schedule,
        )

        sched = trajectory_frame_schedule(traj, time_factor_video)
        n_frames = sched.n_frames
        interval_ms = sched.interval_ms
        target_fps = sched.target_fps
        time_template = "Time = %.2f s"

        def update(frame_idx):
            canvas.clear()
            sim_idx = sim_index_for_frame(frame_idx, sched)
            x = traj.x[:, sim_idx]
            if len(traj.u) > 0:
                u = traj.u[:, sim_idx]
            else:
                u = np.array([])
            t = traj.t[sim_idx]

            transforms = self.sys.get_kinematic_transforms(x, u, t)

            if len(primitives) != len(transforms):
                raise ValueError(
                    "System graphical error: Number of transforms must equal number of base geometric primitives."
                )

            for prim, T in zip(primitives, transforms):
                canvas.draw_primitive(prim, T)

            ax.set_title(time_template % t)
            return canvas.drawn_objects

        ani = animation.FuncAnimation(
            fig, update, frames=n_frames, interval=interval_ms, blit=False
        )

        if save:
            print(f"Saving animation to {file_name}.gif ...")
            ani.save(file_name + ".gif", writer="imagemagick", fps=target_fps)

        if html:
            plt.close(fig)
            try:
                from IPython.display import HTML

                return HTML(ani.to_jshtml())
            except ImportError:
                print(
                    "IPython is not available. Ensure you are running in a Jupyter Notebook environment."
                )
                return ani

        if show:
            plt.show(block=True)
        else:
            plt.close(fig)

        return ani
