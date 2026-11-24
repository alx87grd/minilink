"""Matplotlib backend: static figure and FuncAnimation playback."""

from __future__ import annotations

from itertools import product
from pathlib import Path
from tempfile import TemporaryDirectory

import matplotlib as mpl
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.image as image
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from PIL import Image

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
    world_to_camera,
)
from minilink.graphical.animation.renderers.renderer import (
    AnimationRenderer,
    axis_label_from_column,
    camera_is_world_xy,
)
from minilink.graphical.common.environment import is_blocking_needed
from minilink.graphical.common.matplotlib_style import (
    DPI_EXPORT,
    DPI_FIGURE,
    FIGSIZE_ANIMATION,
    FONT_SIZE,
    style_animation_axes,
)
from minilink.graphical.common.show import hold_back, show_in_window


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

            alpha = float(np.clip(primitive.opacity, 0.0, 1.0))
            if self.is_3d:
                (obj,) = self.ax.plot(
                    x,
                    y,
                    z,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                    alpha=alpha,
                )
            else:
                (obj,) = self.ax.plot(
                    x,
                    y,
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                    alpha=alpha,
                )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, Arrow):
            # Arrow geometry is baked into ``pts``; draw the polyline at its pose.
            local_pts = primitive.pts
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            if self.is_3d:
                (obj,) = self.ax.plot(
                    world_pts[:, 0],
                    world_pts[:, 1],
                    world_pts[:, 2],
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            else:
                (obj,) = self.ax.plot(
                    world_pts[:, 0],
                    world_pts[:, 1],
                    color=primitive.color,
                    linewidth=primitive.linewidth,
                    linestyle=primitive.style,
                )
            self.drawn_objects.append(obj)

        elif isinstance(primitive, TorqueArrow):
            # Honest torque arc: sweep baked into ``pts``. In 2D draw the arc and
            # the chevron head as two separate Line2D so the anti-aliased join is
            # crisp; in 3D draw a single polyline.
            local_pts = primitive.pts
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            if self.is_3d:
                if local_pts.shape[0] >= 2:
                    (obj,) = self.ax.plot(
                        world_pts[:, 0],
                        world_pts[:, 1],
                        world_pts[:, 2],
                        color=primitive.color,
                        linewidth=primitive.linewidth,
                        linestyle=primitive.style,
                    )
                    self.drawn_objects.append(obj)
            else:
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
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            world_center = transform_matrix @ np.append(local_center, 1.0)
            x, y, z = world_center[0], world_center[1], world_center[2]
            alpha = float(np.clip(primitive.opacity, 0.0, 1.0))

            if self.is_3d:
                # 3D: three orthogonal great circles read as a ball and scale
                # with the data axes (Axes3D cannot draw a 2D circle patch).
                r = primitive.radius
                th = np.linspace(0.0, 2.0 * np.pi, 24)
                cos, sin, zero = np.cos(th), np.sin(th), np.zeros_like(th)
                for rx, ry, rz in (
                    (r * cos, r * sin, zero),
                    (r * cos, zero, r * sin),
                    (zero, r * cos, r * sin),
                ):
                    (obj,) = self.ax.plot(
                        x + rx,
                        y + ry,
                        z + rz,
                        color=primitive.color,
                        linewidth=1.0,
                        alpha=alpha,
                    )
                    self.drawn_objects.append(obj)
            else:
                # 2D fallback: draw the sphere as a filled circle in XY.
                circ = patches.Circle(
                    (x, y),
                    radius=primitive.radius,
                    ec=primitive.color,
                    fill=True,
                    fc=primitive.color,
                    alpha=alpha,
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
        self.layers = None  # The frames drawn in layers, flat views only
        self.window_shown = False  # A live scene's window has had its first draw

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
            if camera_is_world_xy(camera):
                ax.set_xlim(target[0] - scale, target[0] + scale)
                ax.set_ylim(target[1] - scale, target[1] + scale)
            else:
                ax.set_xlim(-scale, scale)
                ax.set_ylim(-scale, scale)
            ax.set_xlabel(axis_label_from_column(camera[:3, 0]))
            ax.set_ylabel(axis_label_from_column(camera[:3, 1]))
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
        self.layers = None if is_3d else LayeredFrames(self.fig, self.ax, self.canvas)
        self.window_shown = False
        if title:
            self.ax.set_title(title, fontsize=FONT_SIZE)

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        self.canvas.clear()
        # Normal 2D XY views keep geometry in world coordinates and move only the
        # axis limits, so the scene scrolls under a fixed drawing. Other 2D axis
        # pairs still project onto the selected camera plane.
        # 3D matplotlib keeps world coordinates; limits and ``view_init`` come from the
        # camera once at ``open_scene``, then the interactive UI owns the viewpoint.
        if self.canvas.is_3d or camera_is_world_xy(camera):
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
        window = self.fig.canvas
        if (
            self.window_shown
            and self.layers is not None
            and isinstance(window, FigureCanvasAgg)
        ):
            # The frame in layers, pushed to the window, then the window's events
            self.layers.draw_on(window)
            window.blit(self.fig.bbox)
            if interval_s is None:
                window.flush_events()
            else:
                window.start_event_loop(interval_s)
            return
        # The first frame opens the window, with a full draw
        plt.pause(0.001 if interval_s is None else interval_s)
        self.window_shown = True

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
        self.layers = None
        self.window_shown = False

    def _build_animation(
        self,
        primitives,
        frames,
        schedule,
        *,
        is_3d: bool = False,
        scene_title: str | None = None,
    ):
        fig, ax = self._create_figure_and_ax(is_3d=is_3d, camera=frames[0]["camera"])
        if scene_title:
            fig.suptitle(scene_title, fontsize=FONT_SIZE)
        canvas = MatplotlibCanvas(ax, is_3d=is_3d)

        def update(frame_idx):
            frame = frames[frame_idx]
            canvas.clear()
            camera = frame["camera"]
            frame_primitives = frame["primitives"]
            if is_3d or camera_is_world_xy(camera):
                draw_transforms = frame["transforms"]
            else:
                W = world_to_camera(camera)
                draw_transforms = [W @ T for T in frame["transforms"]]
            for prim, T in zip(frame_primitives, draw_transforms):
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

        # The first frame is on the figure before the animation plays
        update(0)
        self.layers = None if is_3d else LayeredFrames(fig, ax, canvas)

        return fig, ani

    def build_clip(
        self, primitives, frames, schedule, *, is_3d=False, scene_title=None
    ):
        """The trajectory as a ``FuncAnimation`` on a figure of its own: ``(fig, ani)``."""
        return self._build_animation(
            primitives, frames, schedule, is_3d=is_3d, scene_title=scene_title
        )

    def show_clip(self, clip, place):
        """Play the clip inline (an HTML player) or in its window."""
        fig, ani = clip.built()
        show_matplotlib_clip(
            fig, ani, place, layers=self.layers, fps=1000.0 / clip.schedule.interval_ms
        )

    def save_clip(self, clip, path):
        """A ``.gif`` (the default) or a video through ffmpeg (``.mp4``); a clip of one frame also as an image (``.png``, ``.pdf``, ``.svg``, …)."""
        path = Path(path)
        is_still = len(clip.frames) == 1 and path.suffix.lower() not in CLIP_SUFFIXES
        if not is_still:
            require_clip_suffix(path)
        fig, ani = clip.built()
        if is_still:
            fig.savefig(path, bbox_inches="tight")
            written = path

            # The still is the clip's output: matplotlib then holds no warning
            # for an animation never played
            ani._draw_was_started = True
        else:
            written = save_matplotlib_clip(
                ani, path, clip.schedule.target_fps, layers=self.layers
            )

        # A figure built for the file alone is not left for the notebook to show
        hold_back(fig, "matplotlib")

        return written


# Playing and saving a clip

# The files a matplotlib animation saves as: a GIF (also without a suffix), a video
CLIP_SUFFIXES = ("", ".gif", ".mp4")

# The animations playing in a window: a FuncAnimation stops when nothing holds it,
# so each is kept here until its window closes
PLAYING = set()


def show_matplotlib_clip(fig, ani, place, *, layers=None, fps=None):
    """
    Play ``ani`` inline, as an HTML player displayed now, or in its window.

    With ``layers`` (and the clip's ``fps``), the player's frames are drawn in
    layers: the same images, without a full draw of the figure for each.
    """
    if place == "inline":
        # Lazy import: IPython is optional, needed only in a notebook
        from IPython.display import HTML, display

        display(HTML(player_html(ani, layers=layers, fps=fps)))
        plt.close(fig)
        return

    # A window, alone: held until it closes; a bare script blocks on it
    PLAYING.add(ani)
    fig.canvas.mpl_connect("close_event", lambda event: PLAYING.discard(ani))
    show_in_window(fig)


def player_html(ani, *, layers=None, fps=None) -> str:
    """The HTML player of ``ani``, its frames embedded."""
    if layers is None or fps is None:
        return ani.to_jshtml()

    with TemporaryDirectory() as folder:
        path = Path(folder, "player.html")
        writer = LayeredHTMLWriter(
            layers, fps=fps, embed_frames=True, default_mode="loop"
        )
        ani.save(str(path), writer=writer)
        html = path.read_text()

    return html


def save_matplotlib_clip(ani, path, fps, *, layers=None):
    """
    Write ``ani`` to ``path``: a GIF without a suffix or with ``.gif``, an ``.mp4`` video through ffmpeg.

    With ``layers``, the frames are drawn in layers: the same images, without
    a full draw of the figure for each.
    """
    path = Path(path)
    require_clip_suffix(path)
    if not path.suffix:
        path = path.with_suffix(".gif")
    is_gif = path.suffix.lower() == ".gif"
    if is_gif:
        writer = (
            ClipPaletteWriter(fps=fps)
            if layers is None
            else LayeredPillowWriter(layers, fps=fps)
        )
        ani.save(str(path), writer=writer, dpi=DPI_EXPORT)
    elif layers is None or not animation.writers.is_available("ffmpeg"):
        ani.save(str(path), writer="ffmpeg", fps=fps, dpi=DPI_EXPORT)
    else:
        writer = LayeredFFMpegWriter(layers, fps=fps)
        ani.save(str(path), writer=writer, dpi=DPI_EXPORT)

    return path


def require_clip_suffix(path):
    """Refuse a file a matplotlib animation cannot be: only ``.gif`` and ``.mp4`` (a GIF without a suffix)."""
    if path.suffix.lower() not in CLIP_SUFFIXES:
        raise ValueError(
            "a matplotlib animation saves as a .gif or an .mp4 video, "
            f"got {path.name!r}; pass a file name ending in .gif or .mp4"
        )


# Frames drawn in layers
#
# A full draw of a figure spends most of its time on what does not move: the tick
# labels and their fonts, the axis labels, the figure itself. A clip's frames
# share them while the view holds still, so they are drawn once and kept as an
# image; a frame is that image with the rest drawn over it, in the order of a
# full draw.


class LayeredFrames:
    """
    The frames of a flat animation as images, each the same as a full draw of the figure.

    ``rgba()`` gives the frame the figure holds. While the view (the axis
    limits and labels) is the one of the frame before, the background is the
    image kept for that view, and only the primitives' artists, the grid and
    tick lines, the frame and the title are drawn over it. A frame on a new
    view is a full draw.
    """

    def __init__(self, fig, ax, canvas):
        self.fig = fig
        self.ax = ax
        self.canvas = canvas  # The MatplotlibCanvas: its drawn_objects are the frame's
        self.shown = None  # The figure's own canvas and dpi, set back at close
        self.shown_dpi = None
        self.agg = None
        self.view = None  # The view of the last frame
        self.title_position = None  # Where a full draw of this view puts the title
        self.background = None  # (view, image, ticks drawn) of the kept background

    def open(self, dpi) -> None:
        """Draw at ``dpi`` on a canvas of its own until :meth:`close`."""
        self.shown = self.fig.canvas
        self.shown_dpi = self.fig.dpi
        # on the canvas of its own, the resolution changes no window's size
        self.agg = QuietCanvas(self.fig)
        self.fig.set_dpi(dpi)
        self.view = None
        self.background = None

    def close(self) -> None:
        """The figure back on its own canvas, at its own resolution."""
        if self.shown is None:
            return
        self.fig.set_dpi(self.shown_dpi)
        self.fig.set_canvas(self.shown)
        self.shown = None
        self.agg = None
        self.background = None

    def rgba(self) -> np.ndarray:
        """The frame the figure holds, ``(height, width, 4)`` bytes."""
        self.draw_on(self.agg)

        return np.asarray(self.agg.buffer_rgba())

    def draw_on(self, agg) -> None:
        """Draw the frame the figure holds into the image of ``agg``, an Agg canvas of the figure."""
        self.agg = agg
        view = self.view_key()
        if not self.layered() or view != self.view:
            # A new view: a full draw, which also places its ticks
            self.view = view
            self.background = None
            agg.draw()
            self.title_position = self.ax.title.get_position()
        else:
            if self.background is None or self.background[0] != view:
                self.keep_background(view)
            self.draw_over_background()

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def view_key(self) -> tuple:
        """What the background depends on: the limits, the axis labels, the image's size."""
        ax = self.ax
        return (
            ax.get_xlim(),
            ax.get_ylim(),
            ax.get_xlabel(),
            ax.get_ylabel(),
            self.agg.get_width_height(),
            self.fig.dpi,
        )

    def layered(self) -> bool:
        """True when the axes hold nothing but what the layers know how to order."""
        ax = self.ax
        known = {
            *(id(artist) for artist in self.canvas.drawn_objects),
            *(id(spine) for spine in ax.spines.values()),
            id(ax.xaxis),
            id(ax.yaxis),
            id(ax.title),
            id(ax.patch),
        }
        others = [
            child
            for child in ax.get_children()
            if id(child) not in known and not is_empty_text(child)
        ]
        return not others and ax.axison and ax.get_frame_on()

    def keep_background(self, view) -> None:
        """Draw and keep what stays on this view: everything but the layers drawn per frame."""
        # The ticks the last full draw placed on this view
        ticks = {axis: axis._update_ticks() for axis in (self.ax.xaxis, self.ax.yaxis)}
        over = self.layers_over(ticks)
        was_visible = [artist.get_visible() for artist in over]
        for artist in over:
            artist.set_visible(False)
        try:
            # into the image only: a window is shown the finished frame
            FigureCanvasAgg.draw(self.agg)
            image_kept = self.agg.copy_from_bbox(self.fig.bbox)
        finally:
            for artist, visible in zip(over, was_visible):
                artist.set_visible(visible)
        self.background = (view, image_kept, ticks)

    def layers_over(self, ticks) -> list:
        """The artists drawn over the background, in the order a full draw gives them."""
        ax = self.ax
        children = [*self.canvas.drawn_objects, *ax.spines.values()]
        children += [ax.xaxis, ax.yaxis, ax.title]
        ordered = sorted(children, key=lambda artist: artist.zorder)
        over = []
        for artist in ordered:
            if artist in ticks:
                # an axis draws, tick by tick, its grid line then its tick lines;
                # its labels are on the background
                for tick in ticks[artist]:
                    over += [tick.gridline, tick.tick1line, tick.tick2line]
            else:
                over.append(artist)

        return over

    def draw_over_background(self) -> None:
        _, image_kept, ticks = self.background
        renderer = self.agg.get_renderer()
        self.agg.restore_region(image_kept)
        # a draw of the axes places the title; the layers put it where the
        # full draw of this view did
        self.ax.title.set_position(self.title_position)
        for artist in self.layers_over(ticks):
            artist.draw(renderer)


class QuietCanvas(FigureCanvasAgg):
    """An image canvas that draws only when a frame is asked for, never on a redraw request."""

    def draw_idle(self, *args, **kwargs):
        pass


def is_empty_text(artist) -> bool:
    """True for a text artist with nothing to draw (the unused side titles of an axes)."""
    return hasattr(artist, "get_text") and artist.get_text() == ""


class LayeredFigure:
    """Stands in for the figure in a movie writer: ``savefig`` writes the layered frame."""

    def __init__(self, fig, layers):
        self.fig = fig
        self.layers = layers

    def savefig(self, target, *, format, dpi, **savefig_kwargs):
        if format not in ("png", "rgba") or not keeps_figure_colours(
            self.fig, savefig_kwargs
        ):
            # a frame the layers do not write: the figure's own, a full draw
            self.fig.savefig(target, format=format, dpi=dpi, **savefig_kwargs)
            return
        rgba = self.layers.rgba()
        if format == "png":
            image.imsave(target, rgba, format="png", origin="upper", dpi=dpi)
        else:
            target.write(rgba.tobytes())

    def __getattr__(self, name):
        if name == "fig":
            raise AttributeError(name)

        return getattr(self.fig, name)


def keeps_figure_colours(fig, savefig_kwargs) -> bool:
    """True when a save with these options paints the figure in its own face and edge colours."""
    if savefig_kwargs.get("transparent", mpl.rcParams["savefig.transparent"]):
        return False
    for side, own in (
        ("facecolor", fig.get_facecolor()),
        ("edgecolor", fig.get_edgecolor()),
    ):
        colour = savefig_kwargs.get(side, mpl.rcParams[f"savefig.{side}"])
        if isinstance(colour, str) and colour == "auto":
            continue
        if not mcolors.same_color(colour, own):
            return False

    return True


class LayeredSaving:
    """A movie writer whose frames come from :class:`LayeredFrames` (``png`` or raw ``rgba`` frames)."""

    def __init__(self, layers, **kwargs):
        super().__init__(**kwargs)
        self.layers = layers

    def setup(self, fig, outfile, dpi=None, **kwargs):
        super().setup(fig, outfile, dpi=dpi, **kwargs)
        self.layers.open(self.dpi)
        self.fig = LayeredFigure(fig, self.layers)

    def finish(self):
        if isinstance(self.fig, LayeredFigure):
            self.fig = self.fig.fig
        self.layers.close()
        super().finish()


class LayeredHTMLWriter(LayeredSaving, animation.HTMLWriter):
    """The HTML player, its frames drawn in layers."""


class ClipPaletteWriter(animation.PillowWriter):
    """
    The GIF writer with one palette for the whole clip.

    A GIF holds 256 colours. Reducing each frame to a palette of its own is
    most of the time of a save; here the palette is reduced once, from frames
    spread over the clip, and every frame's colours go to their nearest entry.
    """

    def finish(self):
        frames = self._frames
        if any(frame.mode != "RGB" for frame in frames):
            # frames with transparency keep the writer's own reduction
            super().finish()
            return
        indexed = frames_on_one_palette(frames)
        indexed[0].save(
            self.outfile,
            save_all=True,
            append_images=indexed[1:],
            duration=int(1000 / self.fps),
            loop=0,
        )


class LayeredPillowWriter(LayeredSaving, ClipPaletteWriter):
    """The GIF writer, its frames drawn in layers."""


class LayeredFFMpegWriter(LayeredSaving, animation.FFMpegWriter):
    """The ffmpeg video writer, its frames drawn in layers."""


# One palette for a clip

# Frames, spread over the clip, the palette is reduced from
_PALETTE_FRAMES = 6


def frames_on_one_palette(frames) -> list:
    """RGB ``frames`` as palette images sharing one palette of 256 colours."""
    # The palette: the median cut of a mosaic of frames spread over the clip
    picks = np.linspace(0, len(frames) - 1, min(_PALETTE_FRAMES, len(frames)))
    picks = [frames[int(i)] for i in picks]
    width, height = picks[0].size
    mosaic = Image.new("RGB", (width, height * len(picks)))
    for i, frame in enumerate(picks):
        mosaic.paste(frame, (0, i * height))
    reduced = mosaic.convert("P", palette=Image.Palette.ADAPTIVE)
    flat = reduced.getpalette()
    palette = np.array(flat, dtype=np.int32).reshape(-1, 3)

    # Each colour of a frame to its nearest entry, found once per colour and
    # kept in a table over the 2^24 colours
    entry = np.zeros(1 << 24, dtype=np.uint8)
    known = np.zeros(1 << 24, dtype=bool)
    indexed = []
    for frame in frames:
        rgb = np.asarray(frame)
        code = (
            (rgb[..., 0].astype(np.uint32) << 16)
            | (rgb[..., 1].astype(np.uint32) << 8)
            | rgb[..., 2]
        )
        new = np.unique(code[~known[code]])
        if len(new):
            colours = np.stack([new >> 16, (new >> 8) & 255, new & 255], axis=1)
            distance = (colours[:, None, :].astype(np.int32) - palette[None]) ** 2
            entry[new] = np.argmin(distance.sum(axis=2), axis=1)
            known[new] = True
        image_p = Image.fromarray(entry[code], mode="P")
        image_p.putpalette(flat)
        indexed.append(image_p)

    return indexed
