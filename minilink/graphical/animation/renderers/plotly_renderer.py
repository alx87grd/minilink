"""Plotly backend for static frames and precomputed trajectory playback."""

from __future__ import annotations

import types

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.common.plotly_style import (
    PLOTLY_2D_MARGIN,
    PLOTLY_3D_MARGIN,
    PLOTLY_ANIMATION_2D_MARGIN,
    PLOTLY_ANIMATION_3D_MARGIN,
    PLOTLY_ANIMATION_HEIGHT,
    PLOTLY_FIG_WIDTH,
    PLOTLY_TEMPLATE,
)
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
    world_to_camera,
)
from minilink.graphical.animation.renderers.renderer import AnimationRenderer


_AXIS_LABEL_BY_INDEX = ("X", "Y", "Z")
_BOX_EDGES = (
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


def _import_plotly():
    try:
        import plotly.graph_objects as go
    except ImportError as exc:
        raise ImportError(
            "Plotly rendering requires Plotly. Install with: "
            "pip install 'minilink[plotting]'",
        ) from exc
    return go


def _attach_plotly_animation_ipython_display(fig) -> None:
    """
    Attach IPython display logic so framed animations autoplay per figure.

    JupyterLab / VS Code default to the ``plotly_mimetype`` renderer, which serializes
    JSON only and never runs Plotly's HTML ``auto_play`` bootstrap. When that renderer
    is active, force ``notebook_connected`` (HTML + CDN Plotly.js) with
    ``auto_play=True``. Classic notebook stacks use ``pio.show(..., auto_play=True)``
    on the default renderer chain instead.

    ``fig._minilink_plotly_animation_opts`` must be set before calling this function so
    autoplay uses the same frame timing as the Play control (Plotly's default
    ``Plotly.animate(..., null)`` pacing is much slower).
    """

    def _ipython_display_(self):
        import plotly.io as pio

        if not (pio.renderers.render_on_display and pio.renderers.default):
            print(repr(self))
            return

        default = (pio.renderers.default or "").strip()
        tokens = [s.strip() for s in default.split("+") if s.strip()] if default else []

        uses_plotly_mimetype = False
        for name in tokens:
            try:
                renderer = pio.renderers[name]
            except Exception:
                continue
            if type(renderer).__name__ == "PlotlyRenderer":
                uses_plotly_mimetype = True
                break

        anim_opts = getattr(self, "_minilink_plotly_animation_opts", None)

        if uses_plotly_mimetype:
            pio.show(
                self,
                renderer="notebook_connected",
                auto_play=True,
                animation_opts=anim_opts,
            )
        else:
            pio.show(self, auto_play=True, animation_opts=anim_opts)

    fig._ipython_display_ = types.MethodType(_ipython_display_, fig)


def _axis_label_from_column(col) -> str:
    col = np.asarray(col, dtype=float).reshape(3)
    abs_col = np.abs(col)
    i = int(np.argmax(abs_col))
    if abs_col[i] < 0.999:
        return ""
    other = np.delete(abs_col, i)
    if np.any(other > 1e-3):
        return ""
    return ("-" if col[i] < 0 else "") + _AXIS_LABEL_BY_INDEX[i]


def _plotly_color(color: str) -> str:
    return mcolors.to_hex(color)


def _plotly_dash(style: str) -> str:
    return {
        "-": "solid",
        "--": "dash",
        ":": "dot",
        "-.": "dashdot",
    }.get(style, "solid")


def _plotly_marker(marker: str) -> str:
    return {
        "o": "circle",
        "x": "x",
        "+": "cross",
        "s": "square",
        "^": "triangle-up",
        "v": "triangle-down",
    }.get(marker, "circle")


def _pad_points(points) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    if pts.shape[1] == 2:
        pts = np.column_stack((pts, np.zeros(pts.shape[0])))
    return pts[:, :3]


def _transform_points(points, T) -> np.ndarray:
    pts = _pad_points(points)
    pts_h = np.column_stack((pts, np.ones(pts.shape[0])))
    return (np.asarray(T, dtype=float) @ pts_h.T).T[:, :3]


def _line_arrays_from_edges(vertices, edges):
    vertices = np.asarray(vertices, dtype=float)
    x, y, z = [], [], []
    for i, j in edges:
        p0 = vertices[i]
        p1 = vertices[j]
        x.extend((p0[0], p1[0], None))
        y.extend((p0[1], p1[1], None))
        z.extend((p0[2], p1[2], None))
    return x, y, z


def _max_camera_scale(frames) -> float:
    return max(1.0, *(float(frame["camera"][3, 3]) for frame in frames))


def _camera_ranges(camera):
    target = np.asarray(camera[:3, 3], dtype=float).reshape(3)
    scale = float(camera[3, 3])
    return tuple([float(c - scale), float(c + scale)] for c in target)


def _camera_is_world_xy(camera) -> bool:
    """True when 2D drawing can stay in world XY and move only axis ranges."""
    R_xy = np.asarray(camera[:3, :2], dtype=float)
    return bool(
        np.allclose(R_xy[:, 0], [1.0, 0.0, 0.0])
        and np.allclose(R_xy[:, 1], [0.0, 1.0, 0.0])
    )


def _has_dynamic_world_xy_camera(frames) -> bool:
    if not frames or not all(_camera_is_world_xy(frame["camera"]) for frame in frames):
        return False
    first = frames[0]["camera"]
    first_state = np.array(
        [first[0, 3], first[1, 3], first[3, 3]],
        dtype=float,
    )
    return any(
        not np.allclose(
            np.array(
                [frame["camera"][0, 3], frame["camera"][1, 3], frame["camera"][3, 3]],
                dtype=float,
            ),
            first_state,
        )
        for frame in frames[1:]
    )


def _circle_points(radius: float, center, *, n: int = 72) -> np.ndarray:
    center = np.asarray(center, dtype=float)
    if center.size == 2:
        center = np.array([center[0], center[1], 0.0], dtype=float)
    th = np.linspace(0.0, 2.0 * np.pi, n)
    return np.column_stack(
        (
            center[0] + float(radius) * np.cos(th),
            center[1] + float(radius) * np.sin(th),
            np.full_like(th, center[2]),
        )
    )


def _box_vertices(primitive: Box) -> np.ndarray:
    lx, ly, lz = primitive.length_x, primitive.length_y, primitive.length_z
    c = np.asarray(primitive.center, dtype=float).reshape(3)
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                corners.append(
                    np.array([sx * lx / 2, sy * ly / 2, sz * lz / 2], dtype=float) + c
                )
    return np.asarray(corners, dtype=float)


def _plane_vertices(primitive: Plane) -> np.ndarray:
    n = np.asarray(primitive.normal, dtype=float).reshape(3)
    n = n / (np.linalg.norm(n) + 1e-12)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, n))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    half = 0.5 * float(primitive.size)
    c = n * float(primitive.offset)
    return np.asarray(
        [
            c - half * u - half * v,
            c + half * u - half * v,
            c - half * u + half * v,
            c + half * u + half * v,
        ],
        dtype=float,
    )


def _line_trace(go, *, x, y, z=None, color, width, dash, name, is_3d):
    if is_3d:
        return go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines",
            line={"color": color, "width": width},
            name=name,
            showlegend=False,
        )
    return go.Scatter(
        x=x,
        y=y,
        mode="lines",
        line={"color": color, "width": width, "dash": dash},
        name=name,
        showlegend=False,
    )


def _marker_trace(go, *, x, y, z=None, color, size, symbol, name, is_3d):
    if is_3d:
        return go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="markers",
            marker={"color": color, "size": size},
            name=name,
            showlegend=False,
        )
    return go.Scatter(
        x=x,
        y=y,
        mode="markers",
        marker={"color": color, "size": size, "symbol": symbol},
        name=name,
        showlegend=False,
    )


def _empty_trace(go, *, is_3d, name="unsupported"):
    if is_3d:
        return go.Scatter3d(x=[], y=[], z=[], mode="lines", name=name, showlegend=False)
    return go.Scatter(x=[], y=[], mode="lines", name=name, showlegend=False)


class PlotlyRenderer(AnimationRenderer):
    """
    Notebook-friendly Plotly renderer.

    This backend is intended for static figures and precomputed trajectory
    playback. It deliberately does not implement a high-frequency live update
    loop for ``game()`` or trajectory-optimization iteration callbacks.
    """

    supports_interactive = False

    def __init__(self, animator):
        super().__init__(animator)
        self.fig = None
        self.is_3d = False
        self.show = True
        self.title = None

    def open_scene(
        self,
        *,
        is_3d: bool,
        show: bool,
        camera,
        title: str | None = None,
    ) -> None:
        _import_plotly()
        self.fig = None
        self.is_3d = is_3d
        self.show = show
        self.title = title

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        traces = self._traces_for_frame(primitives, transforms, camera)
        title = self.title or f"{self.sys.name} — t = {t:.2f} s"
        self.fig = self._build_figure(traces, camera, title=title)

    def present(self, *, block: bool, interval_s: float | None = None):
        if self.fig is not None and self.show:
            self.fig.show()
        return self.fig

    def poll_events(self):
        return {"quit": False}

    def close_scene(self) -> None:
        self.fig = None

    def render_inline_animation(self, primitives, frames, schedule, *, is_3d: bool):
        self.is_3d = is_3d
        return self._build_animation_figure(primitives, frames, schedule)

    def play_native(self, primitives, frames, schedule, *, is_3d: bool):
        self.is_3d = is_3d
        fig = self._build_animation_figure(primitives, frames, schedule)
        fig.show(
            auto_play=True,
            animation_opts=getattr(fig, "_minilink_plotly_animation_opts", None),
        )
        self.fig = fig
        return fig

    def _build_animation_figure(self, primitives, frames, schedule):
        go = _import_plotly()
        if not frames:
            raise ValueError("Cannot animate an empty frame list.")

        frame_traces = [
            self._traces_for_frame(
                primitives,
                frame["transforms"],
                frame["camera"],
            )
            for frame in frames
        ]
        initial = frames[0]
        fig = self._build_figure(
            frame_traces[0],
            initial["camera"],
            title=f"Animation: {self.sys.name}",
        )
        fig.update_layout(**self._fixed_animation_layout(frame_traces, frames))

        dynamic_xy_layout = (not self.is_3d) and _has_dynamic_world_xy_camera(frames)

        plotly_frames = []
        for i, traces in enumerate(frame_traces):
            layout = self._xy_layout(frames[i]["camera"]) if dynamic_xy_layout else None
            plotly_frames.append(go.Frame(data=traces, layout=layout, name=str(i)))
        fig.frames = tuple(plotly_frames)

        frame_duration = max(1, int(round(schedule.interval_ms)))
        # Plotly HTML autoplay calls Plotly.animate(gd, null, opts). Without opts,
        # plotly.js uses a slow default step duration; match the Play button / FuncAnimation pacing.
        fig._minilink_plotly_animation_opts = {
            "frame": {"duration": frame_duration, "redraw": True},
            "fromcurrent": True,
            "transition": {"duration": 0},
        }
        slider_steps = [
            {
                "args": [
                    [str(i)],
                    {
                        "frame": {"duration": 0, "redraw": True},
                        "mode": "immediate",
                        "transition": {"duration": 0},
                    },
                ],
                "label": f"{frame['t']:.2f}",
                "method": "animate",
            }
            for i, frame in enumerate(frames)
        ]
        fig.update_layout(
            margin=self._animation_margin(),
            updatemenus=[
                {
                    "type": "buttons",
                    "showactive": False,
                    "x": 0.0,
                    "y": -0.08,
                    "xanchor": "left",
                    "yanchor": "top",
                    "direction": "left",
                    "pad": {"r": 10, "t": 0},
                    "buttons": [
                        {
                            "label": "Play",
                            "method": "animate",
                            "args": [
                                None,
                                {
                                    "frame": {
                                        "duration": frame_duration,
                                        "redraw": True,
                                    },
                                    "fromcurrent": True,
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                        {
                            "label": "Pause",
                            "method": "animate",
                            "args": [
                                [None],
                                {
                                    "frame": {"duration": 0, "redraw": True},
                                    "mode": "immediate",
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                    ],
                }
            ],
            sliders=[
                {
                    "active": 0,
                    "currentvalue": {"prefix": "t = ", "suffix": " s"},
                    "pad": {"t": 35, "b": 5},
                    "steps": slider_steps,
                }
            ],
        )
        _attach_plotly_animation_ipython_display(fig)
        return fig

    def _build_figure(self, traces, camera, *, title: str):
        go = _import_plotly()
        fig = go.Figure(data=traces)
        if self.is_3d:
            self._apply_3d_layout(fig, camera, title=title)
        else:
            self._apply_2d_layout(fig, camera, title=title)
        return fig

    def _fixed_animation_layout(self, frame_traces, frames):
        if self.is_3d:
            return {"scene": self._scene_layout(frames[0]["camera"])}
        return self._xy_layout(
            frames[0]["camera"],
            extent=_max_camera_scale(frames),
        )

    def _xy_layout(self, camera, *, extent: float | None = None):
        scale = float(camera[3, 3]) if extent is None else float(extent)
        target = np.asarray(camera[:3, 3], dtype=float).reshape(3)
        x_label = _axis_label_from_column(camera[:3, 0])
        y_label = _axis_label_from_column(camera[:3, 1])
        if _camera_is_world_xy(camera):
            x_range = [float(target[0] - scale), float(target[0] + scale)]
            y_range = [float(target[1] - scale), float(target[1] + scale)]
        else:
            x_range = [-scale, scale]
            y_range = [-scale, scale]
        return {
            "xaxis": {
                "range": x_range,
                "autorange": False,
                "title": {"text": x_label},
                "zeroline": True,
                "scaleanchor": "y",
                "scaleratio": 1,
            },
            "yaxis": {
                "range": y_range,
                "autorange": False,
                "title": {"text": y_label},
                "zeroline": True,
            },
        }

    def _apply_2d_layout(self, fig, camera, *, title: str) -> None:
        fig.update_layout(
            title=title,
            showlegend=False,
            margin=dict(PLOTLY_2D_MARGIN),
            template=PLOTLY_TEMPLATE,
            width=PLOTLY_FIG_WIDTH,
            height=PLOTLY_ANIMATION_HEIGHT,
            **self._xy_layout(camera),
        )

    def _apply_3d_layout(self, fig, camera, *, title: str) -> None:
        fig.update_layout(
            title=title,
            showlegend=False,
            margin=dict(PLOTLY_3D_MARGIN),
            template=PLOTLY_TEMPLATE,
            width=PLOTLY_FIG_WIDTH,
            height=PLOTLY_ANIMATION_HEIGHT,
            scene=self._scene_layout(camera),
        )

    def _animation_margin(self) -> dict:
        if self.is_3d:
            return dict(PLOTLY_ANIMATION_3D_MARGIN)
        return dict(PLOTLY_ANIMATION_2D_MARGIN)

    def _scene_layout(self, camera, *, ranges=None):
        if ranges is None:
            ranges = _camera_ranges(camera)
        view_out = np.asarray(camera[:3, 2], dtype=float).reshape(3)
        n = np.linalg.norm(view_out)
        if n < 1e-12:
            view_out = np.array([1.25, 1.25, 1.25], dtype=float)
        else:
            view_out = view_out / n * 2.0
        up = np.asarray(camera[:3, 1], dtype=float).reshape(3)

        return {
            "xaxis": {"range": ranges[0], "autorange": False},
            "yaxis": {"range": ranges[1], "autorange": False},
            "zaxis": {"range": ranges[2], "autorange": False},
            "aspectmode": "cube",
            "camera": {
                "eye": {
                    "x": float(view_out[0]),
                    "y": float(view_out[1]),
                    "z": float(view_out[2]),
                },
                "up": {
                    "x": float(up[0]),
                    "y": float(up[1]),
                    "z": float(up[2]),
                },
                "projection": {"type": "orthographic"},
            },
        }

    def _traces_for_frame(self, primitives, transforms, camera):
        go = _import_plotly()
        if self.is_3d or _camera_is_world_xy(camera):
            draw_transforms = transforms
        else:
            W = world_to_camera(camera)
            draw_transforms = [W @ T for T in transforms]
        return [
            self._trace_for_primitive(go, prim, T, i)
            for i, (prim, T) in enumerate(zip(primitives, draw_transforms))
        ]

    def _trace_for_primitive(self, go, primitive, T, index: int):
        color = _plotly_color(getattr(primitive, "color", "black"))
        width = max(1.0, float(getattr(primitive, "linewidth", 1.0)))
        dash = _plotly_dash(getattr(primitive, "style", "-"))
        name = f"{primitive.__class__.__name__} {index}"

        if isinstance(primitive, Point):
            pts = _transform_points(primitive.pt, T)
            return _marker_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2] if self.is_3d else None,
                color=color,
                size=max(4.0, float(primitive.size)),
                symbol=_plotly_marker(primitive.marker),
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, (CustomLine, Arrow)):
            pts = _transform_points(primitive.pts, T)
            return _line_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2] if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, TorqueArrow):
            sweep, T_rigid = extract_amplitude(T)
            pts = _transform_points(primitive.compute_pts(sweep), T_rigid)
            return _line_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2] if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, Circle):
            if self.is_3d:
                pts = _transform_points(
                    _circle_points(primitive.radius, primitive.center),
                    T,
                )
            else:
                center = _transform_points(primitive.center, T)[0]
                pts = _circle_points(primitive.radius, center)
            trace = _line_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2] if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )
            if not self.is_3d and bool(primitive.fill):
                trace.fill = "toself"
                trace.fillcolor = color
            return trace

        if isinstance(primitive, Sphere):
            center = _transform_points(primitive.center, T)[0]
            if self.is_3d:
                return _marker_trace(
                    go,
                    x=[center[0]],
                    y=[center[1]],
                    z=[center[2]],
                    color=color,
                    size=max(4.0, 18.0 * float(primitive.radius)),
                    symbol="circle",
                    name=name,
                    is_3d=True,
                )
            pts = _circle_points(primitive.radius, center)
            trace = _line_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=False,
            )
            trace.fill = "toself"
            trace.fillcolor = color
            trace.opacity = float(np.clip(primitive.opacity, 0.0, 1.0))
            return trace

        if isinstance(primitive, Rod):
            local = np.array([[0.0, 0.0, 0.0], [0.0, -primitive.length, 0.0]])
            pts = _transform_points(local, T)
            return _line_trace(
                go,
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2] if self.is_3d else None,
                color=color,
                width=max(width, 2.0),
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, Plane):
            vertices = _transform_points(_plane_vertices(primitive), T)
            x, y, z = _line_arrays_from_edges(vertices, ((0, 1), (1, 3), (3, 2), (2, 0)))
            return _line_trace(
                go,
                x=x,
                y=y,
                z=z if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, Box):
            vertices = _transform_points(_box_vertices(primitive), T)
            x, y, z = _line_arrays_from_edges(vertices, _BOX_EDGES)
            return _line_trace(
                go,
                x=x,
                y=y,
                z=z if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        if isinstance(primitive, ExtrudedPolygon):
            vertices = _transform_points(primitive.vertices_local(), T)
            x, y, z = _line_arrays_from_edges(vertices, primitive.edges())
            return _line_trace(
                go,
                x=x,
                y=y,
                z=z if self.is_3d else None,
                color=color,
                width=width,
                dash=dash,
                name=name,
                is_3d=self.is_3d,
            )

        return _empty_trace(go, is_3d=self.is_3d, name=name)
