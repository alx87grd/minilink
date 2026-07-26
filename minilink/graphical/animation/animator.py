"""
Trajectory animation orchestration.

The :class:`Animator` drives the frame-keyed drawable contract
(:meth:`~minilink.core.system.System.tf` /
:meth:`~minilink.core.system.System.get_kinematic_geometry` /
:meth:`~minilink.core.system.System.get_dynamic_geometry`) through the renderer
backends under :mod:`minilink.graphical.animation.renderers` (picked by name via
:func:`make_renderer`). Per frame it resolves the named frame poses, flattens the
cached kinematic geometry and the per-frame dynamic geometry into the flat
``(primitive, world 4x4)`` list the renderers consume, and resolves the camera
from the system's hints.

Live sessions (interactive ``game()`` mode) are orchestrated by
:class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`, which
drives :meth:`Animator.open_live_scene` / :meth:`Animator.update_live_frame`
per frame; this module stays playback and frame resolution only.
"""

from __future__ import annotations

import warnings

import numpy as np

from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.drawables import validate_overlay
from minilink.graphical.animation.renderers.matplotlib_renderer import (
    MatplotlibRenderer,
)
from minilink.graphical.animation.renderers.meshcat_renderer import MeshcatRenderer
from minilink.graphical.animation.renderers.plotly_renderer import PlotlyRenderer
from minilink.graphical.animation.renderers.pygame_renderer import PygameRenderer
from minilink.graphical.animation.renderers.renderer import AnimationRenderer
from minilink.graphical.animation.renderers.timing import (
    sim_index_for_frame,
    trajectory_frame_schedule,
)
from minilink.graphical.animation.visualization import (
    ensure_world_frame,
    flatten_draw_list,
)
from minilink.graphical.common.environment import prefers_inline_animation

__all__ = [
    "Animator",
    "AnimationRenderer",
    "MatplotlibRenderer",
    "MeshcatRenderer",
    "PlotlyRenderer",
    "PygameRenderer",
    "make_renderer",
]


def make_renderer(name: str, animator: "Animator") -> AnimationRenderer:
    """
    Return a backend instance for *name*.

    To add a backend, implement :class:`~minilink.graphical.animation.renderers.renderer.AnimationRenderer`
    and extend this function.
    """
    key = name.strip().lower()
    if key == "matplotlib":
        return MatplotlibRenderer(animator)
    if key == "meshcat":
        return MeshcatRenderer(animator)
    if key == "plotly":
        return PlotlyRenderer(animator)
    if key == "pygame":
        return PygameRenderer(animator)
    raise ValueError(
        "Unknown renderer "
        f"{name!r}. Expected 'matplotlib', 'meshcat', 'plotly', or 'pygame'."
    )


class Animator:
    """Playback coordinator: frame-keyed hooks → flat draw list → renderer.

    Owns the simulated system and delegates drawing to an
    :class:`AnimationRenderer`. Each frame resolves the system's named frame
    poses (``tf``), the cached kinematic geometry
    (``get_kinematic_geometry``), and the per-frame dynamic geometry
    (``get_dynamic_geometry``) into a flat ``(primitive, world 4x4)`` draw list.
    Matplotlib figure size and resolution live in
    :mod:`minilink.graphical.common.matplotlib_style`.
    """

    def __init__(self, sys):
        self.sys = sys
        self._overlay_kinematics = {}

    @staticmethod
    def _coerce_overlays(overlays):
        if overlays is None:
            return ()
        if isinstance(overlays, (list, tuple)):
            return tuple(overlays)
        return (overlays,)

    def _resolve_overlay_draw_list(self, overlay, t):
        validate_overlay(overlay)
        cache_key = id(overlay)
        if cache_key not in self._overlay_kinematics:
            self._overlay_kinematics[cache_key] = overlay.get_kinematic_geometry()
        kinematic = self._overlay_kinematics[cache_key]
        frames = overlay.tf(t)
        dynamic = overlay.get_dynamic_geometry(t)
        draw_list = flatten_draw_list(frames, kinematic, dynamic)
        if draw_list:
            return (list(part) for part in zip(*draw_list))
        return [], []

    def _merge_overlays(self, frame, overlays, t):
        overlays = self._coerce_overlays(overlays)
        if not overlays:
            return frame
        primitives = list(frame["primitives"])
        transforms = list(frame["transforms"])
        for overlay in overlays:
            o_primitives, o_transforms = self._resolve_overlay_draw_list(overlay, t)
            primitives.extend(o_primitives)
            transforms.extend(o_transforms)
        return {
            **frame,
            "primitives": primitives,
            "transforms": transforms,
        }

    def _resolve_frame(self, x, u, t, *, kinematic, camera_override=None, overlays=()):
        """Resolve one frame to a per-frame ``(primitives, transforms, camera)`` dict."""
        frames = ensure_world_frame(self.sys.tf(x, u, t))
        dynamic = self.sys.get_dynamic_geometry(x, u, t)
        draw_list = flatten_draw_list(frames, kinematic, dynamic)
        if draw_list:
            primitives, transforms = (list(part) for part in zip(*draw_list))
        else:
            primitives, transforms = [], []
        camera = resolve_camera_from_hints(
            self.sys, frames, x, u, t, override=camera_override
        )
        frame = {
            "x": x,
            "u": u,
            "t": float(t),
            "primitives": primitives,
            "transforms": transforms,
            "camera": camera,
        }
        return self._merge_overlays(frame, overlays, t)

    def show(
        self,
        x,
        u,
        t=0.0,
        is_3d=False,
        renderer="matplotlib",
        camera=None,
        overlays=None,
    ):
        """Render a single static frame at state *x*, *u*, *t*."""
        backend = make_renderer(renderer, self)
        kinematic = self.sys.get_kinematic_geometry()
        frame = self._resolve_frame(
            x,
            u,
            t,
            kinematic=kinematic,
            camera_override=camera,
            overlays=overlays,
        )
        backend.open_scene(
            is_3d=is_3d,
            show=True,
            camera=frame["camera"],
            title=f"{self.sys.name} — t = {t:.2f} s",
        )
        backend.draw_frame(frame["primitives"], frame["transforms"], t, frame["camera"])
        result = backend.present(block=True)
        backend.close_scene()
        return result

    def _build_frames(self, traj, schedule, *, kinematic, camera_override, overlays=()):
        frames = []
        for frame_idx in range(schedule.n_frames):
            sim_idx = sim_index_for_frame(frame_idx, schedule)
            # Convert at the graphics boundary (JAX TrajOpt trajectories stay JAX until here).
            x = np.asarray(traj.x[:, sim_idx], dtype=float)
            u = (
                np.asarray(traj.u[:, sim_idx], dtype=float)
                if len(traj.u) > 0
                else np.array([])
            )
            t = float(np.asarray(traj.t[sim_idx]))
            frames.append(
                self._resolve_frame(
                    x,
                    u,
                    t,
                    kinematic=kinematic,
                    camera_override=camera_override,
                    overlays=overlays,
                )
            )
        return frames

    def animate_simulation(
        self,
        traj,
        time_factor_video=1.0,
        is_3d=False,
        save=False,
        file_name="Animation",
        show=True,
        html: bool | None = None,
        renderer="matplotlib",
        native: bool = True,
        scene_title: str | None = None,
        camera=None,
        overlays=None,
    ):
        """
        Plays back a full simulation trajectory.

        The three orthogonal kwargs are:

        - ``renderer``  : graphics tech
          (``"matplotlib"``, ``"meshcat"``, ``"plotly"``, ``"pygame"``).
        - ``html``      : output channel. ``None`` auto-resolves via
          :func:`minilink.graphical.common.environment.prefers_inline_animation`:
          ``True`` in Colab and in local Jupyter when the active matplotlib
          backend is non-interactive (``inline`` / ``agg``), ``False`` for
          bare script, IPython REPL, and Jupyter with an interactive backend
          (``qt`` / ``widget`` / ``macosx`` / ``tk`` / ``nbagg``). Explicit
          ``True``/``False`` is honored.
        - ``native``    : playback engine. ``True`` (default) drives the
          backend's own animation engine:
          ``matplotlib.animation.FuncAnimation`` for matplotlib,
          ``meshcat.animation.Animation`` + ``set_animation`` for meshcat,
          and browser-side Plotly frames for plotly.
          ``False`` falls back to the per-frame Python loop (handy for
          debugging or when the native path's limitations matter).

        ``camera`` accepts the optional override: a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable.

        Notes
        -----
        Meshcat native animation only keyframes rigid pose (position+quaternion).
        Per-frame dynamic geometry (e.g. ``Arrow`` length/direction,
        ``TorqueArrow`` sweep) is frozen at ``t=0`` in the native path.

        Plotly does not support the per-frame Python loop (``native=False`` with
        ``html=False``): use ``native=True`` or ``html=True`` for inline/browser
        playback.
        """
        if html is None:
            html = prefers_inline_animation()

        import sys

        if "google.colab" in sys.modules and renderer.strip().lower() == "meshcat":
            html = True

        renderer_key = renderer.strip().lower()
        if renderer_key == "plotly" and not native and show and not html:
            raise ValueError(
                "renderer='plotly' with native=False is not supported when html=False "
                "(Plotly has no working per-frame redraw loop). "
                "Use native=True, or use html=True for inline framed playback."
            )

        backend = make_renderer(renderer, self)
        kinematic = self.sys.get_kinematic_geometry()
        schedule = trajectory_frame_schedule(traj, time_factor_video)
        frames = self._build_frames(
            traj,
            schedule,
            kinematic=kinematic,
            camera_override=camera,
            overlays=overlays,
        )
        # First frame's primitives as a representative list for APIs that need one.
        primitives = frames[0]["primitives"] if frames else []

        if html:
            try:
                return backend.render_inline_animation(
                    primitives, frames, schedule, is_3d=is_3d
                )
            except NotImplementedError:
                warnings.warn(
                    f"html=True is not supported for renderer={renderer!r}; "
                    "ignoring html.",
                    stacklevel=2,
                )

        if save:
            try:
                backend.export_animation(primitives, frames, schedule, file_name)
            except NotImplementedError:
                warnings.warn(
                    f"save=True is not supported for renderer={renderer!r}; "
                    "skipping export.",
                    stacklevel=2,
                )

        if not show:
            return None

        if native:
            try:
                return backend.play_native(
                    primitives,
                    frames,
                    schedule,
                    is_3d=is_3d,
                    scene_title=scene_title,
                )
            except NotImplementedError:
                warnings.warn(
                    f"native=True is not supported for renderer={renderer!r}; "
                    "falling back to the Python-loop path.",
                    stacklevel=2,
                )

        backend.open_scene(
            is_3d=is_3d,
            show=show,
            camera=frames[0]["camera"],
            title=scene_title or f"Animation: {self.sys.name}",
        )
        for frame in frames:
            backend.draw_frame(
                frame["primitives"], frame["transforms"], frame["t"], frame["camera"]
            )
            backend.present(block=False, interval_s=schedule.interval_ms / 1000.0)
            events = backend.poll_events()
            if events.get("quit", False):
                break
        backend.close_scene()
        return None

    # Live-session API (driven by RealtimeSimulator)

    def open_live_scene(
        self, backend, x, u, t, *, is_3d=False, kinematic=None, title=None
    ):
        """Open *backend* and draw the initial live frame; return the frame dict.

        Pacing (frame-rate sleep) belongs to the live orchestrator, so the
        frame is presented non-blocking with no interval.
        """
        if kinematic is None:
            kinematic = self.sys.get_kinematic_geometry()
        frame = self._resolve_frame(x, u, t, kinematic=kinematic)
        backend.open_scene(
            is_3d=is_3d,
            show=True,
            camera=frame["camera"],
            title=title or f"Live: {self.sys.name}",
        )
        backend.draw_frame(
            frame["primitives"], frame["transforms"], frame["t"], frame["camera"]
        )
        backend.present(block=False, interval_s=None)
        return frame

    def update_live_frame(self, backend, x, u, t, *, kinematic=None):
        """Draw one live frame on an already-open *backend*; return backend events.

        Dynamic geometry (force/torque arrows) is rebuilt from the frame-keyed
        hooks each call, so live sessions render the same visuals as playback.
        """
        if kinematic is None:
            kinematic = self.sys.get_kinematic_geometry()
        frame = self._resolve_frame(x, u, t, kinematic=kinematic)
        backend.draw_frame(
            frame["primitives"], frame["transforms"], frame["t"], frame["camera"]
        )
        backend.present(block=False, interval_s=None)
        return backend.poll_events()
