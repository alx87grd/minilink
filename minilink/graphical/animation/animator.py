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

Roadmap (not implemented here—see ``ROADMAP.md`` §7 and P2):

- **Interactive integrator backends**: :meth:`Animator.game` /
  :meth:`Animator.run_interactive` currently own a simple time loop (Euler +
  substeps in ``game``). These should gain a ``Simulator``-style pluggable
  **integration** layer instead of hard-coding one integrator in the animator.
- **Live I/O backends**: ``game`` reads ``u`` only via **pygame** today; future
  **input** backends (e.g. TCP for cosimulation) and optional **live output
  push** should sit beside that as swappable sources/sinks.
"""

from __future__ import annotations

import warnings

import numpy as np

from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.interactive import (
    draw_keyboard_input_overlay,
    u_from_keyboard,
)
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
from minilink.graphical.animation.visualization import flatten_draw_list
from minilink.graphical.animation.drawables import validate_overlay
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


def _require_interactive_renderer(name: str, renderer: AnimationRenderer) -> None:
    if not renderer.supports_interactive:
        raise ValueError(
            f"renderer={name!r} does not support "
            "interactive loops; use it with render() or animate() on a "
            "precomputed trajectory."
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

    def _resolve_frame(
        self, x, u, t, *, kinematic, camera_override=None, overlays=()
    ):
        """Resolve one frame to a per-frame ``(primitives, transforms, camera)`` dict."""
        frames = self.sys.tf(x, u, t)
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
            x = traj.x[:, sim_idx]
            u = traj.u[:, sim_idx] if len(traj.u) > 0 else np.array([])
            t = traj.t[sim_idx]
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
        Per-frame dynamic geometry (e.g. ``TorqueArrow`` sweep) is frozen at
        ``t=0`` in the native path.

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
        # Representative primitive list for renderers that read the fixed arg;
        # the matplotlib builder reads each frame's own ``"primitives"``.
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

    def run_interactive(
        self,
        update_callback,
        *,
        x0,
        u0=None,
        t0=0.0,
        dt=1 / 30.0,
        renderer="pygame",
        is_3d=False,
        show=True,
        max_steps=None,
    ):
        """
        Prototype callback-driven interactive loop.
        callback signature:
            new_x, new_u, should_stop = update_callback(x, u, t, step_idx, events)
        """
        backend = make_renderer(renderer, self)
        _require_interactive_renderer(renderer, backend)
        kinematic = self.sys.get_kinematic_geometry()

        x = np.asarray(x0)
        u = np.asarray([] if u0 is None else u0)
        t = float(t0)
        step_idx = 0

        frame = self._resolve_frame(x, u, t, kinematic=kinematic)
        backend.open_scene(
            is_3d=is_3d,
            show=show,
            camera=frame["camera"],
            title=f"Interactive: {self.sys.name}",
        )

        # Draw initial state once so the callback can just update controls.
        backend.draw_frame(frame["primitives"], frame["transforms"], t, frame["camera"])
        backend.present(block=False, interval_s=dt)

        # ROADMAP: this loop is a minimal integrator+render tick; a future backend should
        # own step integration (multiple schemes) like Simulator, not only this while-body.
        while True:
            events = backend.poll_events()
            if events.get("quit", False):
                break

            new_x, new_u, should_stop = update_callback(x, u, t, step_idx, events)
            x = np.asarray(new_x)
            u = np.asarray(new_u)
            t += dt
            step_idx += 1

            frame = self._resolve_frame(x, u, t, kinematic=kinematic)
            backend.draw_frame(
                frame["primitives"], frame["transforms"], t, frame["camera"]
            )
            backend.present(block=False, interval_s=dt)

            if should_stop:
                break
            if max_steps is not None and step_idx >= max_steps:
                break
        backend.close_scene()

    def game(
        self,
        *,
        dt=1 / 100.0,
        dynamics_substeps=1000,
        renderer="pygame",
        is_3d=False,
        x0=None,
        u0=None,
        t0=0.0,
        max_steps=None,
    ):
        """
        Prototype real-time interactive mode.

        - Poll keyboard using pygame (input only).
        - Compute dynamics using Euler integration optionally with multiple
          internal substeps per rendered frame.
        - Update the visualization by redrawing each tick; dynamic geometry
          (force/torque arrows) is rebuilt live from the frame-keyed hooks.

        Roadmap: **input** should become pluggable backends (keyboard vs TCP
        cosimulation, etc.); **integration** should be pluggable backends (not
        Euler-only here); optional **live output push** is a later sibling—see
        ``ROADMAP.md`` §7.
        """
        if dynamics_substeps < 1 or int(dynamics_substeps) != dynamics_substeps:
            raise ValueError("dynamics_substeps must be a positive integer.")
        dynamics_substeps = int(dynamics_substeps)
        backend = make_renderer(renderer, self)
        _require_interactive_renderer(renderer, backend)

        # Lazy pygame import: game() should be optional.
        try:
            import pygame
        except ImportError as e:
            raise ImportError(
                "pygame is required for renderer/input mode. Install with: "
                "pip install 'minilink[visualization]'"
            ) from e

        pygame.init()
        vis = renderer.strip().lower()
        # When visualization is not pygame, use a visible window so SDL can
        # deliver keyboard events (1x1 is not focusable on many platforms).
        keyboard_input_screen = None
        if vis != "pygame":
            keyboard_input_screen = pygame.display.set_mode((640, 240))
            pygame.display.set_caption(
                f"minilink game — keyboard (focus here) — {self.sys.name}"
            )

        kinematic = self.sys.get_kinematic_geometry()

        x = np.asarray(self.sys.x0 if x0 is None else x0, dtype=float).copy()
        u = np.asarray(np.zeros(self.sys.m) if u0 is None else u0, dtype=float).copy()
        t = float(t0)
        step_idx = 0

        frame = self._resolve_frame(x, u, t, kinematic=kinematic)
        backend.open_scene(
            is_3d=is_3d,
            show=True,
            camera=frame["camera"],
            title=f"Interactive game — {self.sys.name}",
        )

        # Initial input from keyboard (pygame display exists for renderer=pygame).
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                backend.close_scene()
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                backend.close_scene()
                pygame.quit()
                return

        keys = pygame.key.get_pressed()
        u = u_from_keyboard(keys, m=self.sys.m, pygame=pygame)
        draw_keyboard_input_overlay(keyboard_input_screen, pygame, u)

        frame = self._resolve_frame(x, u, t, kinematic=kinematic)
        backend.draw_frame(frame["primitives"], frame["transforms"], t, frame["camera"])
        backend.present(block=False, interval_s=dt)

        while True:
            should_quit = False

            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    should_quit = True
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    should_quit = True

            keys = pygame.key.get_pressed()
            u = u_from_keyboard(keys, m=self.sys.m, pygame=pygame)
            draw_keyboard_input_overlay(keyboard_input_screen, pygame, u)

            # ROADMAP: Euler + substeps only—replace with interactive integrator
            # backends; live u should come from an input backend (keys vs TCP).
            # Euler steps for dynamic systems only (ZOH on u during frame).
            if self.sys.n > 0:
                dt_dyn = dt / dynamics_substeps
                t_dyn = t
                for _ in range(dynamics_substeps):
                    dx = self.sys.f(x, u, t_dyn)
                    x = x + np.asarray(dx, dtype=float) * dt_dyn
                    t_dyn += dt_dyn

            t = t + dt
            step_idx += 1

            frame = self._resolve_frame(x, u, t, kinematic=kinematic)
            backend.draw_frame(
                frame["primitives"], frame["transforms"], t, frame["camera"]
            )
            backend.present(block=False, interval_s=dt)

            backend_events = backend.poll_events()
            if backend_events.get("quit", False):
                break
            if should_quit:
                break
            if max_steps is not None and step_idx >= max_steps:
                break

        backend.close_scene()
        pygame.quit()
