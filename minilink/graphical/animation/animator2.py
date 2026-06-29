"""v2 trajectory animation orchestration (parallel to :mod:`animator`).

``Animator2`` is the v2 **orchestrator**: it drives the frame-keyed drawable
contract (``tf_v2`` / ``get_kinematic_geometry_v2`` / ``get_dynamic_geometry_v2``)
through the **existing** renderers, reusing :func:`make_renderer` and every
backend untouched. Per frame it:

1. resolves the primary drawable's hooks — ``frames = sys.tf_v2(x, u, t)``,
   ``kinematic = sys.get_kinematic_geometry_v2()`` (cached once), and
   ``dynamic = sys.get_dynamic_geometry_v2(x, u, t)`` (rebuilt each frame);
2. flattens them with :func:`flatten_draw_list` into the flat
   ``(primitive, world 4x4)`` list the renderers consume, carried **per frame**
   so reviving dynamic geometry needs no further frame-model rework;
3. resolves the camera with :func:`resolve_camera_from_hints` (honoring an
   ``animate(camera=…)`` override).

This phase wires a **single primary** drawable (no overlays). Overlays and
multi-source camera selection are added later; the old :class:`Animator` and the
old hooks are left completely untouched.
"""

from __future__ import annotations

import warnings

import numpy as np

from minilink.graphical.animation.animator import make_renderer
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.renderers.timing import (
    sim_index_for_frame,
    trajectory_frame_schedule,
)
from minilink.graphical.animation.visualization import flatten_draw_list
from minilink.graphical.common.environment import prefers_inline_animation


class Animator2:
    """v2 playback coordinator: frame-keyed hooks → flat draw list → renderer."""

    def __init__(self, sys):
        self.sys = sys

    def _resolve_frame(self, x, u, t, *, kinematic, camera_override=None):
        """Resolve one frame to a per-frame ``(primitives, transforms, camera)`` dict."""
        frames = self.sys.tf_v2(x, u, t)
        dynamic = self.sys.get_dynamic_geometry_v2(x, u, t)
        draw_list = flatten_draw_list(frames, kinematic, dynamic)
        if draw_list:
            primitives, transforms = (list(part) for part in zip(*draw_list))
        else:
            primitives, transforms = [], []
        camera = resolve_camera_from_hints(
            self.sys, frames, x, u, t, override=camera_override
        )
        return {
            "x": x,
            "u": u,
            "t": float(t),
            "primitives": primitives,
            "transforms": transforms,
            "camera": camera,
        }

    def show(self, x, u, t=0.0, is_3d=False, renderer="matplotlib", camera=None):
        """Render a single static v2 frame at state *x*, *u*, *t*."""
        backend = make_renderer(renderer, self)
        kinematic = self.sys.get_kinematic_geometry_v2()
        frame = self._resolve_frame(
            x, u, t, kinematic=kinematic, camera_override=camera
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

    def _build_frames(self, traj, schedule, *, kinematic, camera_override):
        frames = []
        for frame_idx in range(schedule.n_frames):
            sim_idx = sim_index_for_frame(frame_idx, schedule)
            x = traj.x[:, sim_idx]
            u = traj.u[:, sim_idx] if len(traj.u) > 0 else np.array([])
            t = traj.t[sim_idx]
            frames.append(
                self._resolve_frame(
                    x, u, t, kinematic=kinematic, camera_override=camera_override
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
    ):
        """Play back a trajectory through the v2 pipeline (single primary drawable).

        Mirrors :meth:`Animator.animate_simulation`; the difference is the
        frame-keyed hooks and the **per-frame** draw list (kinematic cached once,
        dynamic rebuilt each frame). ``camera`` accepts the Layer-3 override (a
        constant 4x4 or a ``camera(frames, x, u, t)`` callable).
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
        kinematic = self.sys.get_kinematic_geometry_v2()
        schedule = trajectory_frame_schedule(traj, time_factor_video)
        frames = self._build_frames(
            traj, schedule, kinematic=kinematic, camera_override=camera
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
