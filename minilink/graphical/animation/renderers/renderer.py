"""Abstract animation backend."""

from abc import ABC, abstractmethod


class AnimationRenderer(ABC):
    """
    One backend (matplotlib, meshcat, pygame, …) for static views and
    trajectory playback. Subclasses own all library-specific setup and I/O.
    """

    supports_interactive = True

    def __init__(self, animator):
        self.animator = animator
        self.sys = animator.sys

    @abstractmethod
    def open_scene(
        self,
        *,
        is_3d: bool,
        show: bool,
        camera,
        title: str | None = None,
    ) -> None:
        """Initialize backend resources for one render session.

        ``camera`` is a 4x4 :func:`~minilink.graphical.animation.primitives.camera_matrix`
        used to set up initial framing. Renderers consume the slots they
        understand (target, projection-axis columns, view scale, view-out
        direction) and ignore the rest; see ``DESIGN.md`` §4.7.
        """

    @abstractmethod
    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        """Draw one frame from precomputed transforms and camera."""

    @abstractmethod
    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        """Present the currently drawn frame and optionally block/sleep."""

    def poll_events(self):
        """Return backend events/state for interactive loops.

        ROADMAP: live **external input** (beyond quit keys) and optional **live output push**
        for cosimulation belong in dedicated I/O abstractions; renderers stay draw/present-only
        where possible—see ``ROADMAP.md`` §7.
        """
        return {}

    @abstractmethod
    def close_scene(self) -> None:
        """Release/close backend resources for the current session."""

    def render_inline_animation(self, primitives, frames, schedule, *, is_3d: bool):
        """Optional notebook inline output (default: unsupported).

        ``frames`` entries each carry ``"transforms"`` and ``"camera"``
        (see :meth:`minilink.graphical.animation.Animator._prepare_transforms`).
        """
        raise NotImplementedError("Inline animation is not supported by this renderer.")

    def export_animation(self, primitives, frames, schedule, file_name: str) -> None:
        """Optional animation export (default: unsupported)."""
        raise NotImplementedError("Animation export is not supported by this renderer.")

    def play_native(
        self,
        primitives,
        frames,
        schedule,
        *,
        is_3d: bool,
        scene_title: str | None = None,
    ):
        """
        Optional native playback using the backend's own animation engine.

        Subclasses that support a native animation API (matplotlib
        ``FuncAnimation``, meshcat ``Animation`` + ``set_animation``, ...)
        override this hook. Default raises ``NotImplementedError`` so callers
        can fall back to the generic per-frame loop.
        """
        raise NotImplementedError("Native playback is not supported by this renderer.")
