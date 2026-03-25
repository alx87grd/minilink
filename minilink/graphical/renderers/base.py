"""Abstract animation backend."""

from __future__ import annotations

from abc import ABC, abstractmethod

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from minilink.graphical.animation import Animator


class AnimationRenderer(ABC):
    """
    One backend (matplotlib, meshcat, pygame, …) for static views and
    trajectory playback. Subclasses own all library-specific setup and I/O.
    """

    def __init__(self, animator: Animator):
        self.animator = animator
        self.sys = animator.sys

    @abstractmethod
    def open_scene(self, *, is_3d: bool, show: bool, title: str | None = None) -> None:
        """Initialize backend resources for one render session."""

    @abstractmethod
    def draw_frame(self, primitives, transforms, t: float) -> None:
        """Draw one frame from precomputed transforms."""

    @abstractmethod
    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        """Present the currently drawn frame and optionally block/sleep."""

    def poll_events(self) -> dict[str, Any]:
        """Return backend events/state for interactive loops."""
        return {}

    @abstractmethod
    def close_scene(self) -> None:
        """Release/close backend resources for the current session."""

    def render_inline_animation(self, primitives, frames, schedule) -> Any:
        """Optional notebook inline output (default: unsupported)."""
        raise NotImplementedError("Inline animation is not supported by this renderer.")

    def export_animation(self, primitives, frames, schedule, file_name: str) -> None:
        """Optional animation export (default: unsupported)."""
        raise NotImplementedError("Animation export is not supported by this renderer.")
