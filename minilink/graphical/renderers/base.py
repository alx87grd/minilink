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
    def render_static(self, x, u, t: float, is_3d: bool) -> None:
        """Show a single frame for state *x*, input *u*, time *t*."""

    @abstractmethod
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
    ) -> Any:
        """Play or export a trajectory. Return value is backend-specific (e.g. matplotlib animation)."""
