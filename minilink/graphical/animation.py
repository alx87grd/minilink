"""
Trajectory animation orchestration.

Backends live under :mod:`minilink.graphical.renderers`; the animator picks one by name
(see :func:`_make_renderer`).
"""

from __future__ import annotations

from minilink.graphical.renderers.base import AnimationRenderer
from minilink.graphical.renderers.matplotlib_renderer import MatplotlibRenderer
from minilink.graphical.renderers.meshcat_renderer import MeshcatRenderer
from minilink.graphical.renderers.pygame_renderer import PygameRenderer

__all__ = [
    "Animator",
    "AnimationRenderer",
    "MatplotlibRenderer",
    "MeshcatRenderer",
    "PygameRenderer",
]


def _make_renderer(name: str, animator: "Animator") -> AnimationRenderer:
    """
    Return a backend instance for *name* (``matplotlib``, ``meshcat``, ``pygame``).

    To add a backend, implement :class:`~minilink.graphical.renderers.base.AnimationRenderer`
    and extend this function.
    """
    key = name.strip().lower()
    if key == "matplotlib":
        return MatplotlibRenderer(animator)
    if key == "meshcat":
        return MeshcatRenderer(animator)
    if key == "pygame":
        return PygameRenderer(animator)
    raise ValueError(
        f"Unknown renderer {name!r}. Expected 'matplotlib', 'meshcat', or 'pygame'."
    )


class Animator:
    """
    Coordinates playback: owns display settings and the simulated system,
    and delegates drawing to an :class:`AnimationRenderer`.
    """

    def __init__(self, sys):
        self.sys = sys

        # Display settings (used by backends such as matplotlib)
        self.figsize = (8, 6)
        self.dpi = 100
        self.domain = [[-10, 10], [-10, 10], [-10, 10]]

    def show(self, x, u, t=0.0, is_3d=False, renderer="matplotlib"):
        """Renders a single static frame of the system at state *x*, *u*, *t*."""
        backend = _make_renderer(renderer, self)
        backend.render_static(x, u, t, is_3d)

    def animate_simulation(
        self,
        traj,
        time_factor_video=1.0,
        is_3d=False,
        save=False,
        file_name="Animation",
        show=True,
        html=False,
        renderer="matplotlib",
    ):
        """Plays back a full simulation trajectory."""
        backend = _make_renderer(renderer, self)
        return backend.render_animation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            save=save,
            file_name=file_name,
            show=show,
            html=html,
        )
