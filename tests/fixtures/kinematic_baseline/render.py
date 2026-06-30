"""Headless kinematic frame rendering for catalog render-smoke tests."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

BASELINE_DPI = 100


def _reset_matplotlib_state() -> None:
    """Return matplotlib to defaults so renders stay stable across pytest order."""
    import matplotlib.pyplot as plt

    plt.close("all")
    matplotlib.rcdefaults()


def render_baseline_png(sys, x, u, t, path, *, dpi=BASELINE_DPI):
    """Render one static kinematic frame to ``path`` (Agg, fixed DPI).

    Uses the same draw path as :meth:`Animator.show` (frame-keyed geometry +
    ``tf`` + dynamic geometry), but saves the figure instead of presenting it.
    """
    _reset_matplotlib_state()
    from minilink.graphical.animation.animator import Animator
    from minilink.graphical.animation.renderers.matplotlib_renderer import (
        MatplotlibRenderer,
    )

    animator = Animator(sys)
    renderer = MatplotlibRenderer(animator)
    kinematic = sys.get_kinematic_geometry()
    frame = animator._resolve_frame(x, u, t, kinematic=kinematic)

    renderer.open_scene(is_3d=False, show=False, camera=frame["camera"], title=sys.name)
    renderer.draw_frame(frame["primitives"], frame["transforms"], t, frame["camera"])
    renderer.fig.savefig(path, dpi=dpi)
    renderer.close_scene()
