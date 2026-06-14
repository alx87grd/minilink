"""Animation, rendering, and graphical primitive API.

The names below are re-exported lazily via ``__getattr__`` on purpose:
:mod:`~minilink.graphical.animation.primitives` must stay importable without
pulling in matplotlib (the core kinematic hooks import it), and importing
:mod:`~minilink.graphical.animation.animator` eagerly here would do exactly
that.
"""

__all__ = [
    "Animator",
    "AnimationRenderer",
    "MatplotlibRenderer",
    "MeshcatRenderer",
    "PlotlyRenderer",
    "PygameRenderer",
    "make_renderer",
]


def __getattr__(name):
    if name in __all__:
        from minilink.graphical.animation import animator

        return getattr(animator, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
