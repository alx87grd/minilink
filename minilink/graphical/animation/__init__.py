"""Animation, rendering, and graphical primitive API."""

__all__ = [
    "Animator",
    "AnimationRenderer",
    "MatplotlibRenderer",
    "MeshcatRenderer",
    "PlotlyRenderer",
    "PygameRenderer",
    "_make_renderer",
]


def __getattr__(name):
    if name in __all__:
        from minilink.graphical.animation import animator

        return getattr(animator, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
