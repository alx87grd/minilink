"""Animation: the animator, its renderers, and the graphical primitives they draw."""

# The names below are re-exported lazily, on purpose: the core kinematic hooks import
# minilink.graphical.animation.primitives, which must stay importable without
# matplotlib, and an eager import of the animator here would pull it in.
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
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation import animator

        return getattr(animator, name)

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
