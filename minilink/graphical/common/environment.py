"""Runtime environment: script, IPython, Jupyter or Colab, and the figure policies each one sets."""

import os
import sys

__all__ = [
    "EnvName",
    "detect_env",
    "resolve_show",
    "is_headless",
    "is_blocking_needed",
    "allow_tall_stacked_figures",
    "override_env",
]

EnvName = str

# One cached detection, so every environment-sensitive site (the animation HTML
# default, figure blocking, the meshcat prompt, the stacked plot height) reads the
# same answer through the policy predicates below.
_CACHE = None


###############################################################################
# Public API
###############################################################################


def detect_env():
    """
    Return one of ``"script"``, ``"ipython"``, ``"jupyter"``, ``"colab"``.

    The result is cached on first call. Use :func:`override_env` in tests
    to force a value.
    """
    global _CACHE
    if _CACHE is not None:
        return _CACHE

    # Colab, recognized by its loaded module
    if "google.colab" in sys.modules:
        _CACHE = "colab"
        return _CACHE

    # The running IPython shell, if any: IPython is optional
    try:
        from IPython import get_ipython

        ip = get_ipython()
    except ImportError:
        ip = None

    # No shell is a script; a ZMQ shell is a notebook kernel; any other a terminal REPL
    if ip is None:
        _CACHE = "script"
    elif type(ip).__name__ == "ZMQInteractiveShell":
        _CACHE = "jupyter"
    else:
        _CACHE = "ipython"

    return _CACHE


def resolve_show(show, renderer):
    """
    Where a figure drawn with ``renderer`` goes: ``"inline"``, ``"window"`` or ``None``.

    ``show=True`` picks the place: inline in a notebook (matplotlib only with an
    inline backend), a window elsewhere. ``"inline"`` and ``"window"`` name it,
    and are refused by name where it cannot be. ``False`` is ``None``. A
    headless run (:func:`is_headless`) shows nothing.
    """
    require_show(show)
    if show is False or is_headless():
        return None

    env = detect_env()
    place = default_place(env, renderer) if show is True else show

    # Where nothing can be shown: silence for show=True, a refusal when named, and
    # for pygame, which only draws in a window of its own
    if not can_show(place, env, renderer):
        if show is True and renderer != "pygame":
            return None
        raise ValueError(cannot_show_message(place, env, renderer))

    return place


def is_headless() -> bool:
    """True when ``MINILINK_HEADLESS`` is set: no figure is shown, whatever ``show`` asks."""
    return os.environ.get(HEADLESS_VARIABLE, "") not in ("", "0")


def is_blocking_needed() -> bool:
    """
    True when the top-level process needs to block on GUI calls to stay alive.

    Only the bare script case; IPython REPL, Jupyter, and Colab all keep the
    interpreter alive independently of any GUI event loop.
    """
    return detect_env() == "script"


def allow_tall_stacked_figures() -> bool:
    """
    True when multi-row trajectory/signal plots can use a tall ``figsize``.

    In Jupyter and Colab, figures usually appear in scrollable cell output. In a
    bare script or terminal IPython session, figures typically open in a window
    that should stay within a reasonable screen height.
    """
    return detect_env() in ("jupyter", "colab")


def override_env(name) -> None:
    """
    Test/CI hook to force a specific env.

    Pass one of ``"script"``, ``"ipython"``, ``"jupyter"``, ``"colab"`` to
    override, or ``None`` to reset and re-detect on next call.
    """
    global _CACHE
    _CACHE = name


###############################################################################
# Internal machinery
###############################################################################


# The variable that silences every figure, for automated runs
HEADLESS_VARIABLE = "MINILINK_HEADLESS"

# The values show= takes
SHOWS = (True, False, "inline", "window")


def require_show(show):
    """Refuse a ``show`` that is not ``True``, ``False``, ``"inline"`` or ``"window"``."""
    if not any(
        show is value or (isinstance(show, str) and show == value) for value in SHOWS
    ):
        raise ValueError(
            f'show must be True, False, "inline" or "window", got {show!r}'
        )


def default_place(env, renderer):
    """The place ``show=True`` picks: inline in a notebook, a window elsewhere."""
    if renderer == "pygame":
        return "window"
    if env == "colab":
        return "inline"
    if env == "jupyter":
        if renderer == "matplotlib" and not mpl_backend_is_inline():
            return "window"
        return "inline"

    return "window"


def can_show(place, env, renderer) -> bool:
    """Whether ``place`` can show a figure drawn with ``renderer`` here."""
    if place == "inline":
        if env not in ("jupyter", "colab") or renderer == "pygame":
            return False
        return renderer != "matplotlib" or mpl_backend_is_inline()

    # A window: a display (pygame's off-screen driver counts), and for matplotlib
    # an interactive backend
    if renderer == "pygame" and os.environ.get("SDL_VIDEODRIVER") == "dummy":
        return True
    if env == "colab" or not has_display():
        return False
    if renderer == "matplotlib":
        return not mpl_backend_is_non_interactive()

    return True


def cannot_show_message(place, env, renderer) -> str:
    """Why ``place`` cannot show a ``renderer`` figure here, and what to pass instead."""
    if place == "inline":
        return (
            f"show='inline' needs a notebook with an inline display; this is a {env} "
            f"session for renderer={renderer!r}; pass show=True or show='window'"
        )

    return (
        f"show='window' needs a display and an interactive backend; this {env} "
        f"session has none for renderer={renderer!r}; pass show=True or show='inline'"
    )


def has_display() -> bool:
    """Whether a window can open: always on macOS and Windows, with ``DISPLAY`` or ``WAYLAND_DISPLAY`` on Linux."""
    if not sys.platform.startswith("linux"):
        return True

    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def mpl_backend_is_inline() -> bool:
    """True when matplotlib draws into the notebook's output (the ``inline`` backend)."""
    import matplotlib

    return "inline" in matplotlib.get_backend().lower()


def mpl_backend_is_non_interactive() -> bool:
    """
    True when the currently active matplotlib backend cannot pop up a window.

    Covers the notebook's ``inline`` backend and the file backends (``agg``,
    ``pdf``, ``svg``, …). Interactive backends (``qt*``, ``macosx``, ``tkagg``,
    ``ipympl`` / ``widget``, ``nbagg``) return False.
    """
    import matplotlib
    from matplotlib import rcsetup

    backend = matplotlib.get_backend().lower()
    file_backends = {name.lower() for name in rcsetup.non_interactive_bk}

    return "inline" in backend or backend in file_backends
