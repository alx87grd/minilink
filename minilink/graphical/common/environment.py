"""
Runtime environment detection for :mod:`minilink.graphical`.

Provides a single cached ``detect_env()`` and derived policy predicates so
every env-sensitive site (animation HTML default, figure blocking, meshcat
prompt, stacked plot height) reads from the same source of truth.
"""

import sys

__all__ = [
    "EnvName",
    "detect_env",
    "prefers_inline_animation",
    "is_inline_capable",
    "is_blocking_needed",
    "allow_tall_stacked_figures",
    "override_env",
]

EnvName = str

_CACHE = None


def detect_env():
    """
    Return one of ``"script"``, ``"ipython"``, ``"jupyter"``, ``"colab"``.

    The result is cached on first call. Use :func:`override_env` in tests
    to force a value.
    """
    global _CACHE
    if _CACHE is not None:
        return _CACHE
    if "google.colab" in sys.modules:
        _CACHE = "colab"
        return _CACHE
    try:
        from IPython import get_ipython

        ip = get_ipython()
    except ImportError:
        ip = None
    if ip is None:
        _CACHE = "script"
    elif type(ip).__name__ == "ZMQInteractiveShell":
        _CACHE = "jupyter"
    else:
        _CACHE = "ipython"
    return _CACHE


def _mpl_backend_is_non_interactive() -> bool:
    """
    True when the currently active matplotlib backend cannot pop up a window.

    Covers the default Jupyter/Colab ``inline`` backend and the bare ``agg``
    backend. Interactive backends (``qt*``, ``macosx``, ``tkagg``, ``ipympl`` /
    ``widget``, ``nbagg``) return False.
    """
    import matplotlib

    backend = matplotlib.get_backend().lower()
    return ("inline" in backend) or (backend == "agg")


def prefers_inline_animation() -> bool:
    """
    Default value for ``html=None`` in animation APIs.

    Returns True in Colab (always) and in Jupyter when the active matplotlib
    backend cannot physically pop up a window. Returns False for script,
    IPython REPL, and Jupyter with an interactive backend (qt, widget/ipympl,
    macosx, tk, nbagg).
    """
    env = detect_env()
    if env == "colab":
        return True
    if env == "jupyter" and _mpl_backend_is_non_interactive():
        return True
    return False


def is_inline_capable() -> bool:
    """
    True when the env can render ``IPython.display`` objects (if explicitly asked).

    Used to decide whether honoring an explicit ``html=True`` will actually
    display something to the user.
    """
    return detect_env() in ("jupyter", "colab")


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


if __name__ == "__main__":
    print("aaa")
    print(is_blocking_needed())
