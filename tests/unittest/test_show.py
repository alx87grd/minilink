"""Where a figure goes: the show resolver's table, and the one show step."""

import pytest

from minilink.graphical.common import environment
from minilink.graphical.common.environment import override_env, resolve_show

RENDERERS = ("matplotlib", "plotly", "meshcat", "pygame", "graphviz")


@pytest.fixture
def place(monkeypatch):
    """Set the session (environment, matplotlib backend, display), then resolve."""

    def resolve(show, renderer, *, env, backend="agg", display=True, headless=False):
        override_env(env)
        monkeypatch.setattr(
            environment, "mpl_backend_is_inline", lambda: backend == "inline"
        )
        monkeypatch.setattr(
            environment,
            "mpl_backend_is_non_interactive",
            lambda: backend in ("inline", "agg"),
        )
        monkeypatch.setattr(environment, "has_display", lambda: display)
        monkeypatch.delenv("SDL_VIDEODRIVER", raising=False)
        if headless:
            monkeypatch.setenv("MINILINK_HEADLESS", "1")
        else:
            monkeypatch.delenv("MINILINK_HEADLESS", raising=False)
        return resolve_show(show, renderer)

    yield resolve
    override_env(None)


# show=True, a desktop session: a window everywhere, matplotlib only with a GUI backend
@pytest.mark.parametrize(
    "renderer, backend, expected",
    [
        ("matplotlib", "qtagg", "window"),
        ("matplotlib", "agg", None),
        ("plotly", "agg", "window"),
        ("meshcat", "agg", "window"),
        ("pygame", "agg", "window"),
        ("graphviz", "agg", "window"),
    ],
)
@pytest.mark.parametrize("env", ["script", "ipython"])
def test_a_script_shows_in_a_window(place, env, renderer, backend, expected):
    assert place(True, renderer, env=env, backend=backend) == expected


# show=True in Jupyter: inline, matplotlib only with the inline backend, pygame in its window
@pytest.mark.parametrize(
    "renderer, backend, expected",
    [
        ("matplotlib", "inline", "inline"),
        ("matplotlib", "widget", "window"),
        ("matplotlib", "agg", None),
        ("plotly", "agg", "inline"),
        ("meshcat", "agg", "inline"),
        ("graphviz", "agg", "inline"),
        ("pygame", "agg", "window"),
    ],
)
def test_a_notebook_shows_inline(place, renderer, backend, expected):
    assert place(True, renderer, env="jupyter", backend=backend) == expected


@pytest.mark.parametrize("renderer", ["matplotlib", "plotly", "meshcat", "graphviz"])
def test_colab_shows_inline(place, renderer):
    assert place(True, renderer, env="colab", backend="inline") == "inline"


def test_colab_refuses_pygame_by_name(place):
    with pytest.raises(ValueError, match="window"):
        place(True, "pygame", env="colab", display=False)


@pytest.mark.parametrize("renderer", ["plotly", "meshcat", "graphviz"])
def test_without_a_display_show_true_is_silent_and_a_window_is_refused(place, renderer):
    assert place(True, renderer, env="script", display=False) is None
    with pytest.raises(ValueError, match="show='window'"):
        place("window", renderer, env="script", display=False)


@pytest.mark.parametrize("renderer", RENDERERS)
def test_inline_in_a_script_is_refused_by_name(place, renderer):
    with pytest.raises(ValueError, match="show='inline'"):
        place("inline", renderer, env="script")


@pytest.mark.parametrize("show", [True, "inline", "window"])
@pytest.mark.parametrize("renderer", RENDERERS)
def test_headless_shows_nothing_whatever_is_asked(place, show, renderer):
    assert place(show, renderer, env="jupyter", backend="inline", headless=True) is None


@pytest.mark.parametrize("renderer", RENDERERS)
def test_show_false_is_nowhere(place, renderer):
    assert place(False, renderer, env="jupyter", backend="inline") is None


@pytest.mark.parametrize("show", ["pdf", 1, None, "html"])
def test_an_unknown_show_is_refused(place, show):
    with pytest.raises(ValueError, match="show must be"):
        place(show, "matplotlib", env="script")


def test_inline_matplotlib_is_displayed_once_and_closed(place, monkeypatch):
    import matplotlib.pyplot as plt

    from minilink.graphical.common import show as show_module

    shown = []
    monkeypatch.setattr(show_module, "display_now", shown.append)
    monkeypatch.setattr(
        show_module, "resolve_show", lambda show, renderer: "inline" if show else None
    )
    fig, _ = plt.subplots()

    show_module.show_figure(fig, "matplotlib", True)

    assert shown == [fig]
    assert fig.number not in plt.get_fignums()
