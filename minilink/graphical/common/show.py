"""The one show step: a drawn figure displayed where the environment puts it."""

from minilink.graphical.common.environment import (
    detect_env,
    is_blocking_needed,
    mpl_backend_is_inline,
    resolve_show,
)

###############################################################################
# Public API
###############################################################################


def show_figure(figure, renderer, show) -> None:
    """
    Show ``figure``, drawn with ``renderer``, where ``show`` resolves.

    Inline in a notebook it is displayed now, at the line that drew it, and a
    matplotlib figure is then closed so the cell does not show it again; in a
    window, matplotlib shows that figure alone and blocks in a bare script
    until it closes. A matplotlib figure not shown is closed in a notebook
    whose inline backend would show it at the cell's end; ``.show()`` on its
    record still displays it.
    """
    if figure is None:
        return

    place = resolve_show(show, renderer)
    if place is None:
        hold_back(figure, renderer)
        return

    PRESENT[renderer](figure, place)


def show_in_window(fig) -> None:
    """Show the matplotlib figure ``fig`` in its window, alone; a bare script blocks on it until it closes."""
    fig.show()
    if is_blocking_needed():
        fig.canvas.mpl_connect(
            "close_event", lambda event: fig.canvas.stop_event_loop()
        )
        fig.canvas.start_event_loop(0)


def hold_back(figure, renderer):
    """Close a matplotlib figure not shown, where the notebook's inline backend would show every open figure at the cell's end."""
    if renderer != "matplotlib" or detect_env() not in ("jupyter", "colab"):
        return
    if not mpl_backend_is_inline():
        return

    # Lazy import: pyplot loads with the first matplotlib figure, not with Plotly's
    import matplotlib.pyplot as plt

    plt.close(figure)


###############################################################################
# Internal machinery
###############################################################################


def present_matplotlib(fig, place):
    """A matplotlib figure inline (displayed, then closed) or in its window."""
    # Lazy import: pyplot loads with the first matplotlib figure, not with Plotly's
    import matplotlib.pyplot as plt

    if place == "inline":
        display_now(fig)
        plt.close(fig)
        return

    show_in_window(fig)


def present_plotly(fig, place):
    """A Plotly figure inline, or in a browser tab."""
    if place == "inline":
        display_now(fig)
        return

    fig.show(renderer="browser")


def present_graphviz(graph, place):
    """A Graphviz diagram inline (its SVG), or as a vector PDF in the system's viewer."""
    # Lazy import: the diagrams load with the first one shown
    from minilink.graphical.diagrams.dot import present_diagram

    present_diagram(graph, place)


def display_now(obj):
    """Display ``obj`` in the notebook's output, now."""
    # Lazy import: IPython is optional, needed only in a notebook
    from IPython.display import display

    display(obj)


# One presenter per renderer
PRESENT = {
    "matplotlib": present_matplotlib,
    "plotly": present_plotly,
    "graphviz": present_graphviz,
}
