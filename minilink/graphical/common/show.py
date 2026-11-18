"""The one show step: a drawn figure displayed where the environment puts it."""

from minilink.graphical.common.environment import is_blocking_needed, resolve_show

###############################################################################
# Public API
###############################################################################


def show_figure(figure, renderer, show) -> None:
    """
    Show ``figure``, drawn with ``renderer``, where ``show`` resolves.

    Inline in a notebook it is displayed now, at the line that drew it, and a
    matplotlib figure is then closed so the cell does not show it again; in a
    window, matplotlib blocks in a bare script until the window closes.
    """
    place = resolve_show(show, renderer)
    if place is None or figure is None:
        return

    PRESENT[renderer](figure, place)


###############################################################################
# Internal machinery
###############################################################################


def present_matplotlib(fig, place):
    """A matplotlib figure inline (displayed, then closed) or in its window."""
    import matplotlib.pyplot as plt

    if place == "inline":
        display_now(fig)
        plt.close(fig)
        return

    plt.show(block=True if is_blocking_needed() else None)


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
    from IPython.display import display

    display(obj)


# One presenter per renderer
PRESENT = {
    "matplotlib": present_matplotlib,
    "plotly": present_plotly,
    "graphviz": present_graphviz,
}
