"""The record every plot returns: the drawn figure, its axes and its description."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

###############################################################################
# Public API
###############################################################################


@dataclass
class PlotResult:
    """
    A drawn figure: ``figure`` (matplotlib's or Plotly's figure, Graphviz's
    graph), ``axes`` (matplotlib's ``Axes`` of a one-panel figure, a list in
    panel order for several; ``None`` on the other renderers) and ``spec``,
    the description it was drawn from.

    ``renderer=None`` holds the description alone, drawn by no renderer.
    """

    renderer: str | None
    figure: Any = None
    axes: Any = None
    spec: Any = None

    def show(self):
        """Show the figure where the environment puts it, as ``show=True`` does."""
        # Lazy import: the show step loads with the first shown figure
        from minilink.graphical.common.show import show_figure

        require_drawn(self)
        show_figure(self.figure, self.renderer, True)

        return self

    def save(self, file_name):
        """
        Write the figure to ``file_name``, in the format of its extension.

        Matplotlib writes any format it knows (``.png``, ``.pdf``, ``.svg``,
        …); Plotly an interactive ``.html`` page, or an image (``.png``,
        ``.svg``, ``.pdf``) through kaleido; Graphviz the format of the
        extension.
        """
        require_drawn(self)

        return SAVE[self.renderer](self.figure, Path(file_name))

    # Display

    def _ipython_display_(self):
        """Display nothing in a notebook: the plot call has already drawn the figure.

        A bare ``plant.plot_trajectory()`` as a cell's last line then shows the
        figure alone, without this record's text under it. The figure stays on
        ``result.figure``.
        """


def plot_result_of(figures):
    """A matplotlib ``(fig, axes)`` as a :class:`PlotResult`; a record passes as it is, a list item by item."""
    if isinstance(figures, list):
        return [plot_result_of(item) for item in figures]
    if isinstance(figures, PlotResult):
        return figures
    fig, axes = figures

    return PlotResult(renderer="matplotlib", figure=fig, axes=axes)


def require_matplotlib(renderer):
    """Refuse a drawing library a plot does not draw with: Matplotlib only."""
    if renderer != "matplotlib":
        raise ValueError(
            f"this plot draws with renderer='matplotlib' only, got {renderer!r}"
        )


###############################################################################
# Internal machinery
###############################################################################


def require_drawn(result):
    """Refuse showing or saving a record that holds a description only."""
    if result.renderer is None or result.figure is None:
        raise ValueError(
            "this record holds a description only (renderer=None); draw it with "
            "minilink.graphical.draw(result.spec, renderer=...) first"
        )


def save_matplotlib(fig, path):
    """A matplotlib figure in the format of the path's extension."""
    fig.savefig(path, bbox_inches="tight")

    return path


def save_plotly(fig, path):
    """A Plotly figure as an interactive page (``.html``), else an image through kaleido."""
    if path.suffix.lower() in (".html", ".htm"):
        fig.write_html(path)
    else:
        fig.write_image(path)

    return path


def save_graphviz(graph, path):
    """A Graphviz graph in the format of the path's extension, a PDF without one."""
    form = path.suffix.lstrip(".").lower() or "pdf"
    target = path if path.suffix else path.with_suffix(".pdf")
    target.write_bytes(graph.pipe(format=form))

    return target


# One writer per renderer
SAVE = {
    "matplotlib": save_matplotlib,
    "plotly": save_plotly,
    "graphviz": save_graphviz,
}
