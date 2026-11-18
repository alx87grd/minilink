"""The record every plot returns: the drawn figure, its axes and its description."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from minilink.graphical.common.show import show_figure

###############################################################################
# Public API
###############################################################################


@dataclass
class PlotResult:
    """
    A drawn figure: ``figure`` (matplotlib's or Plotly's figure, Graphviz's
    graph), ``axes`` (matplotlib's ``Axes`` of a one-panel figure, a list in
    panel order for several; a ``NoAxes`` on a figure matplotlib did not
    draw) and ``spec``, the description this plot was drawn from.

    ``renderer=None`` holds the description alone, drawn by no renderer.
    """

    renderer: str | None
    figure: Any = None
    axes: Any = None
    spec: Any = None

    def show(self):
        """Show the figure where the environment puts it, as ``show=True`` does."""
        require_drawn(self)
        show_figure(self.figure, self.renderer, True)

        return self

    def save(self, file_name):
        """
        Write the figure to ``file_name``, in the format of its extension.

        Matplotlib writes any format it knows (``.png``, ``.pdf``, ``.svg``,
        …), a ``.png`` without an extension; Plotly an interactive ``.html``
        page (the default), or an image (``.png``, ``.svg``, ``.pdf``) through
        kaleido; Graphviz the format of the extension, a ``.pdf`` without one.
        Returns the path written.
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


class NoAxes:
    """
    The ``axes`` of a record matplotlib did not draw: a Plotly figure, a
    Graphviz diagram, or a description alone. It holds nothing, and says so by
    name when it is read, indexed, iterated or passed as ``ax=``: overlays and
    axis adjustments are matplotlib's.
    """

    renderer = None

    def __init__(self, renderer):
        self.renderer = renderer

    def __str__(self):
        if self.renderer is None:
            return (
                "this record holds a description only and no axes; draw it with "
                "renderer='matplotlib' to overlay a plot or adjust its axes"
            )

        return (
            f"a {self.renderer} figure has no matplotlib axes; draw the plot with "
            "renderer='matplotlib' to overlay another or adjust its axes"
        )

    def __getattr__(self, name):
        # The protocols Python probes for (copying, pickling) get its plain answer
        if name.startswith("__"):
            raise AttributeError(name)
        raise AttributeError(str(self))

    def __getitem__(self, index):
        raise TypeError(str(self))

    def __iter__(self):
        raise TypeError(str(self))

    def __len__(self):
        raise TypeError(str(self))


###############################################################################
# Internal machinery
###############################################################################


def require_drawn(result):
    """Refuse showing or saving a record that holds no figure: a description only, or a diagram Graphviz did not draw."""
    if result.renderer is not None and result.figure is None:
        raise ValueError(
            f"this record holds no figure: the {result.renderer} drawing was skipped; "
            "there is nothing to show or save"
        )
    if result.renderer is None:
        raise ValueError(
            "this record holds a description only (renderer=None); draw it with "
            "minilink.graphical.draw(result.spec, renderer=...) first"
        )


def save_matplotlib(fig, path):
    """A matplotlib figure in the format of the path's extension, a PNG without one."""
    # Lazy import: pyplot loads with the first matplotlib figure, not with Plotly's
    import matplotlib.pyplot as plt

    target = path if path.suffix else path.with_suffix(".png")

    # TrueType (Type 42) fonts in PDF and PostScript files: their text stays editable
    with plt.rc_context({"pdf.fonttype": 42, "ps.fonttype": 42}):
        fig.savefig(target, bbox_inches="tight")

    return target


def save_plotly(fig, path):
    """A Plotly figure as an interactive page (``.html``, the default), else an image through kaleido."""
    target = path if path.suffix else path.with_suffix(".html")
    if target.suffix.lower() in (".html", ".htm"):
        fig.write_html(target)
    else:
        fig.write_image(target)

    return target


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
