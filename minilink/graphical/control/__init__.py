"""Control-analysis figures: one spec, rendered with matplotlib or plotly."""

from minilink.graphical.control import style
from minilink.graphical.control.figure_spec import (
    ControlFigure,
    Note,
    Panel,
    RefLine,
    Trace,
)


def render_control_figure(spec: ControlFigure, *, backend="matplotlib", show=True):
    """Render ``spec`` with the selected backend and return a ``PlotResult``."""
    key = str(backend).strip().lower()
    if key in ("matplotlib", "mpl"):
        from minilink.graphical.control.matplotlib_backend import render_matplotlib

        return render_matplotlib(spec, show=show)
    if key == "plotly":
        from minilink.graphical.control.plotly_backend import render_plotly

        return render_plotly(spec, show=show)
    raise ValueError(
        f"Unknown plot backend {backend!r}. Expected 'matplotlib' or 'plotly'."
    )


__all__ = [
    "ControlFigure",
    "Note",
    "Panel",
    "RefLine",
    "Trace",
    "render_control_figure",
    "style",
]
