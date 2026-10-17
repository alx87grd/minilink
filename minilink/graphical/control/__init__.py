"""Control-analysis figures: one spec, rendered with matplotlib or plotly."""

from minilink.graphical.control import style
from minilink.graphical.control.figure_spec import (
    ControlFigure,
    Note,
    Panel,
    RefLine,
    Trace,
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


###############################################################################
# Public API
###############################################################################


def render_control_figure(spec: ControlFigure, *, backend="matplotlib", show=True):
    """Render ``spec`` with the selected backend and return a ``PlotResult``."""
    key = str(backend).strip().lower()
    require_plot_backend(key, backend)

    if key in ("matplotlib", "mpl"):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.control.matplotlib_backend import render_matplotlib

        return render_matplotlib(spec, show=show)

    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.control.plotly_backend import render_plotly

    return render_plotly(spec, show=show)


###############################################################################
# Internal machinery
###############################################################################


def require_plot_backend(key, backend):
    """Refuse a backend other than matplotlib (or ``"mpl"``) and plotly."""
    if key not in ("matplotlib", "mpl", "plotly"):
        raise ValueError(
            f"Unknown plot backend {backend!r}. Expected 'matplotlib' or 'plotly'."
        )
