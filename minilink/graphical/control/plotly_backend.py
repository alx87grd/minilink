"""Control figures in Plotly: one subplot row per panel of a ControlFigure."""

from __future__ import annotations

import numpy as np

from minilink.graphical.common import PlotResult
from minilink.graphical.common.plotly_style import (
    PLOTLY_2D_MARGIN,
    PLOTLY_FIG_WIDTH,
    PLOTLY_TEMPLATE,
)
from minilink.graphical.control.figure_spec import ControlFigure, Panel
from minilink.graphical.control.style import AXIS_COLOR

# The spec's marker names as Plotly symbols; an unknown one draws as "x-thin"
_SYMBOLS = {"x": "x-thin", "o": "circle-open", "+": "cross-thin"}


###############################################################################
# Public API
###############################################################################


def render_plotly(spec: ControlFigure, *, show: bool = True) -> PlotResult:
    """Draw ``spec`` with Plotly, one subplot row per panel."""
    go, make_subplots = require_plotly()

    # One subplot row per panel
    n_panels = len(spec.panels)
    fig = make_subplots(
        rows=n_panels, cols=1, shared_xaxes=spec.share_x, vertical_spacing=0.08
    )
    for row, panel in enumerate(spec.panels, start=1):
        draw_panel(fig, go, panel, row)

    # The title, with the subtitle in small type below it
    title = (
        spec.title
        if not spec.subtitle
        else f"{spec.title}<br><sup>{spec.subtitle}</sup>"
    )

    # The layout: 420 px for one panel, 300 px per panel for a stack, and a
    # legend when a trace carries a label
    fig.update_layout(
        title=title,
        width=PLOTLY_FIG_WIDTH,
        height=420 if n_panels == 1 else 300 * n_panels,
        showlegend=any(trace.label for panel in spec.panels for trace in panel.traces),
        margin=dict(PLOTLY_2D_MARGIN),
        template=PLOTLY_TEMPLATE,
    )
    if show:
        fig.show()

    return PlotResult(backend="plotly", payload=fig, figure=fig)


###############################################################################
# Internal machinery
###############################################################################


def require_plotly():
    """Refuse to draw without Plotly; return ``(go, make_subplots)``."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise ImportError(
            "Plotly control plots require Plotly. Install with: pip install 'minilink[plotting]'"
        ) from exc

    return go, make_subplots


def draw_panel(fig, go, panel: Panel, row: int) -> None:
    """Draw one panel in subplot ``row``: its traces, lines and notes, then its axes."""
    # The traces, each with its hover texts when it has them: marker sets, or
    # lines with their arrows
    for trace in panel.traces:
        hover = {"hovertext": trace.hover, "hoverinfo": "text"} if trace.hover else {}
        if trace.mode == "markers":
            fig.add_trace(
                go.Scatter(
                    x=trace.x,
                    y=trace.y,
                    mode="markers",
                    name=trace.label,
                    showlegend=bool(trace.label),
                    marker={
                        "symbol": _SYMBOLS.get(trace.marker, "x-thin"),
                        "size": trace.size + 2,
                        "color": trace.color,
                        "line": {"width": trace.width, "color": trace.color},
                    },
                    **hover,
                ),
                row=row,
                col=1,
            )
        else:
            fig.add_trace(
                go.Scatter(
                    x=trace.x,
                    y=trace.y,
                    mode="lines",
                    name=trace.label,
                    showlegend=bool(trace.label),
                    line={
                        "color": trace.color,
                        "width": trace.width,
                        "dash": trace.dash,
                    },
                    **hover,
                ),
                row=row,
                col=1,
            )
            if trace.arrows:
                draw_arrows(fig, trace.x, trace.y, trace.color, row)

    # The reference lines: vertical at x = value, or horizontal at y = value
    for line in panel.lines:
        style = {"line_color": line.color, "line_dash": line.dash, "line_width": 1}
        if line.axis == "x":
            fig.add_vline(x=line.value, row=row, col=1, **style)
        else:
            fig.add_hline(y=line.value, row=row, col=1, **style)

    # The notes, boxed, anchored at a fraction of the panel; a note in the lower
    # half grows upward, one in the upper half downward
    for note in panel.notes:
        fig.add_annotation(
            text=note.text.replace("\n", "<br>"),
            xref="x domain",
            yref="y domain",
            x=note.x,
            y=note.y,
            xanchor="left",
            yanchor="bottom" if note.y < 0.5 else "top",
            showarrow=False,
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="#b0b0b0",
            row=row,
            col=1,
        )

    # The axes, gridded: a log frequency axis also gets its minor grid
    x_axis = {"title_text": panel.x_label, "showgrid": True}
    y_axis = {"title_text": panel.y_label, "showgrid": True}
    if panel.x_log:
        x_axis.update(
            {"type": "log", "minor": {"showgrid": True, "gridcolor": "#eeeeee"}}
        )

    # The real and imaginary axes of an s-plane plot
    if panel.zero_lines:
        x_axis.update({"zeroline": True, "zerolinecolor": AXIS_COLOR})
        y_axis.update({"zeroline": True, "zerolinecolor": AXIS_COLOR})

    # The limits and ticks the spec fixes; a log axis takes its range in
    # decades, log₁₀ of the limits
    if panel.x_lim is not None:
        x_axis["range"] = (
            list(np.log10(panel.x_lim)) if panel.x_log else list(panel.x_lim)
        )
    if panel.y_lim is not None:
        y_axis["range"] = list(panel.y_lim)
    if panel.y_ticks is not None:
        y_axis.update({"tickmode": "array", "tickvals": list(panel.y_ticks)})

    fig.update_xaxes(row=row, col=1, **x_axis)
    fig.update_yaxes(row=row, col=1, **y_axis)


def draw_arrows(fig, x, y, color, row, count: int = 3) -> None:
    """Direction arrows spread along a line, in subplot ``row``."""
    x = np.asarray(x)
    y = np.asarray(y)

    # Too few samples to show a direction
    if x.size < 3:
        return

    # The arrows spread over the middle 70 % of the samples, each one 1/50 of
    # them long (one sample at least), none running past the last sample
    span = max(1, x.size // 50)
    starts = np.linspace(x.size * 0.15, x.size * 0.85, count).astype(int)
    starts = np.minimum(starts, x.size - 1 - span)
    for k in starts:
        fig.add_annotation(
            x=x[k + span],
            y=y[k + span],
            ax=x[k],
            ay=y[k],
            xref="x",
            yref="y",
            axref="x",
            ayref="y",
            showarrow=True,
            arrowhead=2,
            arrowcolor=color,
            arrowwidth=1.2,
            row=row,
            col=1,
        )
