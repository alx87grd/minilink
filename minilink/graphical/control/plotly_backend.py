"""Plotly renderer for :class:`~minilink.graphical.control.figure_spec.ControlFigure`."""

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

_SYMBOLS = {"x": "x-thin", "o": "circle-open", "+": "cross-thin"}


def render_plotly(spec: ControlFigure, *, show: bool = True) -> PlotResult:
    go, make_subplots = _import_plotly()

    n_panels = len(spec.panels)
    fig = make_subplots(
        rows=n_panels, cols=1, shared_xaxes=spec.share_x, vertical_spacing=0.08
    )
    for row, panel in enumerate(spec.panels, start=1):
        _draw_panel(fig, go, panel, row)

    title = (
        spec.title
        if not spec.subtitle
        else f"{spec.title}<br><sup>{spec.subtitle}</sup>"
    )
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


def _draw_panel(fig, go, panel: Panel, row: int) -> None:
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
                _arrows(fig, trace.x, trace.y, trace.color, row)
    for line in panel.lines:
        style = {"line_color": line.color, "line_dash": line.dash, "line_width": 1}
        if line.axis == "x":
            fig.add_vline(x=line.value, row=row, col=1, **style)
        else:
            fig.add_hline(y=line.value, row=row, col=1, **style)
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

    x_axis = {"title_text": panel.x_label, "showgrid": True}
    y_axis = {"title_text": panel.y_label, "showgrid": True}
    if panel.x_log:
        x_axis.update(
            {"type": "log", "minor": {"showgrid": True, "gridcolor": "#eeeeee"}}
        )
    if panel.zero_lines:
        x_axis.update({"zeroline": True, "zerolinecolor": AXIS_COLOR})
        y_axis.update({"zeroline": True, "zerolinecolor": AXIS_COLOR})
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


def _arrows(fig, x, y, color, row, count: int = 3) -> None:
    x, y = np.asarray(x), np.asarray(y)
    if x.size < 3:
        return
    span = max(1, x.size // 50)
    for k in np.linspace(x.size * 0.15, x.size * 0.85, count).astype(int):
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


def _import_plotly():
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise ImportError(
            "Plotly control plots require Plotly. Install with: pip install 'minilink[plotting]'"
        ) from exc
    return go, make_subplots
