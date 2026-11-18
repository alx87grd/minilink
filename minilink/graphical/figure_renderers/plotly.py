"""A figure description drawn with Plotly: one subplot per panel."""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from minilink.graphical.common.plot_result import PlotResult
from minilink.graphical.common.show import show_figure
from minilink.graphical.figure import (
    Arrows,
    Contour,
    FigureSpec,
    Heatmap,
    Note,
    RefLine,
    Surface,
    Trace,
    overlay,
)

# The description's marker names in Plotly's spelling
MARKERS = {"o": "circle", "o-open": "circle-open", "x": "x", "+": "cross", "*": "star"}

# The colour maps whose matplotlib name Plotly spells otherwise
COLOR_MAPS = {"bwr": "RdBu_r", "coolwarm": "RdBu_r", "seismic": "RdBu_r"}


###############################################################################
# Public API
###############################################################################


def draw_figure(spec: FigureSpec, *, show, ax=None) -> PlotResult:
    """Draw ``spec`` on a new Plotly figure; ``ax`` is matplotlib's and refused here."""
    require_no_axes(ax)

    # The subplot grid, a 3-D scene where a panel is 3-D
    n = len(spec.panels)
    n_cols = min(spec.n_cols, n)
    n_rows = -(-n // n_cols)
    kinds = [{"type": "scene"} if panel.is_3d else {} for panel in spec.panels]
    kinds += [{}] * (n_rows * n_cols - n)
    fig = make_subplots(
        rows=n_rows,
        cols=n_cols,
        specs=[kinds[r * n_cols : (r + 1) * n_cols] for r in range(n_rows)],
        subplot_titles=[panel.title for panel in spec.panels],
        shared_xaxes=spec.share_x,
    )

    # The panels, then the figure's title and size
    for k, panel in enumerate(spec.panels):
        draw_panel(fig, panel, *cell_of(k, n_cols), axis_names(fig, k, panel))
    fig.update_layout(
        title=spec.title or None,
        template="plotly_white",
        legend={
            "x": 0.99,
            "y": 0.99,
            "xanchor": "right",
            "yanchor": "top",
            "bgcolor": "rgba(255, 255, 255, 0.7)",
        },
    )
    if spec.figsize is not None:
        fig.update_layout(width=100 * spec.figsize[0], height=100 * spec.figsize[1])

    show_figure(fig, "plotly", show)

    return PlotResult(renderer="plotly", figure=fig, axes=None, spec=spec)


def draw_into(host: PlotResult, spec: FigureSpec, *, show) -> PlotResult:
    """Add the layers of ``spec`` to the panels of the drawn Plotly figure ``host``."""
    n_cols = min(host.spec.n_cols, len(host.spec.panels))
    for k, panel in enumerate(spec.panels):
        row, col = cell_of(k, n_cols)
        for layer in panel.layers:
            DRAW[type(layer)](host.figure, layer, row, col)

    show_figure(host.figure, "plotly", show)

    combined = overlay(host.spec, [panel.layers for panel in spec.panels])

    return PlotResult(renderer="plotly", figure=host.figure, axes=None, spec=combined)


###############################################################################
# Internal machinery
###############################################################################


def require_no_axes(ax):
    """Refuse matplotlib axes as a Plotly figure's host."""
    if ax is not None:
        raise ValueError(
            "renderer='plotly' draws into a PlotResult of a Plotly figure, not into "
            "matplotlib axes; pass ax=<PlotResult> or renderer='matplotlib'"
        )


def cell_of(k, n_cols):
    """The 1-based ``(row, col)`` of panel ``k`` on a grid of ``n_cols`` columns."""
    return k // n_cols + 1, k % n_cols + 1


def axis_names(fig, k, panel):
    """The layout keys of panel ``k``'s axes: ``scene{k}`` in 3-D, ``xaxis{k}``, ``yaxis{k}`` else."""
    suffix = "" if k == 0 else str(k + 1)
    if panel.is_3d:
        return ("scene" + suffix,)

    return ("xaxis" + suffix, "yaxis" + suffix)


def draw_panel(fig, panel, row, col, names):
    """Draw one panel's layers, then its labels, limits and scales."""
    for layer in panel.layers:
        DRAW[type(layer)](fig, layer, row, col)

    # 3-D: the scene's three titles
    if panel.is_3d:
        fig.update_scenes(
            xaxis_title=panel.x_label,
            yaxis_title=panel.y_label,
            zaxis_title=panel.z_label,
            row=row,
            col=col,
        )
        return

    # 2-D: titles, limits, a log axis, an equal aspect
    fig.update_xaxes(
        title_text=panel.x_label,
        range=log_range(panel.x_lim, panel.x_log),
        type="log" if panel.x_log else None,
        row=row,
        col=col,
    )
    fig.update_yaxes(title_text=panel.y_label, range=panel.y_lim, row=row, col=col)
    if panel.equal_aspect:
        fig.update_yaxes(scaleanchor=names[0].replace("axis", ""), row=row, col=col)


def log_range(limits, is_log):
    """Plotly's range of an axis: the limits, in decades on a log axis."""
    if limits is None or not is_log:
        return limits

    return [float(np.log10(limits[0])), float(np.log10(limits[1]))]


def draw_trace(fig, trace: Trace, row, col):
    """A line, a filled polygon or a marker set."""
    x, y = np.asarray(trace.x, dtype=float), np.asarray(trace.y, dtype=float)
    common = {
        "name": trace.label or None,
        "showlegend": bool(trace.label),
        "opacity": trace.alpha,
        "hovertext": list(trace.hover) or None,
    }
    if trace.mode == "markers":
        style = {
            "mode": "markers",
            "marker": {
                "symbol": MARKERS.get(trace.marker, "circle"),
                "size": trace.size,
                "color": trace.color,
            },
        }
    else:
        style = {
            "mode": "lines",
            "line": {"color": trace.color, "width": trace.width, "dash": trace.dash},
        }
    if trace.fill:
        style["fill"] = "toself"
        style["fillcolor"] = trace.color
    if trace.z is not None:
        fig.add_trace(
            go.Scatter3d(
                x=x, y=y, z=np.asarray(trace.z, dtype=float), **style, **common
            ),
            row=row,
            col=col,
        )
        return
    fig.add_trace(go.Scatter(x=x, y=y, **style, **common), row=row, col=col)


def draw_ref_line(fig, line: RefLine, row, col):
    """A vertical (``axis="x"``) or horizontal reference line."""
    style = {"line_color": line.color, "line_dash": line.dash, "line_width": 1}
    if line.axis == "x":
        fig.add_vline(x=line.value, row=row, col=col, **style)
    else:
        fig.add_hline(y=line.value, row=row, col=col, **style)


def draw_note(fig, note: Note, row, col):
    """A boxed text anchored at a fraction of the panel."""
    fig.add_annotation(
        text=note.text.replace("\n", "<br>"),
        x=note.x,
        y=note.y,
        xref="x domain",
        yref="y domain",
        showarrow=False,
        align="left",
        bgcolor="white",
        bordercolor="#b3b3b3",
        row=row,
        col=col,
    )


def draw_heatmap(fig, heatmap: Heatmap, row, col):
    """A heatmap on the mesh, smooth between the nodes or one cell per value."""
    fig.add_trace(
        go.Heatmap(
            x=np.asarray(heatmap.x, dtype=float),
            y=np.asarray(heatmap.y, dtype=float),
            z=np.asarray(heatmap.z, dtype=float),
            colorscale=COLOR_MAPS.get(heatmap.cmap, heatmap.cmap),
            zmin=heatmap.vmin,
            zmax=heatmap.vmax,
            zsmooth="best" if heatmap.shading == "smooth" else False,
            opacity=heatmap.alpha,
            showscale=heatmap.colorbar is not None,
            colorbar={"title": heatmap.colorbar or None},
        ),
        row=row,
        col=col,
    )


def draw_contour(fig, contour: Contour, row, col):
    """Level lines, one trace per level, or a filled band as a mask between the outer levels."""
    x, y = np.asarray(contour.x, dtype=float), np.asarray(contour.y, dtype=float)
    z = np.asarray(contour.z, dtype=float)
    color = contour.color or "#1f77b4"

    # A filled band: the mesh points between the lowest and highest level
    if contour.filled:
        lower, upper = (
            (np.nanmin(z), np.nanmax(z))
            if contour.levels is None
            else (
                min(contour.levels),
                max(contour.levels),
            )
        )
        inside = np.where((z >= lower) & (z <= upper), 1.0, np.nan)
        fig.add_trace(
            go.Heatmap(
                x=x,
                y=y,
                z=inside,
                colorscale=[[0.0, color], [1.0, color]],
                showscale=False,
                opacity=contour.alpha,
                name=contour.label or None,
                showlegend=bool(contour.label),
                hoverinfo="skip",
            ),
            row=row,
            col=col,
        )
        return

    # Level lines: the levels given, each its own trace, else Plotly's own
    levels = [None] if contour.levels is None else list(contour.levels)
    for k, level in enumerate(levels):
        contours = {
            "coloring": "lines" if contour.cmap else "none",
            "showlabels": False,
        }
        if level is not None:
            contours.update(start=level, end=level, size=1.0)
        fig.add_trace(
            go.Contour(
                x=x,
                y=y,
                z=z,
                contours=contours,
                line={
                    "color": None if contour.cmap else color,
                    "width": contour.width,
                    "dash": contour.dash,
                },
                colorscale=COLOR_MAPS.get(contour.cmap, contour.cmap)
                if contour.cmap
                else None,
                showscale=False,
                opacity=contour.alpha,
                name=contour.label or None,
                showlegend=bool(contour.label) and k == 0,
            ),
            row=row,
            col=col,
        )


def draw_surface(fig, surface: Surface, row, col):
    """A 3-D surface over the mesh, coloured by value."""
    fig.add_trace(
        go.Surface(
            x=np.asarray(surface.x, dtype=float),
            y=np.asarray(surface.y, dtype=float),
            z=np.asarray(surface.z, dtype=float),
            colorscale=COLOR_MAPS.get(surface.cmap, surface.cmap),
            cmin=surface.vmin,
            cmax=surface.vmax,
            opacity=surface.alpha,
            showscale=surface.colorbar is not None,
            colorbar={"title": surface.colorbar or None},
        ),
        row=row,
        col=col,
    )


def draw_vector_field(fig, arrows: Arrows, row, col):
    """Arrows scaled so the longest spans 0.9 of a grid cell, or streamlines."""
    import plotly.figure_factory as ff

    X, Y = np.asarray(arrows.x, dtype=float), np.asarray(arrows.y, dtype=float)
    U, V = np.asarray(arrows.u, dtype=float), np.asarray(arrows.v, dtype=float)
    x_values, y_values = X[0, :], Y[:, 0]
    line = {"color": arrows.color or "#1f77b4", "width": 1}
    if arrows.style == "stream":
        x_values, y_values, U, V = square_grid(x_values, y_values, U, V)
        field = ff.create_streamline(x_values, y_values, U, V, density=1.2, line=line)
    else:
        cell = min(np.diff(x_values).min(), np.diff(y_values).min())
        longest = max(float(np.hypot(U, V).max()), 1e-12)
        field = ff.create_quiver(
            X, Y, U, V, scale=0.9 * cell / longest, arrow_scale=0.3, line=line
        )
    for k, trace in enumerate(field.data):
        trace.name = arrows.label or None
        trace.showlegend = bool(arrows.label) and k == 0
        fig.add_trace(trace, row=row, col=col)


def square_grid(x_values, y_values, U, V):
    """The field on an ``n × n`` grid over the same window: Plotly's streamlines need one."""
    if x_values.size == y_values.size:
        return x_values, y_values, U, V
    from scipy.interpolate import RegularGridInterpolator

    n = max(x_values.size, y_values.size)
    xs = np.linspace(x_values[0], x_values[-1], n)
    ys = np.linspace(y_values[0], y_values[-1], n)
    points = np.stack(np.meshgrid(ys, xs, indexing="ij"), axis=-1)
    on_grid = [RegularGridInterpolator((y_values, x_values), F)(points) for F in (U, V)]

    return xs, ys, *on_grid


# One draw function per layer kind
DRAW = {
    Trace: draw_trace,
    RefLine: draw_ref_line,
    Note: draw_note,
    Heatmap: draw_heatmap,
    Contour: draw_contour,
    Surface: draw_surface,
    Arrows: draw_vector_field,
}
