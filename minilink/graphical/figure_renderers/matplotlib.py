"""A figure description drawn with matplotlib: one axes per panel."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

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

# The description's marker names in matplotlib's spelling
MARKERS = {"o": "o", "o-open": "o", "s": "s", "x": "x", "+": "+", "*": "*"}

# The description's dash names in matplotlib's spelling
DASHES = {"solid": "-", "dash": "--", "dot": ":"}

FONT_SIZE = 10


###############################################################################
# Public API
###############################################################################


def draw_figure(spec: FigureSpec, *, show, ax=None) -> PlotResult:
    """Draw ``spec`` on a new figure, or on the caller's axes ``ax``."""
    # A new figure, or the caller's axes: one per panel
    if ax is None:
        fig, axes = new_axes(spec)
        fill_only = False
    else:
        axes = axes_list(ax, spec)
        fig = axes[0].figure
        fill_only = True

    # The panels, then the title above them
    for target, panel in zip(axes, spec.panels):
        draw_panel(target, panel, fill_only=fill_only)
    if spec.title:
        fig.suptitle(spec.title, fontsize=FONT_SIZE + 1)

    show_figure(fig, "matplotlib", show)

    return PlotResult(
        renderer="matplotlib",
        figure=fig,
        axes=axes[0] if len(axes) == 1 else axes,
        spec=spec,
    )


def draw_into(host: PlotResult, spec: FigureSpec, *, show) -> PlotResult:
    """Add the layers of ``spec`` to the panels of the drawn figure ``host``."""
    axes = axes_list(host.axes, spec)
    for target, panel in zip(axes, spec.panels):
        draw_panel(target, panel, fill_only=True)

    show_figure(host.figure, "matplotlib", show)

    combined = overlay(host.spec, [panel.layers for panel in spec.panels])

    return PlotResult(
        renderer="matplotlib", figure=host.figure, axes=host.axes, spec=combined
    )


###############################################################################
# Internal machinery
###############################################################################


def new_axes(spec: FigureSpec):
    """A figure of ``spec``'s panels on its grid: ``(fig, axes)``, 3-D panels on 3-D axes."""
    n = len(spec.panels)
    n_cols = min(spec.n_cols, n)
    n_rows = -(-n // n_cols)
    fig = plt.figure(figsize=spec.figsize)
    axes = []
    for k, panel in enumerate(spec.panels):
        share = axes[0] if spec.share_x and axes else None
        axes.append(
            fig.add_subplot(
                n_rows,
                n_cols,
                k + 1,
                projection="3d" if panel.is_3d else None,
                sharex=share,
            )
        )

    return fig, axes


def axes_list(ax, spec: FigureSpec) -> list:
    """The axes to draw ``spec``'s panels on, one per panel, refused on a count that differs."""
    axes = list(np.ravel(ax)) if isinstance(ax, (list, tuple, np.ndarray)) else [ax]
    if len(axes) != len(spec.panels):
        raise ValueError(
            f"this figure has {len(spec.panels)} panel(s); ax= holds {len(axes)} axes"
        )

    return axes


def draw_panel(ax, panel, *, fill_only):
    """Draw one panel's layers on ``ax``, then its labels, limits and legend.

    ``fill_only`` keeps what the axes already shows: a title, a label or a
    limit is set only where the axes has none.
    """
    if panel.is_3d and getattr(ax, "name", "") != "3d":
        raise ValueError("a 3-D plot draws on 3-D axes; pass axes with projection='3d'")
    for layer in panel.layers:
        DRAW[type(layer)](ax, layer)

    # The scales, the grid and the aspect
    if panel.x_log:
        ax.set_xscale("log")
        ax.grid(True, which="both", linestyle="-", alpha=0.35)
        ax.grid(True, which="minor", linestyle=":", alpha=0.25)
    else:
        ax.grid(True, alpha=0.3)
    if panel.equal_aspect:
        ax.set_aspect("equal", adjustable="box")

    # The limits, unless the axes holds its own
    if panel.x_lim is not None and (not fill_only or ax.get_autoscalex_on()):
        ax.set_xlim(panel.x_lim)
    if panel.y_lim is not None and (not fill_only or ax.get_autoscaley_on()):
        ax.set_ylim(panel.y_lim)

    # The title and the labels, unless the axes has its own
    set_text(ax.set_title, ax.get_title(), panel.title, fill_only)
    set_text(ax.set_xlabel, ax.get_xlabel(), panel.x_label, fill_only)
    set_text(ax.set_ylabel, ax.get_ylabel(), panel.y_label, fill_only)
    if panel.is_3d:
        set_text(ax.set_zlabel, ax.get_zlabel(), panel.z_label, fill_only)

    # A legend when a layer carries a label, each label once
    if panel.legend and any(getattr(layer, "label", "") for layer in panel.layers):
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), loc=panel.legend, fontsize=8)


def set_text(setter, current, text, fill_only):
    """Set an axes text, unless ``fill_only`` and the axes already has one."""
    if text and not (fill_only and current):
        setter(text)


def draw_trace(ax, trace: Trace):
    """A line, a filled polygon or a marker set."""
    x, y = np.asarray(trace.x, dtype=float), np.asarray(trace.y, dtype=float)
    label = trace.label or None

    # A closed, filled polygon
    if trace.fill:
        ax.fill(x, y, color=trace.color, alpha=trace.alpha, label=label)
        return

    # A marker set, an open circle for "o-open"
    coords = (x, y) if trace.z is None else (x, y, np.asarray(trace.z, dtype=float))
    if trace.mode == "markers":
        ax.plot(
            *coords,
            linestyle="none",
            marker=MARKERS.get(trace.marker, "o"),
            markersize=trace.size,
            markerfacecolor="none" if trace.marker == "o-open" else trace.color,
            color=trace.color,
            alpha=trace.alpha,
            label=label,
        )
        return

    # A line, and its direction arrows
    ax.plot(
        *coords,
        color=trace.color,
        linewidth=trace.width,
        linestyle=DASHES.get(trace.dash, "-"),
        alpha=trace.alpha,
        label=label,
    )
    if trace.arrows:
        draw_direction_arrows(ax, x, y, ax.lines[-1].get_color())


def draw_ref_line(ax, line: RefLine):
    """A vertical (``axis="x"``) or horizontal reference line."""
    style = {"color": line.color, "linestyle": DASHES.get(line.dash, "--"), "lw": 0.8}
    if line.axis == "x":
        ax.axvline(line.value, **style)
    else:
        ax.axhline(line.value, **style)


def draw_note(ax, note: Note):
    """A boxed text anchored at a fraction of the panel, growing away from its edge."""
    ax.text(
        note.x,
        note.y,
        note.text,
        transform=ax.transAxes,
        fontsize=FONT_SIZE - 1,
        va="bottom" if note.y < 0.5 else "top",
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.8,
            "edgecolor": "0.7",
        },
    )


def draw_heatmap(ax, heatmap: Heatmap):
    """A heatmap on the mesh, smooth between the nodes or one cell per value, and its colour bar."""
    from matplotlib.colors import PowerNorm

    norm = None
    if heatmap.gamma != 1.0:
        norm = PowerNorm(gamma=heatmap.gamma, vmin=heatmap.vmin, vmax=heatmap.vmax)
    mesh = ax.pcolormesh(
        heatmap.x,
        heatmap.y,
        heatmap.z,
        shading="gouraud" if heatmap.shading == "smooth" else "auto",
        cmap=heatmap.cmap,
        norm=norm,
        alpha=heatmap.alpha,
    )
    if norm is None:
        mesh.set_clim(vmin=heatmap.vmin, vmax=heatmap.vmax)
    if heatmap.colorbar is not None:
        ax.figure.colorbar(mesh, ax=ax, label=heatmap.colorbar or None)


def draw_contour(ax, contour: Contour):
    """Level lines, or filled bands between levels, and a legend entry when labelled."""
    style = {"levels": None if contour.levels is None else list(contour.levels)}
    if contour.color is not None:
        style["colors"] = [contour.color]
    elif contour.cmap is not None:
        style["cmap"] = contour.cmap
    if contour.filled:
        ax.contourf(contour.x, contour.y, contour.z, alpha=contour.alpha, **style)
        if contour.label:
            ax.fill([], [], color=contour.color, label=contour.label)
        return

    ax.contour(
        contour.x,
        contour.y,
        contour.z,
        linewidths=contour.width,
        linestyles=DASHES.get(contour.dash, "-"),
        alpha=contour.alpha,
        **style,
    )
    if contour.label:
        ax.plot(
            [], [], color=contour.color, linewidth=contour.width, label=contour.label
        )


def draw_surface(ax, surface: Surface):
    """A 3-D surface over the mesh, coloured by value, and its colour bar."""
    mesh_x, mesh_y = np.meshgrid(surface.x, surface.y)
    drawn = ax.plot_surface(
        mesh_x,
        mesh_y,
        np.asarray(surface.z, dtype=float),
        cmap=surface.cmap,
        vmin=surface.vmin,
        vmax=surface.vmax,
        alpha=surface.alpha,
        linewidth=0,
        antialiased=False,
    )
    if surface.colorbar is not None:
        ax.figure.colorbar(drawn, ax=ax, shrink=0.6, label=surface.colorbar or None)


def draw_vector_field(ax, arrows: Arrows):
    """One arrow per grid node, or streamlines through the field."""
    if arrows.style == "stream":
        ax.streamplot(arrows.x, arrows.y, arrows.u, arrows.v, color=arrows.color)
    else:
        ax.quiver(arrows.x, arrows.y, arrows.u, arrows.v, color=arrows.color)
    if arrows.label:
        ax.plot([], [], color=arrows.color, label=arrows.label)


def draw_direction_arrows(ax, x, y, color, count: int = 3):
    """Direction arrows spread over the middle of a line."""
    if x.size < 3:
        return

    # Over the middle 70 % of the samples, each 1/50 of them long, none past the end
    span = max(1, x.size // 50)
    starts = np.linspace(x.size * 0.15, x.size * 0.85, count).astype(int)
    starts = np.minimum(starts, x.size - 1 - span)
    for k in starts:
        ax.annotate(
            "",
            xy=(x[k + span], y[k + span]),
            xytext=(x[k], y[k]),
            arrowprops={"arrowstyle": "-|>", "color": color, "lw": 1.5},
        )


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
