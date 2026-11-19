"""A figure description drawn with matplotlib: one axes per panel."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import PowerNorm

from minilink.graphical.common.matplotlib_style import DPI_FIGURE, FONT_SIZE
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
    direction_arrow_spans,
    grid_of,
)

# The description's marker and dash names in matplotlib's spelling
MARKERS = {"o": "o", "o-open": "o", "s": "s", "x": "x", "+": "+", "*": "*"}
DASHES = {"solid": "-", "dash": "--", "dot": ":"}


###############################################################################
# Public API
###############################################################################


def draw_figure(spec: FigureSpec, *, show, ax=None) -> PlotResult:
    """Draw ``spec`` on a new figure, or on the caller's axes ``ax``."""
    # A new figure, or the axes given: one per panel, each able to take its panel
    given = ax is not None
    if given:
        axes = axes_list(ax, spec)
        fig = axes[0].figure
        for target, panel in zip(axes, spec.panels):
            require_fitting_axes(target, panel)
    else:
        fig, axes = new_axes(spec)

    # The title belongs above a figure these panels fill: a new one, or a given
    # one that holds no other axes and no title of its own
    fills_figure = not given or (len(fig.axes) == len(axes) and not fig.get_suptitle())

    # Panels on one time axis share one legend beside them, on a figure drawn here;
    # given axes are left as they are
    beside = spec.share_x and not given

    # The panels, then the title above them, then the legend beside them
    for target, panel in zip(axes, spec.panels):
        draw_panel(target, panel, given=given, legend=not beside)
    if spec.title and fills_figure:
        fig.suptitle(spec.title, fontsize=FONT_SIZE + 1)
    if beside:
        legend_beside(fig, axes, spec.panels)
    elif not given:
        fig.tight_layout()

    # A plot shows the figure it makes; a given figure is its owner's to show
    if not given:
        show_figure(fig, "matplotlib", show)

    return PlotResult(
        renderer="matplotlib",
        figure=fig,
        axes=axes[0] if len(axes) == 1 else axes,
        spec=spec,
    )


###############################################################################
# Internal machinery
###############################################################################


def new_axes(spec: FigureSpec):
    """A figure of ``spec``'s panels on its grid: ``(fig, axes)``, 3-D panels on 3-D axes."""
    n = len(spec.panels)
    n_rows, n_cols = grid_of(spec)
    fig = plt.figure(figsize=spec.figsize, dpi=DPI_FIGURE)
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

    # A shared x axis is labelled under the bottom panel of each column only
    if spec.share_x:
        for k, ax in enumerate(axes):
            if k + n_cols < n:
                ax.tick_params(labelbottom=False)

    return fig, axes


def axes_list(ax, spec: FigureSpec) -> list:
    """The axes to draw ``spec``'s panels on, one per panel, refused on a count that differs."""
    axes = list(np.ravel(ax)) if isinstance(ax, (list, tuple, np.ndarray)) else [ax]
    if len(axes) != len(spec.panels):
        raise ValueError(
            f"this figure has {len(spec.panels)} panel(s); ax= holds {len(axes)} axes"
        )

    return axes


def draw_panel(ax, panel, *, given, legend=True):
    """Draw one panel's layers on ``ax``, then its labels, limits and legend.

    On a new figure the panel sets everything. On ``given`` axes it keeps what
    they already show: a title or a label is set only where the axes has
    none; empty axes take the panel's limits unless the caller set its own;
    axes that already show something widen their view to hold them.
    ``legend=False`` leaves the panel's legend to the figure's.
    """
    # What the axes shows before these layers, and its view
    ax.autoscale_view()
    x_shown, y_shown = shows_something(ax)
    x_view, y_view = ax.get_xlim(), ax.get_ylim()
    for layer in panel.layers:
        DRAW[type(layer)](ax, layer)

    # The scales, the grid and the aspect
    if panel.x_log:
        ax.set_xscale("log")
        ax.grid(True, which="both", linestyle="-", alpha=0.35)
        ax.grid(True, which="minor", linestyle=":", alpha=0.25)
    else:
        ax.grid(True, alpha=0.3)
    if panel.y_ticks is not None:
        ax.set_yticks(panel.y_ticks)
    if panel.equal_aspect:
        ax.set_aspect("equal", adjustable="box")

    # The limits: a view that shows something widens to hold them, never shrinks
    if panel.x_lim is not None:
        if x_shown:
            ax.set_xlim(wider(x_view, panel.x_lim))
        elif not given or ax.get_autoscalex_on():
            ax.set_xlim(panel.x_lim)
    if panel.y_lim is not None:
        if y_shown:
            ax.set_ylim(wider(y_view, panel.y_lim))
        elif not given or ax.get_autoscaley_on():
            ax.set_ylim(panel.y_lim)

    # The title and the labels, unless the given axes has its own
    set_text(ax.set_title, ax.get_title(), panel.title, given)
    set_text(ax.set_xlabel, ax.get_xlabel(), panel.x_label, given)
    set_text(ax.set_ylabel, ax.get_ylabel(), panel.y_label, given)
    if panel.is_3d:
        set_text(ax.set_zlabel, ax.get_zlabel(), panel.z_label, given)

    # A legend when a layer carries a label: each label once, with the first
    # handle drawn for it
    if legend and panel.legend and any(labels_of(panel)):
        unique = legend_entries([ax])
        legend = ax.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
        opaque_keys(legend)


def legend_beside(fig, axes, panels):
    """One legend right of the panels, each label of every panel that asks for one once; the panels laid out in the width left."""
    asking = [
        ax for ax, panel in zip(axes, panels) if panel.legend and any(labels_of(panel))
    ]
    if not asking:
        fig.tight_layout()
        return

    unique = legend_entries(asking)
    legend = fig.legend(
        unique.values(), unique.keys(), loc="upper right", fontsize=8, frameon=False
    )
    opaque_keys(legend)
    width = legend.get_window_extent().width / fig.bbox.width
    fig.tight_layout(rect=(0.0, 0.0, 1.0 - width, 1.0))


def opaque_keys(legend):
    """Draw a legend's keys at full opacity: a faint layer's label still reads."""
    for handle in legend.legend_handles:
        handle.set_alpha(1.0)


def labels_of(panel):
    """The labels a panel's layers carry, empty ones included."""
    return [getattr(layer, "label", "") for layer in panel.layers]


def legend_entries(axes) -> dict:
    """``{label: handle}`` over ``axes``, each label once with the first handle drawn for it."""
    unique = {}
    for ax in axes:
        for handle, label in zip(*ax.get_legend_handles_labels()):
            unique.setdefault(label, handle)

    return unique


def shows_something(ax):
    """Whether the view along x, and along y, already shows something: on ``ax`` or on axes that share that axis with it."""
    x_shown = any(peer.has_data() for peer in ax.get_shared_x_axes().get_siblings(ax))
    y_shown = any(peer.has_data() for peer in ax.get_shared_y_axes().get_siblings(ax))

    return x_shown, y_shown


def require_fitting_axes(ax, panel):
    """Refuse axes a panel cannot be drawn on: 3-D on 2-D, 2-D on 3-D, a second colour bar."""
    is_3d_axes = getattr(ax, "name", "") == "3d"
    if panel.is_3d and not is_3d_axes:
        raise ValueError("a 3-D plot draws on 3-D axes; pass axes with projection='3d'")
    if is_3d_axes and not panel.is_3d:
        raise ValueError("a 2-D plot does not draw on 3-D axes; pass 2-D axes")

    # A colour bar hangs on the image or the mesh it reads
    drawn = list(ax.collections) + list(ax.images)
    has_bar = any(getattr(artist, "colorbar", None) is not None for artist in drawn)
    adds_bar = any(
        getattr(layer, "colorbar", None) is not None for layer in panel.layers
    )
    if has_bar and adds_bar:
        raise ValueError(
            "these axes already show a colour bar, and this plot draws its own; "
            "draw it on axes of its own"
        )


def wider(view, limits):
    """The view that holds both ``view`` and ``limits``, in ``view``'s direction."""
    low = min(*view, *limits)
    high = max(*view, *limits)

    return (low, high) if view[0] <= view[1] else (high, low)


def set_text(setter, current, text, given):
    """Set an axes text, unless the axes is ``given`` and already has one."""
    if text and not (given and current):
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
        bar = ax.figure.colorbar(mesh, ax=ax, label=heatmap.colorbar or None)
        offset_ticks(bar, heatmap.tick_offset)


def draw_contour(ax, contour: Contour):
    """Level lines, or filled bands between levels, and a legend entry when labelled."""
    style = {"levels": None if contour.levels is None else list(contour.levels)}
    if contour.color is not None:
        style["colors"] = [contour.color]
    if contour.filled:
        ax.contourf(contour.x, contour.y, contour.z, **style)
        if contour.label:
            ax.fill([], [], color=contour.color, label=contour.label)
        return

    ax.contour(
        contour.x,
        contour.y,
        contour.z,
        linewidths=contour.width,
        linestyles=DASHES.get(contour.dash, "-"),
        **style,
    )
    if contour.label:
        ax.plot(
            [], [], color=contour.color, linewidth=contour.width, label=contour.label
        )


def draw_surface(ax, surface: Surface):
    """A 3-D surface over the mesh, coloured by value, and its colour bar."""
    mesh_x, mesh_y = np.meshgrid(surface.x, surface.y)
    scale = {"vmin": surface.vmin, "vmax": surface.vmax}
    if surface.gamma != 1.0:
        scale = {"norm": PowerNorm(gamma=surface.gamma, **scale)}
    if surface.stride is not None:
        scale.update(rstride=surface.stride, cstride=surface.stride)
    drawn = ax.plot_surface(
        mesh_x,
        mesh_y,
        np.asarray(surface.z, dtype=float),
        cmap=surface.cmap,
        alpha=surface.alpha,
        linewidth=0,
        antialiased=False,
        **scale,
    )
    if surface.colorbar is not None:
        bar = ax.figure.colorbar(
            drawn, ax=ax, shrink=0.6, label=surface.colorbar or None
        )
        offset_ticks(bar, surface.tick_offset)


def offset_ticks(bar, offset):
    """Label each colour-bar tick ``t`` as ``t + offset``, to one decimal; nothing for 0."""
    if offset:
        ticks = [t for t in bar.get_ticks() if bar.vmin <= t <= bar.vmax]
        bar.set_ticks(ticks)
        bar.set_ticklabels([f"{t + offset:.1f}" for t in ticks])


def draw_vector_field(ax, arrows: Arrows):
    """One arrow per grid node, or streamlines through the field."""
    if arrows.style == "stream":
        ax.streamplot(arrows.x, arrows.y, arrows.u, arrows.v, color=arrows.color)
    else:
        ax.quiver(arrows.x, arrows.y, arrows.u, arrows.v, color=arrows.color)


def draw_direction_arrows(ax, x, y, color):
    """Direction arrows spread over the middle of a line."""
    for start, end in direction_arrow_spans(x.size):
        if not np.isfinite([x[start], y[start], x[end], y[end]]).all():
            continue
        ax.annotate(
            "",
            xy=(x[end], y[end]),
            xytext=(x[start], y[start]),
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
