"""A figure as plain data: panels of layers that any renderer draws."""

from __future__ import annotations

import importlib
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from minilink.graphical.common.plot_result import PlotResult

# The ten categorical colours in hex, which every renderer reads: blue, orange,
# green, red, purple, brown, pink, grey, olive, cyan
TAB10 = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
)

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class FigureSpec:
    """A titled grid of panels, read row by row, ``n_cols`` per row."""

    panels: tuple[Panel, ...]
    title: str = ""
    n_cols: int = 1
    share_x: bool = False
    figsize: tuple[float, float] | None = None


@dataclass(frozen=True)
class Panel:
    """One axes: its layers, drawn in order, and how the axes are labelled and scaled."""

    layers: tuple = ()
    title: str = ""
    x_label: str = ""
    y_label: str = ""
    z_label: str = ""
    x_lim: tuple[float, float] | None = None
    y_lim: tuple[float, float] | None = None
    x_log: bool = False
    y_ticks: tuple[float, ...] | None = None
    equal_aspect: bool = False
    is_3d: bool = False
    legend: bool = True  # A legend when a layer is labelled


@dataclass(frozen=True)
class Trace:
    """A polyline or a point set; NaN breaks a line, ``fill`` closes and fills it.

    A colour is a CSS name or a hex code, which every renderer reads;
    ``color=None`` takes the renderer's colour cycle. ``hover`` holds one text
    per point for the renderers that show it.
    """

    x: Any
    y: Any
    z: Any = None
    color: str | None = None
    mode: str = "lines"  # "lines" | "markers"
    marker: str = "o"  # "o" | "o-open" | "s" | "x" | "+" | "*"
    size: float = 6.0
    width: float = 1.5
    dash: str = "solid"  # "solid" | "dash" | "dot"
    label: str = ""
    alpha: float = 1.0
    fill: bool = False
    arrows: bool = False  # Direction arrows along a line
    hover: tuple[str, ...] = ()


@dataclass(frozen=True)
class RefLine:
    """A reference line across the panel: ``axis`` is ``"x"`` (vertical) or ``"y"``."""

    axis: str
    value: float
    color: str = "#7f7f7f"
    dash: str = "dash"


@dataclass(frozen=True)
class Note:
    """A text box anchored at a fraction of the panel (``0..1``)."""

    text: str
    x: float = 0.02
    y: float = 0.05


@dataclass(frozen=True)
class Heatmap:
    """Values ``z[j, i]`` at ``(x[i], y[j])`` drawn as colours.

    ``shading="smooth"`` blends between the mesh nodes, ``"cells"`` paints a
    cell around each value. ``colorbar`` is the colour bar's label, ``None``
    for none. ``gamma`` < 1 stretches the low values of the colour scale.
    ``tick_offset`` labels each colour-bar tick ``t`` as ``t + tick_offset``,
    for values drawn shifted.
    """

    x: Any
    y: Any
    z: Any
    cmap: str = "viridis"
    vmin: float | None = None
    vmax: float | None = None
    shading: str = "smooth"  # "smooth" | "cells"
    colorbar: str | None = ""
    alpha: float = 1.0
    gamma: float = 1.0
    tick_offset: float = 0.0


@dataclass(frozen=True)
class Contour:
    """Level sets of ``z[j, i]`` at ``(x[i], y[j])``: lines, or bands between levels when ``filled``.

    ``levels`` are the values drawn (``None`` lets the renderer choose);
    ``color`` draws every level in one colour.
    """

    x: Any
    y: Any
    z: Any
    levels: tuple[float, ...] | None = None
    filled: bool = False
    color: str | None = None
    width: float = 1.0
    dash: str = "solid"
    label: str = ""


@dataclass(frozen=True)
class Surface:
    """Values ``z[j, i]`` at ``(x[i], y[j])`` as a 3-D surface, coloured by value.

    ``gamma`` and ``tick_offset`` scale and label the colours as a
    :class:`Heatmap`'s do; ``stride`` draws every ``stride``-th mesh line,
    ``None`` lets the renderer choose.
    """

    x: Any
    y: Any
    z: Any
    cmap: str = "viridis"
    vmin: float | None = None
    vmax: float | None = None
    colorbar: str | None = ""
    alpha: float = 1.0
    gamma: float = 1.0
    tick_offset: float = 0.0
    stride: int | None = None


@dataclass(frozen=True)
class Arrows:
    """A vector field ``(u, v)`` on the grid ``(x, y)``: one arrow per node, or streamlines."""

    x: Any
    y: Any
    u: Any
    v: Any
    style: str = "quiver"  # "quiver" | "stream"
    color: str | None = None


def draw(spec: FigureSpec, *, renderer="matplotlib", show=None, ax=None):
    """
    Draw ``spec`` with ``renderer`` and return its :class:`PlotResult`.

    ``ax`` is where to draw: a matplotlib ``Axes`` (a list of them for several
    panels), or a :class:`PlotResult`, whose panels receive these layers; a
    line in a colour its host panel already draws takes the next free one, and
    the host's limits and ticks widen to hold both. ``show=None`` shows the figure unless it is drawn into ``ax``.
    ``renderer=None`` draws nothing: the record holds the description alone.
    """
    require_panels(spec)
    if show is None:
        show = ax is None
    if isinstance(renderer, str):
        renderer = renderer.strip().lower()

    # The description alone
    if renderer is None:
        return PlotResult(renderer=None, figure=None, axes=None, spec=spec)

    # Into a drawn figure: its panels take the layers, on its renderer
    if isinstance(ax, PlotResult):
        require_drawn_host(ax)
        require_same_renderer(renderer, ax.renderer)
        require_composable(ax.spec, spec)
        module = renderer_module(ax.renderer)
        apart = stacked_notes(ax.spec, distinct_colors(ax.spec, spec))
        return module.draw_into(ax, apart, show=show)

    module = renderer_module(renderer)
    return module.draw_figure(spec, show=show, ax=ax)


def overlay(host: FigureSpec, layers_by_panel) -> FigureSpec:
    """``host`` with ``layers_by_panel[k]`` added to its panel ``k``, a new description."""
    panels = tuple(
        replace(panel, layers=panel.layers + tuple(layers))
        for panel, layers in zip(host.panels, layers_by_panel)
    )

    return replace(host, panels=panels)


def composed(host: FigureSpec, spec: FigureSpec) -> FigureSpec:
    """``host`` with ``spec`` drawn into it, a new description.

    Each host panel takes the overlay panel's layers, the title and labels it
    has none of, and limits and ticks that hold both panels'.
    """
    panels = tuple(
        replace(
            host_panel,
            layers=host_panel.layers + panel.layers,
            title=host_panel.title or panel.title,
            x_label=host_panel.x_label or panel.x_label,
            y_label=host_panel.y_label or panel.y_label,
            z_label=host_panel.z_label or panel.z_label,
            x_lim=both_limits(host_panel.x_lim, panel.x_lim),
            y_lim=both_limits(host_panel.y_lim, panel.y_lim),
            y_ticks=both_ticks(host_panel.y_ticks, panel.y_ticks),
            legend=host_panel.legend or panel.legend,
        )
        for host_panel, panel in zip(host.panels, spec.panels)
    )

    return replace(host, panels=panels)


###############################################################################
# Internal machinery
###############################################################################

# The renderers by name, imported when first drawn with
RENDERER_MODULES = {
    "matplotlib": "minilink.graphical.figure_renderers.matplotlib",
    "plotly": "minilink.graphical.figure_renderers.plotly",
}


def renderer_module(renderer):
    """The module that draws with ``renderer``."""
    require_renderer(renderer)

    return importlib.import_module(RENDERER_MODULES[renderer])


def require_panels(spec):
    """Refuse a figure without a panel: there is nothing to draw."""
    if not spec.panels:
        raise ValueError("a figure needs at least one panel; this one has none")


def require_renderer(renderer):
    """Refuse a renderer the figures do not draw with."""
    if renderer not in RENDERER_MODULES:
        names = ", ".join(repr(name) for name in RENDERER_MODULES)
        raise ValueError(f"renderer must be one of {names} or None, got {renderer!r}")


def require_drawn_host(host):
    """Refuse a host that holds a description only: it has no figure to draw into."""
    if host.renderer is None:
        raise ValueError(
            "the host record holds a description only (renderer=None) and no figure "
            "to draw into; draw the host first, with draw(host.spec, renderer=...)"
        )


def require_same_renderer(renderer, host_renderer):
    """Refuse drawing into a figure with another renderer than its own; the default renderer follows the host's."""
    if renderer != host_renderer and renderer != "matplotlib":
        raise ValueError(
            f"the host figure is drawn with {host_renderer!r}; "
            f"drop renderer= or pass renderer={host_renderer!r}"
        )


def require_composable(host: FigureSpec, spec: FigureSpec):
    """Refuse an overlay its host cannot take: another panel count, 3-D into 2-D, 2-D into 3-D, a second colour bar."""
    if host is None:
        raise ValueError(
            "the host figure has no description to draw into; draw it with a minilink "
            "plot, or pass matplotlib axes as ax="
        )
    if len(host.panels) != len(spec.panels):
        raise ValueError(
            f"the host has {len(host.panels)} panel(s), this plot {len(spec.panels)}; "
            "draw into a host with as many panels"
        )
    for host_panel, panel in zip(host.panels, spec.panels):
        if panel.is_3d and not host_panel.is_3d:
            raise ValueError("a 3-D plot does not draw into a 2-D panel")
        if host_panel.is_3d and not panel.is_3d:
            raise ValueError(
                "a 2-D plot does not draw into a 3-D panel; draw it into a 2-D host"
            )
        if has_colorbar(host_panel) and has_colorbar(panel):
            raise ValueError(
                "the host panel already has a colour bar; draw the overlay without one"
            )


def both_limits(host_lim, lim):
    """The limits that hold both: the one given when the other is ``None``."""
    if host_lim is None or lim is None:
        return lim if host_lim is None else host_lim

    return (min(host_lim[0], lim[0]), max(host_lim[1], lim[1]))


def both_ticks(host_ticks, ticks):
    """The ticks of both, in order: the ones given when the other is ``None``."""
    if host_ticks is None or ticks is None:
        return ticks if host_ticks is None else host_ticks

    return tuple(sorted(set(host_ticks) | set(ticks)))


# The height of a note's line and of its box's padding, as fractions of the panel
NOTE_LINE = 0.085
NOTE_PAD = 0.06


def stacked_notes(host: FigureSpec, spec: FigureSpec) -> FigureSpec:
    """``spec`` with each note anchored where its host panel already has one moved clear of it, away from the panel's edge."""
    panels = []
    for host_panel, panel in zip(host.panels, spec.panels):
        placed = [layer for layer in host_panel.layers if isinstance(layer, Note)]
        layers = []
        for layer in panel.layers:
            if isinstance(layer, Note):
                layer = clear_of(layer, placed)
                placed.append(layer)
            layers.append(layer)
        panels.append(replace(panel, layers=tuple(layers)))

    return replace(spec, panels=tuple(panels))


def clear_of(note: Note, placed) -> Note:
    """``note`` moved past each placed note that sits at its anchor: up from the bottom half, down from the top."""
    direction = 1.0 if note.y < 0.5 else -1.0
    for other in placed:
        if (other.x, other.y) == (note.x, note.y):
            height = NOTE_LINE * (other.text.count("\n") + 1) + NOTE_PAD
            note = replace(note, y=note.y + direction * height)

    return note


# The colours an overlay's line takes when its own is already the host panel's:
# the palette after its blue, the first line's colour
OVERLAY_COLORS = TAB10[1:]


def distinct_colors(host: FigureSpec, spec: FigureSpec) -> FigureSpec:
    """``spec`` with each line colour its host panel already draws moved to a colour neither panel uses, one per clashing colour.

    Lines and unlabelled markers move. A labelled marker and a fill keep
    their colour: it is their meaning (a start, a goal, an obstacle).
    """
    panels = []
    for host_panel, panel in zip(host.panels, spec.panels):
        used = colors_of(host_panel) | colors_of(panel)
        free = iter(color for color in OVERLAY_COLORS if color not in used)

        # Each clashing colour, in the order the lines draw it, takes its own free
        # colour, and keeps its own when none is left
        moved = {}
        for layer in panel.layers:
            clashes = is_recolored(layer) and layer.color in colors_of(host_panel)
            if clashes and layer.color not in moved:
                moved[layer.color] = next(free, layer.color)
        layers = tuple(
            replace(layer, color=moved[layer.color])
            if is_recolored(layer) and layer.color in moved
            else layer
            for layer in panel.layers
        )
        panels.append(replace(panel, layers=layers))

    return replace(spec, panels=tuple(panels))


def is_recolored(layer) -> bool:
    """Whether an overlay may move ``layer``'s colour: a line, or a marker set without a label."""
    if not isinstance(layer, Trace) or layer.fill:
        return False

    return layer.mode == "lines" or not layer.label


def colors_of(panel) -> set:
    """The colours a panel's layers name; a layer without one names none."""
    return {
        layer.color
        for layer in panel.layers
        if getattr(layer, "color", None) is not None
    }


def has_colorbar(panel) -> bool:
    """Whether a panel holds a layer with a colour bar."""
    return any(getattr(layer, "colorbar", None) is not None for layer in panel.layers)


def grid_of(spec: FigureSpec) -> tuple[int, int]:
    """The ``(n_rows, n_cols)`` of ``spec``'s panel grid, read row by row; no more columns than panels."""
    n = len(spec.panels)
    n_cols = min(spec.n_cols, n)
    n_rows = -(-n // n_cols)

    return n_rows, n_cols


def direction_arrow_spans(n_samples: int, count: int = 3) -> list:
    """The ``(start, end)`` samples of ``count`` direction arrows on a line; none under 3 samples.

    The arrows spread over the middle 70 % of the samples, each 1/50 of them
    long, none past the end.
    """
    if n_samples < 3:
        return []
    span = max(1, n_samples // 50)
    starts = np.linspace(n_samples * 0.15, n_samples * 0.85, count).astype(int)
    starts = np.minimum(starts, n_samples - 1 - span)

    return [(k, k + span) for k in starts]
