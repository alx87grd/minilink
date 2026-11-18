"""A figure as plain data: panels of layers that any renderer draws."""

from __future__ import annotations

import importlib
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from minilink.graphical.common.plot_result import PlotResult

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
    legend: str | None = "best"


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
    ``color`` draws every level in one colour, ``cmap`` colours them by value.
    """

    x: Any
    y: Any
    z: Any
    levels: tuple[float, ...] | None = None
    filled: bool = False
    color: str | None = None
    cmap: str | None = None
    width: float = 1.0
    dash: str = "solid"
    alpha: float = 1.0
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
    label: str = ""


def draw(spec: FigureSpec, *, renderer="matplotlib", show=None, ax=None):
    """
    Draw ``spec`` with ``renderer`` and return its :class:`PlotResult`.

    ``ax`` is where to draw: a matplotlib ``Axes`` (a list of them for several
    panels), or a :class:`PlotResult`, whose panels receive these layers; a
    line in a colour its host panel already draws takes the next free one.
    ``show=None`` shows the figure unless it is drawn into ``ax``.
    ``renderer=None`` draws nothing: the record holds the description alone.
    """
    if show is None:
        show = ax is None

    # The description alone
    if renderer is None:
        return PlotResult(renderer=None, figure=None, axes=None, spec=spec)

    # Into a drawn figure: its panels take the layers, on its renderer
    if isinstance(ax, PlotResult):
        require_same_renderer(renderer, ax.renderer)
        require_composable(ax.spec, spec)
        module = renderer_module(ax.renderer)
        return module.draw_into(ax, distinct_colours(ax.spec, spec), show=show)

    module = renderer_module(renderer)
    return module.draw_figure(spec, show=show, ax=ax)


def overlay(host: FigureSpec, layers_by_panel) -> FigureSpec:
    """``host`` with ``layers_by_panel[k]`` added to its panel ``k``, a new description."""
    panels = tuple(
        replace(panel, layers=panel.layers + tuple(layers))
        for panel, layers in zip(host.panels, layers_by_panel)
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


def require_renderer(renderer):
    """Refuse a renderer the figures do not draw with."""
    if renderer not in RENDERER_MODULES:
        names = ", ".join(repr(name) for name in RENDERER_MODULES)
        raise ValueError(f"renderer must be one of {names} or None, got {renderer!r}")


def require_same_renderer(renderer, host_renderer):
    """Refuse drawing into a figure with another renderer than its own."""
    if renderer != host_renderer and renderer != "matplotlib":
        raise ValueError(
            f"the host figure is drawn with {host_renderer!r}; "
            f"drop renderer= or pass renderer={host_renderer!r}"
        )


def require_composable(host: FigureSpec, spec: FigureSpec):
    """Refuse an overlay its host cannot take: another panel count, 3-D into 2-D, a second colour bar."""
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
        if has_colorbar(host_panel) and has_colorbar(panel):
            raise ValueError(
                "the host panel already has a colour bar; draw the overlay without one"
            )


# The colours an overlay's line takes when its own is already the host panel's
OVERLAY_COLOURS = (
    "#ff7f0e",
    "#2ca02c",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#17becf",
    "#bcbd22",
)


def distinct_colours(host: FigureSpec, spec: FigureSpec) -> FigureSpec:
    """``spec`` with each line whose colour its host panel already draws in the next colour that panel does not use."""
    panels = []
    for host_panel, panel in zip(host.panels, spec.panels):
        used = {getattr(layer, "color", None) for layer in host_panel.layers}
        free = [colour for colour in OVERLAY_COLOURS if colour not in used]
        layers = []
        for layer in panel.layers:
            if isinstance(layer, Trace) and layer.color in used and free:
                layer = replace(layer, color=free[0])
            layers.append(layer)
        panels.append(replace(panel, layers=tuple(layers)))

    return replace(spec, panels=tuple(panels))


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
