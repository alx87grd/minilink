"""A figure as plain data: panels of layers that any renderer draws."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np

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
    marker: str = "o"  # "o" | "o-open" | "x" | "+" | "*"
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


def draw(spec: FigureSpec, *, renderer="matplotlib", show=None, ax=None):
    """
    Draw ``spec`` with ``renderer`` and return its :class:`PlotResult`.

    ``ax`` is where to draw: a matplotlib ``Axes`` (a list of them for several
    panels), or a :class:`PlotResult`, whose panels receive these layers.
    ``show=None`` shows the figure unless it is drawn into ``ax``.
    ``renderer=None`` draws nothing: the record holds the description alone.
    """
    # Lazy import: the record and the renderers load on first use
    from minilink.graphical.common.plot_result import PlotResult

    if show is None:
        show = ax is None

    # The description alone
    if renderer is None:
        return PlotResult(renderer=None, figure=None, axes=None, spec=spec)

    # Into a drawn figure: its panels take the layers, on its renderer
    if isinstance(ax, PlotResult):
        require_same_renderer(renderer, ax.renderer)
        module = renderer_module(ax.renderer)
        return module.draw_into(ax, spec, show=show)

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
    import importlib

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


def as_array(values) -> np.ndarray:
    """A layer's values as a float array."""
    return np.asarray(values, dtype=float)
