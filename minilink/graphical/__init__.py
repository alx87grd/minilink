"""Graphics: the plots, animations and diagrams of systems and their trajectories.

A plot builds a :class:`FigureSpec`, panels of layers as plain data, and
:func:`draw` hands it to a renderer (matplotlib or Plotly); every plot returns
a :class:`PlotResult`, every animation an :class:`AnimationResult`.
"""

from minilink.graphical.common.animation_result import AnimationResult
from minilink.graphical.common.plot_result import PlotResult
from minilink.graphical.figure import (
    Arrows,
    Contour,
    FigureSpec,
    Heatmap,
    Note,
    Panel,
    RefLine,
    Surface,
    Trace,
    draw,
)

__all__ = [
    "AnimationResult",
    "Arrows",
    "Contour",
    "FigureSpec",
    "Heatmap",
    "Note",
    "Panel",
    "PlotResult",
    "RefLine",
    "Surface",
    "Trace",
    "draw",
]
