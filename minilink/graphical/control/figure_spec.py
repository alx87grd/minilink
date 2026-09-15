"""Backend-neutral specification of a control-analysis figure.

One or two panels of line traces, markers, dashed reference lines and text
notes — enough for Bode, pole-zero, root locus, Nyquist and step-response
plots. :mod:`matplotlib_backend` and :mod:`plotly_backend` render the same
spec.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Trace:
    """A line or a marker set; ``hover`` holds one text per point for plotly."""

    x: np.ndarray
    y: np.ndarray
    color: str
    mode: str = "lines"  # "lines" | "markers"
    marker: str = "x"  # "x" | "o" | "+" for markers
    size: float = 8.0
    width: float = 1.5
    dash: str = "solid"  # "solid" | "dash"
    label: str = ""
    hover: tuple[str, ...] = ()
    arrows: bool = False  # direction arrows along a line (Nyquist)


@dataclass(frozen=True)
class RefLine:
    """A dashed reference line across the panel: ``axis`` is ``"x"`` or ``"y"``."""

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
class Panel:
    """One axes: traces, references, notes, and how the axes are drawn."""

    traces: tuple[Trace, ...]
    x_label: str
    y_label: str
    lines: tuple[RefLine, ...] = ()
    notes: tuple[Note, ...] = ()
    x_log: bool = False
    zero_lines: bool = False  # draw the real and imaginary axes (s-plane plots)
    x_lim: tuple[float, float] | None = None
    y_lim: tuple[float, float] | None = None
    y_ticks: tuple[float, ...] | None = None


@dataclass(frozen=True)
class ControlFigure:
    """A titled stack of panels; ``subtitle`` names the channel (From / To)."""

    title: str
    panels: tuple[Panel, ...]
    subtitle: str = ""
    share_x: bool = False
    extra: dict = field(default_factory=dict)
