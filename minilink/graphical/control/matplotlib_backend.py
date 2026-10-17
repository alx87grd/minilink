"""Control figures in matplotlib: one subplot per panel of a ControlFigure."""

from __future__ import annotations

import numpy as np

from minilink.graphical.common import PlotResult
from minilink.graphical.control.figure_spec import ControlFigure, Panel
from minilink.graphical.control.style import AXIS_COLOR

# The spec's marker names in matplotlib's spelling; an unknown one draws as "x"
_MARKERS = {"x": "x", "o": "o", "+": "+"}


###############################################################################
# Public API
###############################################################################


def render_matplotlib(spec: ControlFigure, *, show: bool = True) -> PlotResult:
    """Draw ``spec`` with matplotlib, one subplot per panel."""
    import matplotlib
    import matplotlib.pyplot as plt

    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
    )

    # TrueType (Type 42) fonts in PDF and PostScript exports: their text stays editable
    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    # One subplot per panel: the base height for one panel, 0.8 of it per panel
    # for a stack
    n_panels = len(spec.panels)
    width, height = FIGSIZE_BASE
    fig, axes = plt.subplots(
        n_panels,
        1,
        figsize=(width, height * (1.0 if n_panels == 1 else 0.8 * n_panels)),
        dpi=DPI_FIGURE,
        sharex=spec.share_x,
        frameon=True,
        squeeze=False,
    )
    axes = [row[0] for row in axes]

    # The window's title, when the canvas has a window
    manager = getattr(fig.canvas, "manager", None)
    if callable(getattr(manager, "set_window_title", None)):
        manager.set_window_title(spec.title)

    # The panels, the subtitle above the first one, the title above them all
    for k, (ax, panel) in enumerate(zip(axes, spec.panels)):
        draw_panel(ax, panel, FONT_SIZE)
        if k == 0:
            ax.set_title(spec.subtitle, fontsize=FONT_SIZE - 1, color="0.35")
    fig.suptitle(spec.title, fontsize=FONT_SIZE + 1)
    fig.tight_layout()

    # A window, unless the backend draws off screen
    if show and plt.get_backend().lower() != "agg":
        plt.show(block=is_blocking_needed())

    # One axes for a single panel, the list of them for a stack
    axes_out = axes[0] if n_panels == 1 else axes

    return PlotResult(
        backend="matplotlib", payload=(fig, axes_out), figure=fig, axes=axes_out
    )


###############################################################################
# Internal machinery
###############################################################################


def draw_panel(ax, panel: Panel, font_size: float) -> None:
    """Draw one panel on ``ax``: its traces, reference lines and notes, then its axes."""
    # The traces: marker sets (an open circle for "o"), or lines with their arrows
    for trace in panel.traces:
        if trace.mode == "markers":
            ax.plot(
                trace.x,
                trace.y,
                linestyle="none",
                marker=_MARKERS.get(trace.marker, "x"),
                markersize=trace.size,
                markeredgewidth=trace.width,
                markerfacecolor="none" if trace.marker == "o" else trace.color,
                color=trace.color,
                label=trace.label or None,
            )
        else:
            ax.plot(
                trace.x,
                trace.y,
                color=trace.color,
                linewidth=trace.width,
                linestyle="--" if trace.dash == "dash" else "-",
                label=trace.label or None,
            )
            if trace.arrows:
                draw_arrows(ax, trace.x, trace.y, trace.color)

    # The reference lines: vertical at x = value, or horizontal at y = value
    for line in panel.lines:
        if line.axis == "x":
            ax.axvline(
                line.value,
                color=line.color,
                linestyle="--" if line.dash == "dash" else "-",
                linewidth=0.8,
            )
        else:
            ax.axhline(
                line.value,
                color=line.color,
                linestyle="--" if line.dash == "dash" else "-",
                linewidth=0.8,
            )

    # The notes, boxed, anchored at a fraction of the panel; a note in the lower
    # half grows upward, one in the upper half downward
    for note in panel.notes:
        ax.text(
            note.x,
            note.y,
            note.text,
            transform=ax.transAxes,
            fontsize=font_size - 1,
            va="bottom" if note.y < 0.5 else "top",
            bbox={
                "boxstyle": "round",
                "facecolor": "white",
                "alpha": 0.8,
                "edgecolor": "0.7",
            },
        )

    # The grid: a log frequency axis also gets its minor lines, dotted
    if panel.x_log:
        ax.set_xscale("log")
        ax.grid(True, which="both", linestyle="-", alpha=0.35)
        ax.grid(True, which="minor", linestyle=":", alpha=0.25)
    else:
        ax.grid(True, linestyle="-", alpha=0.35)

    # The real and imaginary axes of an s-plane plot
    if panel.zero_lines:
        ax.axhline(0.0, color=AXIS_COLOR, linewidth=0.8)
        ax.axvline(0.0, color=AXIS_COLOR, linewidth=0.8)

    # The limits and ticks the spec fixes
    if panel.x_lim is not None:
        ax.set_xlim(panel.x_lim)
    if panel.y_lim is not None:
        ax.set_ylim(panel.y_lim)
    if panel.y_ticks is not None:
        ax.set_yticks(panel.y_ticks)

    # The labels, and a legend when a trace carries a label
    ax.set_xlabel(panel.x_label, fontsize=font_size)
    ax.set_ylabel(panel.y_label, fontsize=font_size)
    ax.tick_params(labelsize=font_size - 1)
    if any(trace.label for trace in panel.traces):
        ax.legend(fontsize=font_size - 1, loc="best")


def draw_arrows(ax, x, y, color, count: int = 3) -> None:
    """Direction arrows spread along a line."""
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
        ax.annotate(
            "",
            xy=(x[k + span], y[k + span]),
            xytext=(x[k], y[k]),
            arrowprops={
                "arrowstyle": "-|>",
                "color": color,
                "lw": 1.5,
                "mutation_scale": 14,
            },
        )
