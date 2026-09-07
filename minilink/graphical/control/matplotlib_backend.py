"""Matplotlib renderer for :class:`~minilink.graphical.control.figure_spec.ControlFigure`."""

from __future__ import annotations

import numpy as np

from minilink.graphical.common import PlotResult
from minilink.graphical.control.figure_spec import ControlFigure, Panel
from minilink.graphical.control.style import AXIS_COLOR

_MARKERS = {"x": "x", "o": "o", "+": "+"}


def render_matplotlib(spec: ControlFigure, *, show: bool = True) -> PlotResult:
    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

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
    manager = getattr(fig.canvas, "manager", None)
    if callable(getattr(manager, "set_window_title", None)):
        manager.set_window_title(spec.title)

    for k, (ax, panel) in enumerate(zip(axes, spec.panels)):
        _draw_panel(ax, panel, FONT_SIZE)
        if k == 0:
            ax.set_title(spec.subtitle, fontsize=FONT_SIZE - 1, color="0.35")
    fig.suptitle(spec.title, fontsize=FONT_SIZE + 1)
    fig.tight_layout()

    if show and plt.get_backend().lower() != "agg":
        plt.show(block=is_blocking_needed())

    axes_out = axes[0] if n_panels == 1 else axes
    return PlotResult(
        backend="matplotlib", payload=(fig, axes_out), figure=fig, axes=axes_out
    )


def _draw_panel(ax, panel: Panel, font_size: float) -> None:
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
                _arrows(ax, trace.x, trace.y, trace.color)
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

    if panel.x_log:
        ax.set_xscale("log")
        ax.grid(True, which="both", linestyle="-", alpha=0.35)
        ax.grid(True, which="minor", linestyle=":", alpha=0.25)
    else:
        ax.grid(True, linestyle="-", alpha=0.35)
    if panel.zero_lines:
        ax.axhline(0.0, color=AXIS_COLOR, linewidth=0.8)
        ax.axvline(0.0, color=AXIS_COLOR, linewidth=0.8)
    if panel.x_lim is not None:
        ax.set_xlim(panel.x_lim)
    if panel.y_lim is not None:
        ax.set_ylim(panel.y_lim)
    if panel.y_ticks is not None:
        ax.set_yticks(panel.y_ticks)
    ax.set_xlabel(panel.x_label, fontsize=font_size)
    ax.set_ylabel(panel.y_label, fontsize=font_size)
    ax.tick_params(labelsize=font_size - 1)
    if any(trace.label for trace in panel.traces):
        ax.legend(fontsize=font_size - 1, loc="best")


def _arrows(ax, x, y, color, count: int = 3) -> None:
    """Direction arrows spread along a line."""
    x, y = np.asarray(x), np.asarray(y)
    if x.size < 3:
        return
    span = max(1, x.size // 50)
    for k in np.linspace(x.size * 0.15, x.size * 0.85, count).astype(int):
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
