"""
Shared matplotlib layout and typography defaults (Pyro-aligned where sensible).

**Figure size vs Pyro**

- Pyro's ``TrajectoryPlotter`` uses one small ``figsize`` (e.g. ``(4, 3)`` inches)
  for the *entire* stack of subplots, so the canvas does not grow with signal count
  (subplots become cramped instead).
- Minilink grows stacked plot height with the number of rows so traces stay readable.
  In Jupyter/Colab that height is uncapped (scrollable output); in scripts and terminal
  IPython it is capped by :data:`TRAJECTORY_MAX_FIG_HEIGHT_POPUP` /
  :data:`SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP` (see :func:`minilink.graphical.environment.allow_tall_stacked_figures`).
  Tune widths, row heights, and those caps as needed.

Pyro uses ``default_fontsize = 5`` for dense print figures; Minilink uses a larger
default for notebook readability. Adjust :data:`FONT_SIZE` if you need print density.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from matplotlib.axes import Axes

# 4:3 animation canvas (1.5× Pyro's (4, 3) inches); smaller than (8, 6) for laptops.
FIGSIZE_ANIMATION = (6, 4.5)
# Baseline small figure size (same aspect as Pyro's (4, 3)).
FIGSIZE_BASE = (4, 3)

DPI_FIGURE = 150
DPI_EXPORT = 250
FONT_SIZE = 10

# Stacked ``plot_trajectory``: width × (row height × number of subplots).
TRAJECTORY_FIG_WIDTH = 6.0
TRAJECTORY_ROW_HEIGHT = 1.35
# Cap total figure height (inches) for console / pop-up windows; no cap in notebooks.
TRAJECTORY_MAX_FIG_HEIGHT_POPUP = 8.0

# Stacked ``plot_signals``.
SIGNAL_PLOT_FIG_WIDTH = 6.0
SIGNAL_PLOT_ROW_HEIGHT = 1.5
SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP = 8.0


def trajectory_stack_figsize(
    n_rows: int,
    *,
    allow_tall: bool,
) -> tuple[float, float]:
    """Return ``(width, height)`` for stacked trajectory subplots."""
    w = TRAJECTORY_FIG_WIDTH
    h = TRAJECTORY_ROW_HEIGHT * float(n_rows)
    if not allow_tall:
        h = min(h, TRAJECTORY_MAX_FIG_HEIGHT_POPUP)
    return (w, h)


def signal_stack_figsize(
    n_rows: int,
    *,
    allow_tall: bool,
) -> tuple[float, float]:
    """Return ``(width, height)`` for stacked signal subplots."""
    w = SIGNAL_PLOT_FIG_WIDTH
    h = SIGNAL_PLOT_ROW_HEIGHT * float(n_rows)
    if not allow_tall:
        h = min(h, SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP)
    return (w, h)


def style_animation_axes(ax: Axes, *, is_3d: bool) -> None:
    """Tick labels, grid, and axis label sizes for animation/static kinematic views."""
    ax.tick_params(axis="both", which="both", labelsize=FONT_SIZE)
    ax.grid(True)
    ax.xaxis.label.set_fontsize(FONT_SIZE)
    ax.yaxis.label.set_fontsize(FONT_SIZE)
    if is_3d:
        ax.zaxis.label.set_fontsize(FONT_SIZE)


def style_trajectory_subplot(ax: Axes) -> None:
    """Grid and ticks for stacked trajectory / signal plots."""
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.tick_params(labelsize=FONT_SIZE)
