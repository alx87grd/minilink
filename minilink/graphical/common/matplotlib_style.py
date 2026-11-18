"""Shared matplotlib layout and typography defaults."""

# 4:3 animation canvas, small enough for a laptop screen
FIGSIZE_ANIMATION = (6, 4.5)
# The small figure of one panel, 4:3
FIGSIZE_BASE = (4, 3)

DPI_FIGURE = 150
DPI_EXPORT = 250

# Readable in a notebook; lower it when print density matters
FONT_SIZE = 10

# A stack of signal panels grows with its number of rows, so each trace stays
# readable. In Jupyter and Colab that height is uncapped (scrollable output); in
# scripts and terminal IPython it is capped by the ``*_MAX_FIG_HEIGHT_POPUP``
# constants (see
# :func:`minilink.graphical.common.environment.allow_tall_stacked_figures`).

# Stacked time-signal plots: width × (row height × number of subplots).
TRAJECTORY_FIG_WIDTH = 6.0
TRAJECTORY_ROW_HEIGHT = 1.35
# Cap total figure height (inches) for console / pop-up windows; no cap in notebooks.
TRAJECTORY_MAX_FIG_HEIGHT_POPUP = 8.0

# Stacked specialized signal plots.
SIGNAL_PLOT_FIG_WIDTH = 6.0
SIGNAL_PLOT_ROW_HEIGHT = 1.5
SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP = 8.0


###############################################################################
# Public API
###############################################################################


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


def style_animation_axes(ax, *, is_3d: bool) -> None:
    """Tick labels, grid, and axis label sizes for animation/static kinematic views."""
    ax.tick_params(axis="both", which="both", labelsize=FONT_SIZE)
    ax.grid(True)
    ax.xaxis.label.set_fontsize(FONT_SIZE)
    ax.yaxis.label.set_fontsize(FONT_SIZE)
    if is_3d:
        ax.zaxis.label.set_fontsize(FONT_SIZE)
