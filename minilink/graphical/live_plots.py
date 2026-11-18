"""Live plots in matplotlib: figures redrawn while a planner or a solver runs."""

from minilink.graphical.animation.line_animations import (
    clear_lines,
    draw_lines,
    frame_axes,
)

###############################################################################
# Public API
###############################################################################


def redraw_lines(plot, frame, under, over, title) -> None:
    """
    Redraw the lines ``under`` and ``over`` the fixed ``frame`` on ``plot``'s axes, a new figure on the first call.

    ``plot`` holds the live figure between redraws: its ``ax`` and ``fig``,
    whether it is ``framed``, the ``under_lines`` and ``over_lines`` of
    the latest redraw, its ``title`` text and its ``pause``. ``frame`` is a
    :class:`~minilink.graphical.figure.Panel` drawn on the first call;
    ``under`` and ``over`` are :class:`~minilink.graphical.figure.Trace`
    lines drawn under and over its layers.
    """
    # Lazy import: matplotlib loads with the first redraw
    import matplotlib.pyplot as plt

    # A new figure on the first update, or the one that holds the given axes
    if plot.ax is None:
        plot.fig, plot.ax = plt.subplots(figsize=(7.0, 6.0), frameon=True)
    else:
        plot.fig = plot.ax.figure

    # The frame, set once: the limits, labels and fixed layers stay
    if not plot.framed:
        frame_axes(plot.ax, frame)
        plot.framed = True

    # The lines, redrawn in place of the previous ones
    clear_lines(plot.under_lines)
    clear_lines(plot.over_lines)
    plot.under_lines.extend(draw_lines(plot.ax, under, zorder=1))
    plot.over_lines.extend(draw_lines(plot.ax, over, zorder=3))

    # The title reports the progress
    if plot.title is None:
        plot.title = plot.ax.set_title(title)
    else:
        plot.title.set_text(title)

    pause_figures(plot.pause)


def pause_figures(seconds):
    """Let the open matplotlib windows redraw for ``seconds``; nothing for 0."""
    # Lazy import: matplotlib loads with the first live figure
    import matplotlib.pyplot as plt

    if seconds > 0.0:
        plt.pause(seconds)
