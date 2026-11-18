"""Line animations in matplotlib: lines redrawn frame by frame over a fixed frame."""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from minilink.graphical.common.animation_result import show_animation
from minilink.graphical.figure import require_axes
from minilink.graphical.figure_renderers.matplotlib import draw_trace

###############################################################################
# Public API
###############################################################################


def animate_lines(
    frame,
    frame_of,
    n_frames,
    *,
    interval=30,
    repeat=False,
    show=None,
    ax=None,
    figsize=None,
):
    """
    Animate the lines ``frame_of(i)`` for ``i < n_frames`` over the fixed ``frame``.

    ``frame`` is a :class:`~minilink.graphical.figure.Panel` drawn once: its
    limits, labels and layers. ``frame_of(i)`` returns ``(under, over,
    title)``: the :class:`~minilink.graphical.figure.Trace` lines of frame
    ``i`` drawn under the frame's layers and over them, and the axes title.
    Each frame replaces the previous frame's lines. Returns an
    :class:`~minilink.graphical.common.animation_result.AnimationResult`.
    """
    # A new figure, or the one that holds the given axes
    require_axes(ax)
    if ax is None:
        fig, target = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig, target = ax.figure, ax

    # The fixed frame, and the lines of the frame on screen
    frame_axes(target, frame)
    drawn: list = []

    def update(i):
        under, over, title = frame_of(i)
        clear_lines(drawn)
        drawn.extend(draw_lines(target, under, zorder=1))
        drawn.extend(draw_lines(target, over, zorder=3))
        target.set_title(title)
        return drawn

    anim = FuncAnimation(
        fig, update, frames=n_frames, interval=interval, blit=False, repeat=repeat
    )

    return show_animation(fig, anim, fps=max(1, int(1000 / interval)), show=show, ax=ax)


###############################################################################
# Internal machinery
###############################################################################


def frame_axes(ax, frame):
    """Draw a panel's fixed frame on ``ax``: its title, limits, labels and layers, without grid or legend."""
    if frame.title:
        ax.set_title(frame.title)
    ax.set_xlim(*frame.x_lim)
    ax.set_ylim(*frame.y_lim)
    ax.set_xlabel(frame.x_label)
    ax.set_ylabel(frame.y_label)
    for layer in frame.layers:
        draw_trace(ax, layer)


def draw_lines(ax, traces, *, zorder):
    """One matplotlib line per trace, at ``zorder``; the lines, for a later redraw to clear."""
    lines = []
    for trace in traces:
        (line,) = ax.plot(
            trace.x, trace.y, color=trace.color, lw=trace.width, zorder=zorder
        )
        lines.append(line)

    return lines


def clear_lines(lines):
    """Remove the lines of the previous redraw."""
    for line in lines:
        line.remove()
    lines.clear()
