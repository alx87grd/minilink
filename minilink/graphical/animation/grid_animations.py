"""A field on a 2-D grid slice animated over its frames, with matplotlib."""

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from minilink.graphical.common.animation_result import show_animation
from minilink.graphical.figure import require_axes

###############################################################################
# Public API
###############################################################################


def animate_heatmap(
    x,
    y,
    frame_of,
    n_frames,
    *,
    title_of,
    cmap,
    vmin=None,
    vmax=None,
    x_label="",
    y_label="",
    interval=60,
    show=None,
    ax=None,
):
    """
    Animate the heatmaps ``frame_of(i)`` (values ``z[j, i]`` at ``(x[i], y[j])``) for ``i < n_frames``.

    The first frame is drawn once, each later frame swaps its values into the
    same mesh and titles the axes ``title_of(i)``; the colours span
    ``[vmin, vmax]`` when given, every frame on one scale. Drawn on a new
    figure, or into ``ax``; ``show=None`` plays it unless it is drawn into
    ``ax``. Returns an
    :class:`~minilink.graphical.common.animation_result.AnimationResult`.
    """
    # A new figure, or the one that holds the given axes
    require_axes(ax)
    if ax is None:
        fig, target = plt.subplots()
    else:
        fig, target = ax.figure, ax

    # The first frame, drawn once, on one colour scale
    mesh = target.pcolormesh(x, y, frame_of(0), shading="gouraud", cmap=cmap)
    if vmin is not None or vmax is not None:
        mesh.set_clim(vmin=vmin, vmax=vmax)
    target.set_xlabel(x_label)
    target.set_ylabel(y_label)
    fig.colorbar(mesh, ax=target)

    # Each frame swaps its values into the same mesh
    def update(i):
        mesh.set_array(frame_of(i).ravel())
        target.set_title(title_of(i))
        return (mesh,)

    ani = animation.FuncAnimation(fig, update, n_frames, interval=interval, blit=False)

    return show_animation(fig, ani, fps=max(1, int(1000 / interval)), show=show, ax=ax)
