"""A field on a 2-D grid slice animated over its frames, with matplotlib."""

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from minilink.graphical.common.animation_result import animation_result_of

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
    show=True,
):
    """
    Animate the heatmaps ``frame_of(i)`` (values ``z[j, i]`` at ``(x[i], y[j])``) for ``i < n_frames``.

    The first frame is drawn once, each later frame swaps its values into the
    same mesh and titles the axes ``title_of(i)``; the colours span
    ``[vmin, vmax]`` when given, every frame on one scale. Returns an
    :class:`~minilink.graphical.common.animation_result.AnimationResult`.
    """
    # The first frame, drawn once, on one colour scale
    fig, ax = plt.subplots()
    mesh = ax.pcolormesh(x, y, frame_of(0), shading="gouraud", cmap=cmap)
    if vmin is not None or vmax is not None:
        mesh.set_clim(vmin=vmin, vmax=vmax)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    fig.colorbar(mesh, ax=ax)

    # Each frame swaps its values into the same mesh
    def update(i):
        mesh.set_array(frame_of(i).ravel())
        ax.set_title(title_of(i))
        return (mesh,)

    ani = animation.FuncAnimation(fig, update, n_frames, interval=interval, blit=False)

    return animation_result_of(fig, ani, fps=max(1, int(1000 / interval)), show=show)
