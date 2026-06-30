"""
Matplotlib views of a dynamic-programming cost-to-go and policy.

The value-iteration result is a node-indexed field over the state grid. These
helpers reshape it, take a 2-D slice for systems with ``n > 2``, and draw it as
a heatmap, a 3-D surface, or an animation over the recorded sweeps. Matplotlib
is imported lazily so the planning layer stays importable without a backend.
"""

import numpy as np

# Public API


def plot_value(
    grid,
    J,
    *,
    axes=(0, 1),
    anchor=None,
    vmin=0.0,
    vmax=None,
    cmap="YlOrRd",
    ax=None,
    title="Cost-to-go",
    show=True,
):
    """Draw a 2-D heatmap of a node-indexed value field ``J``."""
    return _plot_field(
        grid,
        J,
        axes=axes,
        anchor=anchor,
        vmin=vmin,
        vmax=vmax,
        cmap=cmap,
        ax=ax,
        title=title,
        show=show,
    )


def plot_policy(
    grid,
    pi,
    *,
    axis=0,
    axes=(0, 1),
    anchor=None,
    cmap="bwr",
    ax=None,
    show=True,
):
    """Draw a 2-D heatmap of input axis ``axis`` of a policy ``pi``."""
    u_axis = grid.input_from_policy(pi)[:, axis]
    return _plot_field(
        grid,
        u_axis,
        axes=axes,
        anchor=anchor,
        vmin=grid.u_lb[axis],
        vmax=grid.u_ub[axis],
        cmap=cmap,
        ax=ax,
        title=f"Policy u[{axis}]",
        show=show,
    )


def plot_value_3d(
    grid, J, *, axes=(0, 1), anchor=None, cmap="YlOrRd", title="Cost-to-go", show=True
):
    """Draw a 3-D surface of a node-indexed value field ``J``."""
    import matplotlib.pyplot as plt

    Z = grid.slice_2d(grid.grid_from_array(J), axes[0], axes[1], anchor=anchor)
    x_level, y_level = grid.x_levels[axes[0]], grid.x_levels[axes[1]]
    mesh_x, mesh_y = np.meshgrid(x_level, y_level)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    surf = ax.plot_surface(
        mesh_x, mesh_y, Z.T, cmap=cmap, linewidth=0, antialiased=False
    )
    _label_axes(ax, grid, axes)
    ax.set_zlabel("J")
    ax.set_title(title)
    fig.colorbar(surf, ax=ax, shrink=0.6)
    _maybe_show(plt, show)
    return fig, ax


def animate(
    grid,
    history,
    *,
    kind="value",
    axis=0,
    axes=(0, 1),
    anchor=None,
    interval=60,
    show=True,
):
    """
    Animate a recorded value (``kind="value"``) or policy (``kind="policy"``)
    over the sweeps in ``history`` (the ``(t, J, pi)`` list from a solve).
    """
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    def frame_field(entry):
        _, J, pi = entry
        if kind == "policy":
            return grid.input_from_policy(pi)[:, axis]
        return J

    fig, ax = plt.subplots()
    x_level, y_level = grid.x_levels[axes[0]], grid.x_levels[axes[1]]
    mesh = ax.pcolormesh(
        x_level,
        y_level,
        grid.slice_2d(
            grid.grid_from_array(frame_field(history[0])),
            axes[0],
            axes[1],
            anchor=anchor,
        ).T,
        shading="gouraud",
        cmap="bwr" if kind == "policy" else "YlOrRd",
    )
    _label_axes(ax, grid, axes)
    fig.colorbar(mesh, ax=ax)

    def update(i):
        Z = grid.slice_2d(
            grid.grid_from_array(frame_field(history[i])),
            axes[0],
            axes[1],
            anchor=anchor,
        )
        mesh.set_array(Z.T.ravel())
        ax.set_title(f"sweep {i}  t={history[i][0]:.2f}")
        return (mesh,)

    ani = animation.FuncAnimation(
        fig, update, len(history), interval=interval, blit=False
    )
    _maybe_show(plt, show)
    return ani


# Internal machinery


def _plot_field(grid, values, *, axes, anchor, vmin, vmax, cmap, ax, title, show):
    import matplotlib.pyplot as plt

    Z = grid.slice_2d(grid.grid_from_array(values), axes[0], axes[1], anchor=anchor)
    x_level, y_level = grid.x_levels[axes[0]], grid.x_levels[axes[1]]

    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    mesh = ax.pcolormesh(x_level, y_level, Z.T, shading="gouraud", cmap=cmap)
    mesh.set_clim(vmin=vmin, vmax=vmax)
    fig.colorbar(mesh, ax=ax)
    _label_axes(ax, grid, axes)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    _maybe_show(plt, show)
    return fig, ax


def _label_axes(ax, grid, axes):
    labels = getattr(grid.sys.state, "labels", None)
    units = getattr(grid.sys.state, "units", None)

    def name(i):
        base = labels[i] if labels else f"x[{i}]"
        unit = f" [{units[i]}]" if units and units[i] else ""
        return base + unit

    ax.set_xlabel(name(axes[0]))
    ax.set_ylabel(name(axes[1]))


def _maybe_show(plt, show):
    if show and plt.get_backend().lower() != "agg":
        plt.show()
