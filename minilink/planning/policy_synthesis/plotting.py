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
    save_path=None,
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
    if save_path is not None:
        ani.save(save_path, writer="pillow", fps=max(1, int(1000 / interval)))
    _maybe_show(plt, show)
    return ani


def plot_cost2go(
    result,
    *,
    jmax=None,
    axes=(0, 1),
    anchor=None,
    vmin=0.0,
    show_3d=False,
    trajectory=None,
    overlay_labels=("closed loop", "start", "end"),
    cmap="YlOrRd",
    ax=None,
    title="Cost-to-go",
    show=True,
):
    """
    Draw cost-to-go heatmap(s) from a :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingResult`.

    ``axes`` may be ``(i, j)`` for one slice or a list of pairs for several.
    """
    if trajectory is not None and _is_batch_axes(axes):
        raise ValueError("trajectory overlay requires a single-slice axes=(i, j) call")
    if _is_batch_axes(axes):
        return [
            plot_cost2go(
                result,
                jmax=jmax,
                axes=pair,
                anchor=anchor,
                vmin=vmin,
                show_3d=False,
                trajectory=None,
                cmap=cmap,
                title=title,
                show=show,
            )
            for pair in axes
        ]
    if show_3d and trajectory is not None:
        raise ValueError("show_3d and trajectory cannot be used together")
    grid, J = result.grid, result.J
    defer_show = trajectory is not None
    fig, ax = plot_value(
        grid,
        J,
        axes=axes,
        anchor=anchor,
        vmin=vmin,
        vmax=jmax,
        cmap=cmap,
        ax=ax,
        title=title,
        show=show and not show_3d and not defer_show,
    )
    if show_3d:
        J_plot = J if jmax is None else np.clip(J, 0.0, jmax)
        plot_value_3d(
            grid,
            np.clip(J_plot, 0.0, np.inf),
            axes=axes,
            anchor=anchor,
            cmap=cmap,
            title=title,
            show=show,
        )
    if trajectory is not None:
        _overlay_trajectory(ax, trajectory, axes, overlay_labels)
        if show and not show_3d:
            import matplotlib.pyplot as plt

            _maybe_show(plt, show)
    return fig, ax


def plot_policy(
    grid_or_result,
    pi=None,
    *,
    axis=0,
    axes=(0, 1),
    anchor=None,
    trajectory=None,
    overlay_labels=("closed loop", "start", "end"),
    cmap="bwr",
    ax=None,
    show=True,
):
    """
    Draw a policy heatmap on the state grid.

    Grid form: ``plot_policy(grid, pi, ...)`` for a raw policy table.
    Result form: ``plot_policy(result, ...)`` from value iteration.

    ``axes`` may be ``(i, j)`` or a list of pairs; ``axis`` may be an int or a
    list aligned with batch ``axes``.
    """
    if pi is not None:
        return _plot_policy_field(
            grid_or_result,
            pi,
            axis=axis,
            axes=axes,
            anchor=anchor,
            cmap=cmap,
            ax=ax,
            show=show,
        )
    result = grid_or_result
    if trajectory is not None and _is_batch_axes(axes):
        raise ValueError("trajectory overlay requires a single-slice axes=(i, j) call")
    if _is_batch_axes(axes):
        if isinstance(axis, (list, tuple)):
            if len(axis) != len(axes):
                raise ValueError("axis list must match axes list length")
            axis_list = axis
        else:
            axis_list = [axis] * len(axes)
        return [
            plot_policy(
                result,
                axis=axis_k,
                axes=axes_k,
                anchor=anchor,
                trajectory=None,
                cmap=cmap,
                show=show,
            )
            for axes_k, axis_k in zip(axes, axis_list)
        ]
    fig, ax = _plot_policy_field(
        result.grid,
        result.pi,
        axis=axis,
        axes=axes,
        anchor=anchor,
        cmap=cmap,
        ax=ax,
        show=show and trajectory is None,
    )
    if trajectory is not None:
        _overlay_trajectory(ax, trajectory, axes, overlay_labels)
        if show:
            import matplotlib.pyplot as plt

            _maybe_show(plt, show)
    return fig, ax


def animate_cost2go(
    result,
    *,
    axes=(0, 1),
    jmax=None,
    anchor=None,
    interval=60,
    save_path=None,
    show=True,
):
    """Animate cost-to-go sweeps recorded on ``result.history``."""
    _require_history(result)
    return animate(
        result.grid,
        result.history,
        kind="value",
        axes=axes,
        anchor=anchor,
        interval=interval,
        show=show,
        save_path=save_path,
    )


def animate_policy(
    result,
    *,
    axis=0,
    axes=(0, 1),
    anchor=None,
    interval=60,
    save_path=None,
    show=True,
):
    """Animate policy sweeps recorded on ``result.history``."""
    _require_history(result)
    return animate(
        result.grid,
        result.history,
        kind="policy",
        axis=axis,
        axes=axes,
        anchor=anchor,
        interval=interval,
        show=show,
        save_path=save_path,
    )


def get_controller(result, *, interpolation="linear"):
    """Build a :class:`~minilink.planning.policy_synthesis.lookup_policy.LookupTableController`."""
    from minilink.planning.policy_synthesis.lookup_policy import LookupTableController

    return LookupTableController(result.grid, result.pi, interpolation=interpolation)


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


# Internal machinery


def _plot_policy_field(grid, pi, *, axis, axes, anchor, cmap, ax, show):
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


def _is_batch_axes(axes):
    return len(axes) > 0 and isinstance(axes[0], (tuple, list))


def _require_history(result):
    if not result.history:
        raise ValueError(
            "result.history is empty; re-run with record_history=True to animate"
        )


def _overlay_trajectory(ax, trajectory, axes, labels):
    x = trajectory.x
    ax.plot(x[axes[0]], x[axes[1]], "k-", linewidth=2, label=labels[0])
    ax.plot(x[axes[0], 0], x[axes[1], 0], "go", label=labels[1])
    ax.plot(x[axes[0], -1], x[axes[1], -1], "r*", markersize=12, label=labels[2])
    ax.legend(loc="upper right", fontsize=8)


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
