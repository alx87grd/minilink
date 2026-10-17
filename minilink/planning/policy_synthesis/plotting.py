"""Matplotlib views of a dynamic-programming cost-to-go and policy."""

import numpy as np

# The value-iteration result is a node-indexed field over the state grid: these
# views reshape it, take a 2-D slice when n > 2, and draw it as a heatmap, a 3-D
# surface or an animation over the recorded sweeps. Matplotlib is imported inside
# each view, so the planning layer stays importable without a backend.


###############################################################################
# Public API
###############################################################################


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
    require_single_slice(trajectory, axes)

    # Several slices: one heatmap per pair, without the overlay
    if is_batch_axes(axes):
        return [
            plot_cost2go(
                result,
                jmax=jmax,
                axes=pair,
                anchor=anchor,
                vmin=vmin,
                show_3d=show_3d,
                trajectory=None,
                cmap=cmap,
                title=title,
                show=show,
            )
            for pair in axes
        ]

    require_flat_overlay(show_3d, trajectory)

    grid = result.grid
    J = result.J
    defer_show = trajectory is not None

    # The heatmap, shown now unless the surface or the overlay comes after it
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

    # The 3-D surface of J, clipped to 0 ≤ J (≤ jmax when given)
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

    # The closed-loop trajectory over the heatmap, then the deferred show
    if trajectory is not None:
        overlay_trajectory(ax, trajectory, axes, overlay_labels)
        if show and not show_3d:
            import matplotlib.pyplot as plt

            maybe_show(plt, show)

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
    # Grid form: the raw table on its grid
    if pi is not None:
        return plot_policy_field(
            grid_or_result,
            pi,
            axis=axis,
            axes=axes,
            anchor=anchor,
            cmap=cmap,
            ax=ax,
            show=show,
        )

    # Result form
    result = grid_or_result
    require_single_slice(trajectory, axes)

    # Several slices: one heatmap per pair, each with its own input axis
    if is_batch_axes(axes):
        axis_list = axis_list_of(axis, axes)
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

    # The heatmap, shown now unless the overlay comes after it
    fig, ax = plot_policy_field(
        result.grid,
        result.pi,
        axis=axis,
        axes=axes,
        anchor=anchor,
        cmap=cmap,
        ax=ax,
        show=show and trajectory is None,
    )

    # The closed-loop trajectory over the heatmap, then the deferred show
    if trajectory is not None:
        overlay_trajectory(ax, trajectory, axes, overlay_labels)
        if show:
            import matplotlib.pyplot as plt

            maybe_show(plt, show)

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
    """
    Animate cost-to-go sweeps recorded on ``result.history``.

    The colours span ``[0, jmax]``; without ``jmax``, the range of the last
    sweep's slice.
    """
    require_history(result)

    return animate(
        result.grid,
        result.history,
        kind="value",
        axes=axes,
        anchor=anchor,
        vmax=jmax,
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
    require_history(result)

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
    return plot_field(
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


def plot_value_3d(
    grid, J, *, axes=(0, 1), anchor=None, cmap="YlOrRd", title="Cost-to-go", show=True
):
    """Draw a 3-D surface of a node-indexed value field ``J``."""
    import matplotlib.pyplot as plt

    # The 2-D slice of the field, over the mesh of its two state levels
    Z = grid.slice_2d(grid.grid_from_array(J), axes[0], axes[1], anchor=anchor)
    x_level = grid.x_levels[axes[0]]
    y_level = grid.x_levels[axes[1]]
    mesh_x, mesh_y = np.meshgrid(x_level, y_level)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    surf = ax.plot_surface(
        mesh_x, mesh_y, Z.T, cmap=cmap, linewidth=0, antialiased=False
    )
    label_axes(ax, grid, axes)
    ax.set_zlabel("J")
    ax.set_title(title)
    fig.colorbar(surf, ax=ax, shrink=0.6)

    maybe_show(plt, show)

    return fig, ax


def animate(
    grid,
    history,
    *,
    kind="value",
    axis=0,
    axes=(0, 1),
    anchor=None,
    vmax=None,
    interval=60,
    show=True,
    save_path=None,
):
    """
    Animate a recorded value or policy field over the sweeps of ``history``.

    ``kind`` is ``"value"`` (the cost-to-go ``J``) or ``"policy"`` (input
    ``axis`` of the greedy law); ``history`` is the ``(t, J, pi)`` list from a
    solve. A value is coloured on ``[0, vmax]``, by default up to the largest
    value of the last sweep's slice.
    """
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    def frame_field(entry):
        _, J, pi = entry
        if kind == "policy":
            return grid.input_from_policy(pi)[:, axis]
        return J

    # The first sweep's slice, drawn once
    fig, ax = plt.subplots()
    x_level = grid.x_levels[axes[0]]
    y_level = grid.x_levels[axes[1]]
    Z = grid.slice_2d(
        grid.grid_from_array(frame_field(history[0])),
        axes[0],
        axes[1],
        anchor=anchor,
    )
    cmap = "bwr" if kind == "policy" else "YlOrRd"
    mesh = ax.pcolormesh(x_level, y_level, Z.T, shading="gouraud", cmap=cmap)

    # A value's colours span [0, vmax], the last sweep's largest value by default,
    # so every frame shares one scale
    if kind == "value":
        mesh.set_clim(vmin=0.0, vmax=last_sweep_max(grid, history, axes, anchor, vmax))

    label_axes(ax, grid, axes)
    fig.colorbar(mesh, ax=ax)

    # Each frame swaps sweep i's slice into the same mesh
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

    maybe_show(plt, show)

    return ani


def cost_scale_limit(price):
    """
    Default top of the cost-to-go colour scale: the price of leaving the grid.

    States that must leave saturate at that price, so clipping there keeps the
    feasible states readable. A callable ``price(x, t)`` has no single level and
    an infinite one is no limit: both leave the scale to Matplotlib (``None``).
    """
    if price is None or callable(price) or not np.isfinite(price):
        return None

    return float(price)


def get_controller(result, *, interpolation="linear"):
    """Build a :class:`~minilink.planning.policy_synthesis.lookup_policy.LookupTableController`."""
    # Lazy import: the lookup law loads on first use, not with the plots
    from minilink.planning.policy_synthesis.lookup_policy import LookupTableController

    return LookupTableController(result.grid, result.pi, interpolation=interpolation)


###############################################################################
# Internal machinery
###############################################################################


def require_single_slice(trajectory, axes):
    """Refuse a trajectory overlay on a batch of slices."""
    if trajectory is not None and is_batch_axes(axes):
        raise ValueError("trajectory overlay requires a single-slice axes=(i, j) call")


def is_batch_axes(axes):
    """``axes`` lists several ``(i, j)`` pairs rather than being one."""
    return len(axes) > 0 and isinstance(axes[0], (tuple, list))


def require_flat_overlay(show_3d, trajectory):
    """Refuse a trajectory overlay together with the 3-D surface."""
    if show_3d and trajectory is not None:
        raise ValueError("show_3d and trajectory cannot be used together")


def overlay_trajectory(ax, trajectory, axes, labels):
    """Draw a trajectory's path, start and end on the ``axes`` slice."""
    x = trajectory.x
    ax.plot(x[axes[0]], x[axes[1]], "k-", linewidth=2, label=labels[0])
    ax.plot(x[axes[0], 0], x[axes[1], 0], "go", label=labels[1])
    ax.plot(x[axes[0], -1], x[axes[1], -1], "r*", markersize=12, label=labels[2])
    ax.legend(loc="upper right", fontsize=8)


def maybe_show(plt, show):
    """Show the figures, unless asked not to or on the non-interactive Agg backend."""
    if show and plt.get_backend().lower() != "agg":
        plt.show()


def plot_policy_field(grid, pi, *, axis, axes, anchor, cmap, ax, show):
    """Heatmap of input ``axis`` of a policy table, scaled to that input's bounds."""
    u_axis = grid.input_from_policy(pi)[:, axis]

    return plot_field(
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


def axis_list_of(axis, axes):
    """One input axis per slice: the list given, or ``axis`` repeated."""
    if isinstance(axis, (list, tuple)):
        if len(axis) != len(axes):
            raise ValueError("axis list must match axes list length")
        return axis

    return [axis] * len(axes)


def last_sweep_max(grid, history, axes, anchor, vmax):
    """The colour limit ``vmax`` as given, else the largest value of the last sweep's slice."""
    if vmax is not None:
        return vmax

    _, J_last, _ = history[-1]
    Z = grid.slice_2d(grid.grid_from_array(J_last), axes[0], axes[1], anchor=anchor)

    return float(np.max(Z))


def require_history(result):
    """Refuse an animation of a result that recorded no sweeps."""
    if not result.history:
        raise ValueError(
            "result.history is empty; re-run with record_history=True to animate"
        )


def plot_field(grid, values, *, axes, anchor, vmin, vmax, cmap, ax, title, show):
    """Heatmap of a node-indexed field on a 2-D slice of the grid."""
    import matplotlib.pyplot as plt

    # The 2-D slice of the field, over its two state levels
    Z = grid.slice_2d(grid.grid_from_array(values), axes[0], axes[1], anchor=anchor)
    x_level = grid.x_levels[axes[0]]
    y_level = grid.x_levels[axes[1]]

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    mesh = ax.pcolormesh(x_level, y_level, Z.T, shading="gouraud", cmap=cmap)
    mesh.set_clim(vmin=vmin, vmax=vmax)
    fig.colorbar(mesh, ax=ax)
    label_axes(ax, grid, axes)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    maybe_show(plt, show)

    return fig, ax


def label_axes(ax, grid, axes):
    """Name the two sliced state axes by their labels and units, ``x[i]`` without labels."""
    labels = getattr(grid.sys.state, "labels", None)
    units = getattr(grid.sys.state, "units", None)

    def name(i):
        base = labels[i] if labels else f"x[{i}]"
        unit = f" [{units[i]}]" if units and units[i] else ""
        return base + unit

    ax.set_xlabel(name(axes[0]))
    ax.set_ylabel(name(axes[1]))
