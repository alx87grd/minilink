"""Matplotlib views of a dynamic-programming cost-to-go and policy."""

import numpy as np

from minilink.graphical.figure import FigureSpec, Heatmap, Panel, Trace, draw
from minilink.planning.plotting import maybe_show

# The value-iteration result is a node-indexed field over the state grid: these
# views reshape it, take a 2-D slice when n > 2, and draw it as a heatmap, a 3-D
# surface or an animation over the recorded sweeps. Matplotlib is imported inside
# each view, so the planning layer stays importable without a backend.


###############################################################################
# Public API
###############################################################################


def plot_cost_to_go(
    grid,
    J,
    *,
    jmax=None,
    x_axis=0,
    y_axis=1,
    anchor=None,
    vmin=0.0,
    is_3d=False,
    trajectory=None,
    overlay_labels=("closed loop", "start", "end"),
    cmap="YlOrRd",
    ax=None,
    title="Cost-to-go",
    show=True,
):
    """
    Draw cost-to-go heatmap(s) of a node-indexed table ``J`` on the state grid.

    ``J`` is by grid node: value iteration's ``solution.record.J``, or a
    policy's ``J^π``. ``x_axis`` and ``y_axis`` are the two state axes of the
    slice, integers, or two lists of one length for several slices.
    """
    require_single_slice(trajectory, x_axis, y_axis)

    # Several slices: one heatmap per pair, without the overlay
    if is_batch(x_axis, y_axis):
        return [
            plot_cost_to_go(
                grid,
                J,
                jmax=jmax,
                x_axis=x_k,
                y_axis=y_k,
                anchor=anchor,
                vmin=vmin,
                is_3d=is_3d,
                trajectory=None,
                cmap=cmap,
                title=title,
                show=show,
            )
            for x_k, y_k in slices_of(x_axis, y_axis)
        ]

    require_flat_overlay(is_3d, trajectory)

    # The heatmap with the closed-loop trajectory over it, shown now unless the
    # surface comes after it
    fig, ax = plot_field(
        grid,
        J,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmin=vmin,
        vmax=jmax,
        cmap=cmap,
        ax=ax,
        title=title,
        show=show and not is_3d,
        overlay=trajectory_layers(trajectory, x_axis, y_axis, overlay_labels),
    )

    # The 3-D surface of J, clipped to 0 ≤ J (≤ jmax when given)
    if is_3d:
        J_plot = J if jmax is None else np.clip(J, 0.0, jmax)
        plot_value_3d(
            grid,
            np.clip(J_plot, 0.0, np.inf),
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            cmap=cmap,
            title=title,
            show=show,
        )

    return fig, ax


def plot_control_law(
    grid,
    pi,
    *,
    u_axis=0,
    x_axis=0,
    y_axis=1,
    anchor=None,
    trajectory=None,
    overlay_labels=("closed loop", "start", "end"),
    cmap="bwr",
    ax=None,
    show=True,
):
    """
    Draw a control-law heatmap of a node-indexed table ``pi`` (action ids) on the state grid.

    ``pi`` is by grid node: value iteration's ``solution.record.pi``.
    ``x_axis`` and ``y_axis`` are the two state axes of the slice, integers,
    or two lists of one length for several slices; ``u_axis`` (the input
    component) may be an int or a list aligned with them.
    """
    require_single_slice(trajectory, x_axis, y_axis)

    # Several slices: one heatmap per pair, each with its own input axis
    if is_batch(x_axis, y_axis):
        slices = slices_of(x_axis, y_axis)
        u_axes = u_axes_of(u_axis, slices)
        return [
            plot_control_law(
                grid,
                pi,
                u_axis=u_k,
                x_axis=x_k,
                y_axis=y_k,
                anchor=anchor,
                trajectory=None,
                cmap=cmap,
                show=show,
            )
            for (x_k, y_k), u_k in zip(slices, u_axes)
        ]

    # The heatmap with the closed-loop trajectory over it
    return plot_control_law_field(
        grid,
        pi,
        u_axis=u_axis,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        cmap=cmap,
        ax=ax,
        show=show,
        overlay=trajectory_layers(trajectory, x_axis, y_axis, overlay_labels),
    )


def animate_cost_to_go(
    grid,
    record,
    *,
    x_axis=0,
    y_axis=1,
    jmax=None,
    anchor=None,
    interval=60,
    file_name=None,
    show=True,
):
    """
    Animate the cost-to-go sweeps a solve recorded on ``record.history``, on ``grid``.

    The colours span ``[0, jmax]``; without ``jmax``, the range of the last
    sweep's slice.
    """
    require_history(record)

    return animate(
        grid,
        record.history,
        kind="value",
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmax=jmax,
        interval=interval,
        show=show,
        file_name=file_name,
    )


def animate_control_law(
    grid,
    record,
    *,
    axis=0,
    x_axis=0,
    y_axis=1,
    anchor=None,
    interval=60,
    file_name=None,
    show=True,
):
    """Animate the control-law sweeps a solve recorded on ``record.history``, on ``grid``."""
    require_history(record)

    return animate(
        grid,
        record.history,
        kind="policy",
        axis=axis,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        interval=interval,
        show=show,
        file_name=file_name,
    )


def plot_value(
    grid,
    J,
    *,
    x_axis=0,
    y_axis=1,
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
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmin=vmin,
        vmax=vmax,
        cmap=cmap,
        ax=ax,
        title=title,
        show=show,
    )


def plot_value_3d(
    grid,
    J,
    *,
    x_axis=0,
    y_axis=1,
    anchor=None,
    cmap="YlOrRd",
    title="Cost-to-go",
    show=True,
):
    """Draw a 3-D surface of a node-indexed value field ``J``."""
    import matplotlib.pyplot as plt

    # The 2-D slice of the field, over the mesh of its two state levels
    Z = grid.slice_2d(grid.grid_from_array(J), x_axis, y_axis, anchor=anchor)
    x_level = grid.x_levels[x_axis]
    y_level = grid.x_levels[y_axis]
    mesh_x, mesh_y = np.meshgrid(x_level, y_level)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    surf = ax.plot_surface(
        mesh_x, mesh_y, Z.T, cmap=cmap, linewidth=0, antialiased=False
    )
    label_axes(ax, grid, x_axis, y_axis)
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
    x_axis=0,
    y_axis=1,
    anchor=None,
    vmax=None,
    interval=60,
    show=True,
    file_name=None,
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
    x_level = grid.x_levels[x_axis]
    y_level = grid.x_levels[y_axis]
    Z = grid.slice_2d(
        grid.grid_from_array(frame_field(history[0])),
        x_axis,
        y_axis,
        anchor=anchor,
    )
    cmap = "bwr" if kind == "policy" else "YlOrRd"
    mesh = ax.pcolormesh(x_level, y_level, Z.T, shading="gouraud", cmap=cmap)

    # A value's colours span [0, vmax], the last sweep's largest value by default,
    # so every frame shares one scale
    if kind == "value":
        mesh.set_clim(
            vmin=0.0, vmax=last_sweep_max(grid, history, x_axis, y_axis, anchor, vmax)
        )

    label_axes(ax, grid, x_axis, y_axis)
    fig.colorbar(mesh, ax=ax)

    # Each frame swaps sweep i's slice into the same mesh
    def update(i):
        Z = grid.slice_2d(
            grid.grid_from_array(frame_field(history[i])),
            x_axis,
            y_axis,
            anchor=anchor,
        )
        mesh.set_array(Z.T.ravel())
        ax.set_title(f"sweep {i}  t={history[i][0]:.2f}")
        return (mesh,)

    ani = animation.FuncAnimation(
        fig, update, len(history), interval=interval, blit=False
    )
    if file_name is not None:
        ani.save(file_name, writer="pillow", fps=max(1, int(1000 / interval)))

    maybe_show(plt, show)

    return ani


def cost_scale_limit(failure_cost):
    """
    Default top of the cost-to-go colour scale: the cost of leaving the grid.

    States that must leave saturate at that cost, so clipping there keeps the
    feasible states readable. A callable ``failure_cost(x, t)`` has no single level and
    an infinite one is no limit: both leave the scale to Matplotlib (``None``).
    """
    if failure_cost is None or callable(failure_cost) or not np.isfinite(failure_cost):
        return None

    return float(failure_cost)


###############################################################################
# Internal machinery
###############################################################################


def require_single_slice(trajectory, x_axis, y_axis):
    """Refuse a trajectory overlay on a batch of slices."""
    if trajectory is not None and is_batch(x_axis, y_axis):
        raise ValueError(
            "trajectory overlay requires one slice: x_axis and y_axis as integers"
        )


def is_batch(x_axis, y_axis):
    """``x_axis`` and ``y_axis`` list several slices rather than naming one."""
    return isinstance(x_axis, (tuple, list)) or isinstance(y_axis, (tuple, list))


def slices_of(x_axis, y_axis):
    """The ``(x_axis, y_axis)`` pairs of a batch: the two lists side by side, of one length."""
    if len(x_axis) != len(y_axis):
        raise ValueError("x_axis and y_axis lists must have the same length")

    return list(zip(x_axis, y_axis))


def require_flat_overlay(is_3d, trajectory):
    """Refuse a trajectory overlay together with the 3-D surface."""
    if is_3d and trajectory is not None:
        raise ValueError("is_3d and trajectory cannot be used together")


def overlay_trajectory(ax, trajectory, x_axis, y_axis, labels):
    """Draw a trajectory's path, start and end on the ``(x_axis, y_axis)`` slice."""
    x = trajectory.x
    ax.plot(x[x_axis], x[y_axis], "k-", linewidth=2, label=labels[0])
    ax.plot(x[x_axis, 0], x[y_axis, 0], "go", label=labels[1])
    ax.plot(x[x_axis, -1], x[y_axis, -1], "r*", markersize=12, label=labels[2])
    ax.legend(loc="upper right", fontsize=8)


def plot_control_law_field(
    grid, pi, *, u_axis, x_axis, y_axis, anchor, cmap, ax, show, overlay=()
):
    """Heatmap of input ``u_axis`` of a control-law table, scaled to that input's bounds."""
    u = grid.input_from_policy(pi)[:, u_axis]

    return plot_field(
        grid,
        u,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmin=grid.u_lb[u_axis],
        vmax=grid.u_ub[u_axis],
        cmap=cmap,
        ax=ax,
        title=f"Control law u[{u_axis}]",
        show=show,
        overlay=overlay,
    )


def u_axes_of(u_axis, slices):
    """One input axis per slice: the list given, or ``u_axis`` repeated."""
    if isinstance(u_axis, (list, tuple)):
        if len(u_axis) != len(slices):
            raise ValueError(
                "u_axis list must match the x_axis and y_axis lists' length"
            )
        return u_axis

    return [u_axis] * len(slices)


def last_sweep_max(grid, history, x_axis, y_axis, anchor, vmax):
    """The colour limit ``vmax`` as given, else the largest value of the last sweep's slice."""
    if vmax is not None:
        return vmax

    _, J_last, _ = history[-1]
    Z = grid.slice_2d(grid.grid_from_array(J_last), x_axis, y_axis, anchor=anchor)

    return float(np.max(Z))


def require_history(record):
    """Refuse an animation of a solve that recorded no sweeps."""
    if not record.history:
        raise ValueError(
            "record.history is empty; re-run with record_history=True to animate"
        )


def plot_field(
    grid,
    values,
    *,
    x_axis,
    y_axis,
    anchor,
    vmin,
    vmax,
    cmap,
    ax,
    title,
    show,
    overlay=(),
):
    """Heatmap of a node-indexed field on a 2-D slice of the grid, ``overlay`` layers over it."""
    spec = field_figure(
        grid,
        values,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmin=vmin,
        vmax=vmax,
        cmap=cmap,
        title=title,
        overlay=overlay,
    )
    result = draw(spec, show=show, ax=ax)

    return result.figure, result.axes


def field_figure(
    grid, values, *, x_axis, y_axis, anchor, vmin, vmax, cmap, title, overlay=()
):
    """The heatmap of a node-indexed field on a 2-D slice of the grid, as a description."""
    # The 2-D slice of the field, over its two state levels
    Z = grid.slice_2d(grid.grid_from_array(values), x_axis, y_axis, anchor=anchor)
    heatmap = Heatmap(
        x=grid.x_levels[x_axis],
        y=grid.x_levels[y_axis],
        z=Z.T,
        cmap=cmap,
        vmin=vmin,
        vmax=vmax,
    )
    panel = Panel(
        layers=(heatmap, *overlay),
        title=title,
        x_label=axis_name(grid, x_axis),
        y_label=axis_name(grid, y_axis),
        legend="upper right",
    )

    return FigureSpec(panels=(panel,))


def trajectory_layers(trajectory, x_axis, y_axis, labels):
    """A trajectory's path, start and end on the ``(x_axis, y_axis)`` slice, as layers."""
    if trajectory is None:
        return ()
    x = trajectory.x

    return (
        Trace(x[x_axis], x[y_axis], color="black", width=2.0, label=labels[0]),
        Trace(
            x[x_axis, :1], x[y_axis, :1], color="green", mode="markers", label=labels[1]
        ),
        Trace(
            x[x_axis, -1:],
            x[y_axis, -1:],
            color="red",
            mode="markers",
            marker="*",
            size=12.0,
            label=labels[2],
        ),
    )


def axis_name(grid, i):
    """A state axis's label and unit, ``x[i]`` without labels."""
    labels = getattr(grid.sys.state, "labels", None)
    units = getattr(grid.sys.state, "units", None)
    base = labels[i] if labels else f"x[{i}]"
    unit = f" [{units[i]}]" if units and units[i] else ""

    return base + unit


def label_axes(ax, grid, x_axis, y_axis):
    """Name the two sliced state axes by their labels and units, ``x[i]`` without labels."""
    labels = getattr(grid.sys.state, "labels", None)
    units = getattr(grid.sys.state, "units", None)

    def name(i):
        base = labels[i] if labels else f"x[{i}]"
        unit = f" [{units[i]}]" if units and units[i] else ""
        return base + unit

    ax.set_xlabel(name(x_axis))
    ax.set_ylabel(name(y_axis))
