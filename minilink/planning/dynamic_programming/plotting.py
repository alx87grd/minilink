"""Views of a dynamic-programming cost-to-go and policy: heatmaps, surfaces and sweep animations."""

import numpy as np

from minilink.graphical.common.animation_result import require_matplotlib_animation
from minilink.graphical.common.labels import state_axis_label
from minilink.graphical.figure import FigureSpec, Heatmap, Panel, Surface, Trace, draw

# The value-iteration result is a node-indexed field over the state grid: these
# views reshape it, take a 2-D slice when n > 2, and describe it as a heatmap or
# a 3-D surface that a renderer draws, or animate it over the recorded sweeps.


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
    title="Cost-to-go",
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Draw cost-to-go heatmap(s) of a node-indexed table ``J`` on the state grid.

    ``J`` is by grid node: value iteration's ``solution.record.J``, or a
    policy's ``J^π``. ``x_axis`` and ``y_axis`` are the two state axes of the
    slice, integers, or two lists of one length for several slices.
    ``is_3d`` draws the surface of ``J`` instead of the heatmap.
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
                renderer=renderer,
                show=show,
            )
            for x_k, y_k in slices_of(x_axis, y_axis)
        ]

    require_flat_overlay(is_3d, trajectory)

    # The 3-D surface of J, clipped to 0 ≤ J (≤ jmax when given)
    if is_3d:
        J_plot = J if jmax is None else np.clip(J, 0.0, jmax)
        surface = value_surface_figure(
            grid,
            np.clip(J_plot, 0.0, np.inf),
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            cmap=cmap,
            title=title,
        )
        return draw(surface, renderer=renderer, show=show, ax=ax)

    # The heatmap with the closed-loop trajectory over it
    heatmap = field_figure(
        grid,
        J,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmin=vmin,
        vmax=jmax,
        cmap=cmap,
        title=title,
        overlay=trajectory_layers(trajectory, x_axis, y_axis, overlay_labels),
    )

    return draw(heatmap, renderer=renderer, show=show, ax=ax)


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
    title=None,
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Draw a control-law heatmap of a node-indexed table ``pi`` (action ids) on the state grid.

    ``pi`` is by grid node: value iteration's ``solution.record.pi``.
    ``x_axis`` and ``y_axis`` are the two state axes of the slice, integers,
    or two lists of one length for several slices; ``u_axis`` (the input
    component) may be an int or a list aligned with them. ``title`` replaces
    the title ``Control law u[i]``.
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
                title=title,
                renderer=renderer,
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
        title=title,
        overlay=trajectory_layers(trajectory, x_axis, y_axis, overlay_labels),
        renderer=renderer,
        show=show,
        ax=ax,
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
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Animate the cost-to-go sweeps a solve recorded on ``record.history``, on ``grid``.

    The colours span ``[0, jmax]``; without ``jmax``, the range of the last
    sweep's slice. Drawn with matplotlib, on a new figure or into ``ax``;
    ``show=None`` plays it unless it is drawn into ``ax``.
    """
    require_matplotlib_animation(renderer)
    require_history(record)

    return animate_field(
        grid,
        record.history,
        kind="value",
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        vmax=jmax,
        interval=interval,
        show=show,
        ax=ax,
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
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Animate the control-law sweeps a solve recorded on ``record.history``, on ``grid``.

    Drawn with matplotlib, on a new figure or into ``ax``; ``show=None`` plays
    it unless it is drawn into ``ax``.
    """
    require_matplotlib_animation(renderer)
    require_history(record)

    return animate_field(
        grid,
        record.history,
        kind="policy",
        axis=axis,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        interval=interval,
        show=show,
        ax=ax,
    )


def cost_scale_limit(failure_cost):
    """
    Default top of the cost-to-go colour scale: the cost of leaving the grid.

    States that must leave saturate at that cost, so clipping there keeps the
    feasible states readable. A callable ``failure_cost(x, t)`` has no single level and
    an infinite one is no limit: both leave the scale to the renderer (``None``).
    """
    if failure_cost is None or callable(failure_cost) or not np.isfinite(failure_cost):
        return None

    return float(failure_cost)


###############################################################################
# Internal machinery
###############################################################################


def animate_field(
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
    show=None,
    ax=None,
):
    """
    Animate a recorded value or policy field over the sweeps of ``history``.

    ``kind`` is ``"value"`` (the cost-to-go ``J``) or ``"policy"`` (input
    ``axis`` of the greedy law); ``history`` is the ``(t, J, pi)`` list from a
    solve. A value is coloured on ``[0, vmax]``, by default up to the largest
    value of the last sweep's slice. Drawn with matplotlib.
    """
    # Lazy import: the graphics load on first use, not with the planning layer
    from minilink.graphical.animation.grid_animations import animate_heatmap

    # Sweep i's slice of the value, or of the policy's input axis
    def frame_of(i):
        _, J, pi = history[i]
        values = grid.input_from_policy(pi)[:, axis] if kind == "policy" else J
        Z = grid.slice_2d(grid.grid_from_array(values), x_axis, y_axis, anchor=anchor)
        return Z.T

    # A value's colours span [0, vmax], the last sweep's largest value by default,
    # so every frame shares one scale
    is_value = kind == "value"
    top = (
        last_sweep_max(grid, history, x_axis, y_axis, anchor, vmax)
        if is_value
        else None
    )

    return animate_heatmap(
        grid.x_levels[x_axis],
        grid.x_levels[y_axis],
        frame_of,
        len(history),
        title_of=lambda i: f"sweep {i}  t={history[i][0]:.2f}",
        cmap="YlOrRd" if is_value else "bwr",
        vmin=0.0 if is_value else None,
        vmax=top,
        x_label=state_axis_label(grid.sys, x_axis),
        y_label=state_axis_label(grid.sys, y_axis),
        interval=interval,
        show=show,
        ax=ax,
    )


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


def plot_control_law_field(
    grid,
    pi,
    *,
    u_axis,
    x_axis,
    y_axis,
    anchor,
    cmap,
    title=None,
    overlay=(),
    renderer="matplotlib",
    show=None,
    ax=None,
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
        title=f"Control law u[{u_axis}]" if title is None else title,
        overlay=overlay,
        renderer=renderer,
        show=show,
        ax=ax,
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
    title,
    overlay=(),
    renderer="matplotlib",
    show=None,
    ax=None,
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

    return draw(spec, renderer=renderer, show=show, ax=ax)


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
        x_label=state_axis_label(grid.sys, x_axis),
        y_label=state_axis_label(grid.sys, y_axis),
        legend="upper right",
    )

    return FigureSpec(panels=(panel,))


def value_surface_figure(grid, J, *, x_axis, y_axis, anchor, cmap, title):
    """The 3-D surface of a node-indexed field on a 2-D slice of the grid, as a description."""
    # The 2-D slice of the field, over its two state levels
    Z = grid.slice_2d(grid.grid_from_array(J), x_axis, y_axis, anchor=anchor)
    surface = Surface(
        x=grid.x_levels[x_axis], y=grid.x_levels[y_axis], z=Z.T, cmap=cmap
    )
    panel = Panel(
        layers=(surface,),
        title=title,
        x_label=state_axis_label(grid.sys, x_axis),
        y_label=state_axis_label(grid.sys, y_axis),
        z_label="J",
        is_3d=True,
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
