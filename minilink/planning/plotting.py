"""The planning views of one solution: its cost-to-go, its control law and its trajectory."""

from __future__ import annotations

import numpy as np

from minilink.core.fields import GridField
from minilink.core.sets import is_finite_box
from minilink.graphical.common.labels import state_axis_label
from minilink.graphical.figure import FigureSpec, Heatmap, Panel, Surface, draw, overlay
from minilink.planning.dynamic_programming import plotting as grid_plotting

###############################################################################
# Public API
###############################################################################


def draw_cost_to_go(
    problem,
    field,
    *,
    grid,
    family,
    x_axis,
    y_axis,
    anchor,
    bounds,
    grid_shape,
    t,
    jmax,
    is_3d,
    trajectory,
    title,
    renderer,
    show,
    ax,
):
    """
    The cost-to-go field ``J`` drawn over two state axes, the others pinned at ``anchor``: a :class:`PlotResult`.

    A grid planner's table is drawn on the grid's own nodes (``grid``, the
    field that grid's table) unless the call asks for another mesh
    (``bounds``, ``grid_shape``) or another time; any other field is sampled
    on a mesh over ``bounds``, the problem's state box by default, ``anchor``
    the goal, else the start. ``jmax`` tops the colour scale, by default the
    field's value past its grid (the failure cost) when it has one; an
    infinite ``jmax`` hands the scale to the renderer. ``is_3d`` draws the
    surface instead of the heatmap; ``trajectory`` overlays a run on the
    heatmap.
    """
    grid_plotting.require_flat_overlay(is_3d, trajectory)
    jmax = color_top(field, jmax)

    # On the grid's own nodes: the table itself
    if is_grid_table(field, grid) and on_the_nodes(bounds, grid_shape, (51, 51), t):
        return grid_plotting.plot_cost_to_go(
            grid,
            node_table(field),
            jmax=jmax,
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            is_3d=is_3d,
            trajectory=trajectory,
            title="Cost-to-go" if title is None else title,
            renderer=renderer,
            show=show,
            ax=ax,
        )

    # Else the field sampled on a mesh
    x_level, y_level, J = sample_field(
        problem,
        field,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
    )
    title = f"cost-to-go ({family})" if title is None else title

    # The surface
    if is_3d:
        surface = cost_to_go_surface(
            problem, x_level, y_level, J, x_axis, y_axis, jmax, title
        )
        return draw(surface, renderer=renderer, show=show, ax=ax)

    # The heatmap with the run over it
    overlay = grid_plotting.trajectory_layers(
        trajectory, x_axis, y_axis, ("closed loop", "start", "end")
    )
    panel = cost_to_go_panel(
        problem, x_level, y_level, J, x_axis, y_axis, jmax, title, overlay
    )
    heatmap = FigureSpec(panels=(panel,))

    return draw(heatmap, renderer=renderer, show=show, ax=ax)


def plot_control_law(
    solution,
    *,
    x_axis=None,
    y_axis=None,
    u_axis=0,
    anchor=None,
    bounds=None,
    grid_shape=(101, 101),
    t=0.0,
    is_3d=False,
    trajectory=None,
    title=None,
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    A solution's feedback law over two state axes: :meth:`PlanningSolution.plot_control_law`.

    A grid planner's law is drawn as its table on the grid's own nodes unless
    the call asks for another mesh (``bounds``, ``grid_shape``), another time
    or the surface, the other axes at ``anchor``; any other law is the
    controller's own ``plot_control_law``, ``bounds`` from the problem's box
    and the colour limits from its inputs, the other coordinates at their
    nominal values (``anchor`` refused). ``trajectory`` overlays a run on the
    heatmap.
    """
    grid_plotting.require_flat_overlay(is_3d, trajectory)
    solution.require_feedback_law()
    pi = getattr(solution.record, "pi", None)
    grid = getattr(solution.controller, "grid", None)
    i = 0 if x_axis is None else x_axis
    j = 1 if y_axis is None else y_axis

    # On the grid's own nodes: the law's table itself
    if (
        pi is not None
        and grid is not None
        and not is_3d
        and on_the_nodes(bounds, grid_shape, (101, 101), t)
    ):
        return grid_plotting.plot_control_law(
            grid,
            pi,
            u_axis=u_axis,
            x_axis=i,
            y_axis=j,
            anchor=anchor,
            trajectory=trajectory,
            title=title,
            renderer=renderer,
            show=show,
            ax=ax,
        )

    # Else the controller's own law over the problem's box, with the run over it
    require_no_anchor(anchor)
    law = controller_law_figure(
        solution,
        x_axis=x_axis,
        y_axis=y_axis,
        u_axis=u_axis,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
        is_3d=is_3d,
        title=title,
    )
    run = grid_plotting.trajectory_layers(
        trajectory, i, j, ("closed loop", "start", "end")
    )
    spec = overlay(law, [run])

    return draw(spec, renderer=renderer, show=show, ax=ax)


###############################################################################
# Internal machinery
###############################################################################


def controller_law_figure(solution, *, bounds=None, **kwargs):
    """The description of the solution's controller's own ``plot_control_law``, ``bounds`` and the colour limits filled from the problem."""
    given = dict(kwargs)
    if bounds is not None:
        given["bounds"] = bounds
    law = solution.controller.plot_control_law(
        **control_law_kwargs(solution, given), renderer=None
    )

    return law.spec


def sample_field(problem, field, *, x_axis, y_axis, anchor, bounds, grid_shape, t):
    """``(x_level, y_level, J)``: a field sampled on a mesh over the state axes ``x_axis`` and ``y_axis``, the others at ``anchor``."""
    box = state_box(problem)

    # The sweep's bounds: the state box over the two axes, unless given
    if bounds is None:
        require_bounded_state_box(box)
        i, j = x_axis, y_axis
        bounds = ((box.lower[i], box.upper[i]), (box.lower[j], box.upper[j]))

    # The other axes pinned at the anchor: the goal, else the start
    if anchor is None:
        anchor = problem.goal_point()
    anchor = np.asarray(anchor, dtype=float)

    # The mesh levels of the two axes
    x_level = np.linspace(bounds[0][0], bounds[0][1], int(grid_shape[0]))
    y_level = np.linspace(bounds[1][0], bounds[1][1], int(grid_shape[1]))

    # J(x) at every mesh point, J(x, t) away from t = 0
    J = np.empty((x_level.size, y_level.size))
    x = anchor.copy()
    for i, xi in enumerate(x_level):
        for j, yj in enumerate(y_level):
            x[x_axis], x[y_axis] = xi, yj
            J[i, j] = float(field(x) if t == 0.0 else field(x, t=t))

    return x_level, y_level, J


def plot_solution_trajectory(solution, traj, *, signals, renderer, show, ax=None):
    """The solution's plan, or its controller's run ``traj``, on its system, the figure titled as plan or run."""
    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.signals.time_signals import plot_time_signals

    # The title says which: the plan the planner computed, or the controller's run
    kind = "plan" if solution.trajectory is not None else "run"

    return plot_time_signals(
        solution.problem.sys,
        traj,
        signals=signals,
        title=f"{kind} ({solution.family})",
        renderer=renderer,
        show=show,
        ax=ax,
    )


def control_law_kwargs(solution, kwargs) -> dict:
    """Fill ``bounds`` from the problem's box and the colour limits from its inputs, unless given."""
    kwargs = dict(kwargs)
    problem = solution.problem
    box = state_box(problem)
    n = int(problem.sys.n)

    # The sweep the policy's verb picks by default: x[0] against x[1], x[0] alone on
    # a scalar state
    x_axis = kwargs.get("x_axis")
    y_axis = kwargs.get("y_axis")
    x_axis = 0 if x_axis is None else x_axis
    y_axis = (1 if n >= 2 else None) if y_axis is None else y_axis

    # The sweep's bounds from the state box, unless given
    if "bounds" not in kwargs and box is not None:
        if y_axis is None:
            kwargs["bounds"] = (box.lower[x_axis], box.upper[x_axis])
        else:
            kwargs["bounds"] = (
                (box.lower[x_axis], box.upper[x_axis]),
                (box.lower[y_axis], box.upper[y_axis]),
            )

    # The colour limits from the input set's box when it is finite, unless given
    u_box = problem.U.bounding_box()
    u_axis = kwargs.get("u_axis", 0)
    if is_finite_box(u_box):
        kwargs.setdefault("vmin", float(u_box.lower[u_axis]))
        kwargs.setdefault("vmax", float(u_box.upper[u_axis]))

    return kwargs


def require_no_anchor(anchor):
    """Refuse an anchor for a law the controller draws: it pins the other coordinates at their nominal values."""
    if anchor is not None:
        raise ValueError(
            "anchor pins the other axes of a grid planner's law; a controller's own "
            "law is drawn with the other coordinates at their nominal values"
        )


def color_top(field, jmax):
    """The top of the cost-to-go colour scale: ``jmax``, else the field's value past its grid; an infinite one is no top (``None``)."""
    if jmax is None:
        jmax = (field.params or {}).get("outside")

    return grid_plotting.cost_scale_limit(jmax)


def is_grid_table(field, grid) -> bool:
    """Whether ``field`` is the table of ``grid``: a grid field on its levels, one value per node."""
    if grid is None or not isinstance(field, GridField):
        return False

    return node_table(field).size == grid.n_nodes and field.n == len(grid.x_levels)


def on_the_nodes(bounds, grid_shape, default_shape, t) -> bool:
    """Whether a call reads a table on its own nodes: no other mesh asked for (``bounds``, ``grid_shape``), at the first time."""
    return bounds is None and tuple(grid_shape) == default_shape and t == 0.0


def node_table(field) -> np.ndarray:
    """A grid field's values by node id ``(n_nodes,)``: its first table on a schedule (stage 0)."""
    values = np.asarray(field.params["values"], dtype=float)
    if field.timed:
        values = values[0]

    return values.reshape(-1)


def cost_to_go_panel(
    problem, x_level, y_level, J, x_axis, y_axis, jmax, title, layers=()
):
    """The heatmap of a sampled field on the colour scale ``0 ≤ J ≤ jmax``, ``layers`` over it, as a panel."""
    heatmap = Heatmap(x=x_level, y=y_level, z=J.T, cmap="YlOrRd", vmin=0.0, vmax=jmax)

    return Panel(
        layers=(heatmap, *layers),
        title=title,
        x_label=state_axis_label(problem.sys, x_axis),
        y_label=state_axis_label(problem.sys, y_axis),
        legend=True,
    )


def cost_to_go_surface(problem, x_level, y_level, J, x_axis, y_axis, jmax, title):
    """The surface of a sampled field, clipped to ``0 ≤ J ≤ jmax``, as a description."""
    J_plot = np.clip(J, 0.0, np.inf if jmax is None else jmax)
    surface = Surface(x=x_level, y=y_level, z=J_plot.T, cmap="YlOrRd")
    panel = Panel(
        layers=(surface,),
        title=title,
        x_label=state_axis_label(problem.sys, x_axis),
        y_label=state_axis_label(problem.sys, y_axis),
        z_label="J",
        is_3d=True,
    )

    return FigureSpec(panels=(panel,))


def state_box(problem):
    """The finite box to sweep: the problem's constraint set, else the plant's state box, else ``None``."""
    box = problem.X.bounding_box()
    if is_finite_box(box):
        return box
    box = problem.sys.state.box
    return box if is_finite_box(box) else None


def require_bounded_state_box(box):
    """Refuse a sweep without ``bounds`` over an unbounded state box."""
    if box is None:
        raise ValueError(
            "the problem's state box is unbounded; pass bounds=((xlo, xhi), (ylo, yhi))"
        )
