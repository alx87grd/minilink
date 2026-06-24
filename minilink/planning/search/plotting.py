"""
Built-in search-tree visualisation for the RRT family.

:func:`plot_tree` draws the final tree and solution path in a chosen pair of
state axes; :func:`animate_search` replays the tree growth (node insertion order)
as an animation. Both are reached through the `RRTPlanner.plot_tree` /
`RRTPlanner.animate_search` facades and import matplotlib lazily; pass an ``ax``
(e.g. from ``scene.plot(...)``) to overlay the tree on a scene heatmap.
"""

import numpy as np

# Public API


def plot_tree(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    ax=None,
    show: bool = True,
    figsize=(6.0, 5.0),
    tree_color="0.6",
    path_color="tab:blue",
    title: str | None = None,
):
    """
    Plot the final tree and solution in the ``(x_axis, y_axis)`` state projection.

    Returns ``(fig, ax)``. Tree edges are grey, the solution path is highlighted,
    and the start, goal, and goal region are marked.
    """
    import matplotlib.pyplot as plt

    tree = _require_tree(planner)
    problem = planner.problem
    created = ax is None
    if created:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    for node in tree.nodes:
        if node.parent is not None:
            states = node.edge.states
            ax.plot(
                states[:, x_axis], states[:, y_axis], color=tree_color, lw=0.4, zorder=1
            )

    if planner.last_result is not None:
        traj = planner.last_result
        ax.plot(
            traj.x[x_axis],
            traj.x[y_axis],
            color=path_color,
            lw=2.5,
            zorder=3,
            label="path",
        )

    _mark_endpoints(ax, planner, x_axis, y_axis)
    _label_axes(ax, problem.sys, x_axis, y_axis)
    if title is not None:
        ax.set_title(title)
    elif created:
        ax.set_title(f"RRT tree ({len(tree.nodes)} nodes)")
    ax.legend(loc="best")

    _maybe_show(plt, show)
    return fig, ax


def animate_search(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    ax=None,
    step: int = 5,
    interval: int = 30,
    show: bool = True,
    save: str | None = None,
    figsize=(6.0, 5.0),
    tree_color="0.6",
    path_color="tab:blue",
):
    """
    Replay the tree growth as an animation in the ``(x_axis, y_axis)`` projection.

    ``step`` edges are revealed per frame; the solution path is drawn at the end.
    Returns the :class:`~matplotlib.animation.FuncAnimation` (keep a reference).
    Pass ``save="tree.gif"`` to write it, or ``show=True`` to play it.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    tree = _require_tree(planner)
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    _set_limits(ax, planner, x_axis, y_axis)
    _label_axes(ax, planner.problem.sys, x_axis, y_axis)
    _mark_endpoints(ax, planner, x_axis, y_axis)
    ax.set_title("RRT search")

    edges = [node for node in tree.nodes if node.parent is not None]
    drawn: list = []

    def update(frame):
        target = min(len(edges), (frame + 1) * step)
        while len(drawn) < target:
            states = edges[len(drawn)].edge.states
            (line,) = ax.plot(
                states[:, x_axis], states[:, y_axis], color=tree_color, lw=0.4, zorder=1
            )
            drawn.append(line)
        if target >= len(edges) and planner.last_result is not None:
            traj = planner.last_result
            ax.plot(traj.x[x_axis], traj.x[y_axis], color=path_color, lw=2.5, zorder=3)
        return drawn

    n_frames = (len(edges) + step - 1) // step + 1
    anim = FuncAnimation(
        fig, update, frames=n_frames, interval=interval, blit=False, repeat=False
    )
    if save is not None:
        anim.save(save)
    _maybe_show(plt, show)
    return anim


# Internal machinery


def _require_tree(planner):
    if getattr(planner, "tree", None) is None:
        raise ValueError("No search tree yet; call compute_solution() first")
    return planner.tree


def _mark_endpoints(ax, planner, x_axis, y_axis):
    import matplotlib.pyplot as plt

    problem = planner.problem
    x0 = np.asarray(problem.x_start, dtype=float)
    ax.plot([x0[x_axis]], [x0[y_axis]], "ks", zorder=4, label="start")
    if problem.x_goal is not None:
        xg = np.asarray(problem.x_goal, dtype=float)
        ax.plot([xg[x_axis]], [xg[y_axis]], "r*", markersize=13, zorder=4, label="goal")
        radius = _goal_radius(planner)
        if radius is not None:
            ax.add_patch(
                plt.Circle(
                    (xg[x_axis], xg[y_axis]),
                    radius,
                    color="r",
                    fill=False,
                    ls="--",
                    zorder=2,
                )
            )


def _goal_radius(planner):
    from minilink.core.sets import BallSet

    Xf = planner.problem.Xf
    if isinstance(Xf, BallSet):
        return float(Xf.radius)
    if planner.problem.x_goal is not None:
        return float(planner.options.goal_tolerance)
    return None


def _label_axes(ax, sys, x_axis, y_axis):
    labels = getattr(sys.state, "labels", None)
    units = getattr(sys.state, "units", None)

    def axis_label(i):
        name = labels[i] if labels and i < len(labels) else f"x[{i}]"
        unit = f" [{units[i]}]" if units and i < len(units) and units[i] else ""
        return name + unit

    ax.set_xlabel(axis_label(x_axis))
    ax.set_ylabel(axis_label(y_axis))


def _set_limits(ax, planner, x_axis, y_axis):
    pts = np.array([node.x for node in planner.tree.nodes], dtype=float)
    if planner.problem.x_goal is not None:
        pts = np.vstack([pts, np.asarray(planner.problem.x_goal, dtype=float)])
    for axis, setter in ((x_axis, ax.set_xlim), (y_axis, ax.set_ylim)):
        lo, hi = float(pts[:, axis].min()), float(pts[:, axis].max())
        pad = 0.05 * (hi - lo or 1.0)
        setter(lo - pad, hi + pad)


def _maybe_show(plt, show):
    if show and plt.get_backend().lower() != "agg":
        from minilink.graphical.common.environment import is_blocking_needed

        plt.show(block=is_blocking_needed())
