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
    path_style: str = "dense",
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
        if path_style == "waypoints":
            waypoints = _solution_waypoints(planner)
            if waypoints is not None:
                ax.plot(
                    waypoints[:, x_axis],
                    waypoints[:, y_axis],
                    color=path_color,
                    lw=2.5,
                    zorder=3,
                    label="path",
                )
        else:
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


def animate_convergence(
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    ax=None,
    interval: int = 50,
    show: bool = True,
    save: str | None = None,
    figsize=(7.0, 6.0),
    tree_color="0.75",
    path_color="tab:blue",
    title_prefix: str = "RRT* convergence",
):
    """
    Animate recorded RRT* history showing tree growth and best-path refinement.

    Requires ``planner.history`` from a solve with ``record_history=True``.
    Each frame redraws the full tree (including rewiring) and highlights the
    current best goal path. Returns the
    :class:`~matplotlib.animation.FuncAnimation` (keep a reference).
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    history = getattr(planner, "history", None)
    if not history:
        raise ValueError(
            "No search history recorded; run compute_solution with record_history=True"
        )

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    _set_limits(ax, planner, x_axis, y_axis)
    _label_axes(ax, planner.problem.sys, x_axis, y_axis)
    _mark_endpoints(ax, planner, x_axis, y_axis)

    tree_lines: list = []
    path_line = None
    title = ax.set_title(title_prefix)

    def _clear_artists():
        nonlocal path_line
        for line in tree_lines:
            line.remove()
        tree_lines.clear()
        if path_line is not None:
            path_line.remove()
            path_line = None

    def update(frame):
        nonlocal path_line
        snapshot = history[frame]
        _clear_artists()

        for edge_states in snapshot.tree_edges:
            (line,) = ax.plot(
                edge_states[:, x_axis],
                edge_states[:, y_axis],
                color=tree_color,
                lw=0.35,
                zorder=1,
            )
            tree_lines.append(line)

        if snapshot.path_states is not None:
            (path_line,) = ax.plot(
                snapshot.path_states[:, x_axis],
                snapshot.path_states[:, y_axis],
                color=path_color,
                lw=2.5,
                zorder=3,
            )

        if snapshot.reached_goal and snapshot.best_cost is not None:
            title.set_text(
                f"{title_prefix}  iter={snapshot.iteration}  "
                f"best cost={snapshot.best_cost:.2f}"
            )
        else:
            title.set_text(
                f"{title_prefix}  iter={snapshot.iteration}  searching..."
            )
        return tree_lines + ([path_line] if path_line is not None else [])

    anim = FuncAnimation(
        fig,
        update,
        frames=len(history),
        interval=interval,
        blit=False,
        repeat=True,
    )
    if save is not None:
        anim.save(save)
    _maybe_show(plt, show)
    return anim


def draw_search_state(
    ax,
    planner,
    *,
    x_axis: int = 0,
    y_axis: int = 1,
    tree_color="0.75",
    path_color="tab:blue",
):
    """
    Draw the current search tree and best path on ``ax``.

    Returns ``(tree_lines, path_line)`` artist handles for live updates.
    """
    return _draw_search_state(
        ax,
        planner,
        x_axis=x_axis,
        y_axis=y_axis,
        tree_color=tree_color,
        path_color=path_color,
    )


# Internal machinery


def _draw_search_state(
    ax,
    planner,
    *,
    x_axis,
    y_axis,
    tree_color,
    path_color,
):
    tree_lines = []
    for node in planner.tree.nodes:
        if node.parent is None:
            continue
        states = np.asarray(node.edge.states, dtype=float)
        (line,) = ax.plot(
            states[:, x_axis],
            states[:, y_axis],
            color=tree_color,
            lw=0.35,
            zorder=1,
        )
        tree_lines.append(line)

    path_line = None
    path_states = _best_path_states(planner)
    if path_states is not None:
        (path_line,) = ax.plot(
            path_states[:, x_axis],
            path_states[:, y_axis],
            color=path_color,
            lw=2.5,
            zorder=3,
        )
    return tree_lines, path_line


def _clear_search_artists(tree_lines, path_line):
    for line in list(tree_lines):
        line.remove()
    tree_lines.clear()
    if path_line is not None:
        path_line.remove()


def _best_path_states(planner):
    node = getattr(planner, "solution_node", None)
    if node is None or not getattr(planner, "reached_goal", False):
        return None
    if hasattr(planner, "_path_states"):
        return planner._path_states(node)
    return None


def _solution_waypoints(planner):
    """Tree-node polyline from root to ``solution_node`` (no edge subsampling)."""
    node = getattr(planner, "solution_node", None)
    tree = getattr(planner, "tree", None)
    if node is None or tree is None:
        return None

    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
    chain.reverse()

    points = [np.asarray(tree.root.x, dtype=float)]
    for nd in chain:
        points.append(np.asarray(nd.x, dtype=float))
    return np.asarray(points)


def _search_title(step):
    if step.reached_goal and step.best_cost is not None:
        return (
            f"RRT search  phase={step.phase}  iter={step.iteration}  "
            f"nodes={step.tree_nodes}  best cost={step.best_cost:.2f}"
        )
    return (
        f"RRT search  phase={step.phase}  iter={step.iteration}  "
        f"nodes={step.tree_nodes}  searching..."
    )


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
