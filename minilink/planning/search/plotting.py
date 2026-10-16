"""Search-tree plots: the RRT family's tree and solution path in a state projection."""

import numpy as np

# Matplotlib is imported inside each view, so the planners stay usable without a
# display backend until a plot is requested.


###############################################################################
# Public API
###############################################################################


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
    and the start, goal, and goal region are marked. Reached through
    ``RRTPlanner.plot_tree``; pass an ``ax`` (e.g. from ``scene.plot(...)``) to
    overlay the tree on a scene heatmap.
    """
    import matplotlib.pyplot as plt

    tree = require_tree(planner)
    problem = planner.problem

    # A new figure, or the one that holds the given axes
    created = ax is None
    if created:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    # The tree: each edge drawn along the states its extension swept
    for node in tree.nodes:
        if node.parent is not None:
            states = node.edge.states
            ax.plot(
                states[:, x_axis], states[:, y_axis], color=tree_color, lw=0.4, zorder=1
            )

    # The solution path: through the tree's nodes, or along the dense trajectory
    if planner.last_solution is not None:
        if path_style == "waypoints":
            waypoints = solution_waypoints(planner)
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
            traj = planner.last_solution.trajectory
            ax.plot(
                traj.x[x_axis],
                traj.x[y_axis],
                color=path_color,
                lw=2.5,
                zorder=3,
                label="path",
            )

    # The start and the goal, the labels and the title
    mark_endpoints(ax, planner, x_axis, y_axis)
    label_axes(ax, problem.sys, x_axis, y_axis)
    if title is not None:
        ax.set_title(title)
    elif created:
        ax.set_title(f"RRT tree ({len(tree.nodes)} nodes)")
    ax.legend(loc="best")

    maybe_show(plt, show)

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

    ``step`` edges are revealed per frame, in node insertion order; the solution
    path is drawn at the end. Returns the
    :class:`~matplotlib.animation.FuncAnimation` (keep a reference). Pass
    ``save="tree.gif"`` to write it, or ``show=True`` to play it.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    tree = require_tree(planner)

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    # The frame: the limits, labels, endpoints and title stay fixed while the tree grows
    set_limits(ax, planner, x_axis, y_axis)
    label_axes(ax, planner.problem.sys, x_axis, y_axis)
    mark_endpoints(ax, planner, x_axis, y_axis)
    ax.set_title("RRT search")

    # The edges in insertion order, and the lines drawn so far
    edges = [node for node in tree.nodes if node.parent is not None]
    drawn: list = []

    def update(frame):
        # Reveal up to step more edges, in insertion order
        target = min(len(edges), (frame + 1) * step)
        while len(drawn) < target:
            states = edges[len(drawn)].edge.states
            (line,) = ax.plot(
                states[:, x_axis], states[:, y_axis], color=tree_color, lw=0.4, zorder=1
            )
            drawn.append(line)

        # Once every edge is shown, the solution path
        if target >= len(edges) and planner.last_solution is not None:
            traj = planner.last_solution.trajectory
            ax.plot(traj.x[x_axis], traj.x[y_axis], color=path_color, lw=2.5, zorder=3)

        return drawn

    # One frame per batch of step edges, and a last one for the path
    n_frames = (len(edges) + step - 1) // step + 1
    anim = FuncAnimation(
        fig, update, frames=n_frames, interval=interval, blit=False, repeat=False
    )
    if save is not None:
        anim.save(save)

    maybe_show(plt, show)

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

    history = require_history(planner)

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    # The frame: the limits, labels and endpoints stay fixed while the tree rewires
    set_limits(ax, planner, x_axis, y_axis)
    label_axes(ax, planner.problem.sys, x_axis, y_axis)
    mark_endpoints(ax, planner, x_axis, y_axis)

    # The lines of the current frame, and the title that reports its progress
    tree_lines: list = []
    path_line = None
    title = ax.set_title(title_prefix)

    def update(frame):
        nonlocal path_line
        snapshot = history[frame]

        # Clear the previous frame's tree and path
        clear_search_artists(tree_lines, path_line)
        path_line = None

        # The tree as recorded, rewired edges included
        for edge_states in snapshot.tree_edges:
            (line,) = ax.plot(
                edge_states[:, x_axis],
                edge_states[:, y_axis],
                color=tree_color,
                lw=0.35,
                zorder=1,
            )
            tree_lines.append(line)

        # The best path to the goal so far
        if snapshot.path_states is not None:
            (path_line,) = ax.plot(
                snapshot.path_states[:, x_axis],
                snapshot.path_states[:, y_axis],
                color=path_color,
                lw=2.5,
                zorder=3,
            )

        # The iteration, and the best cost once the goal is reached
        if snapshot.reached_goal and snapshot.best_cost is not None:
            title.set_text(
                f"{title_prefix}  iter={snapshot.iteration}  "
                f"best cost={snapshot.best_cost:.2f}"
            )
        else:
            title.set_text(f"{title_prefix}  iter={snapshot.iteration}  searching...")

        return tree_lines + ([path_line] if path_line is not None else [])

    # One frame per recorded snapshot, looping
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

    maybe_show(plt, show)

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
    # The tree: each edge drawn along the states its extension swept
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

    # The path to the goal node, once the goal is reached
    path_line = None
    path_states = best_path_states(planner)
    if path_states is not None:
        (path_line,) = ax.plot(
            path_states[:, x_axis],
            path_states[:, y_axis],
            color=path_color,
            lw=2.5,
            zorder=3,
        )

    return tree_lines, path_line


_draw_search_state = draw_search_state  # ripple


###############################################################################
# Internal machinery
###############################################################################


def require_tree(planner):
    """Refuse a planner with no search tree yet; return its tree."""
    if getattr(planner, "tree", None) is None:
        raise ValueError("No search tree yet; call solve() first")

    return planner.tree


def solution_waypoints(planner):
    """Tree-node polyline from root to ``solution_node`` (no edge subsampling)."""
    node = getattr(planner, "solution_node", None)
    tree = getattr(planner, "tree", None)
    if node is None or tree is None:
        return None

    # The chain of nodes from the goal node back to the root, then reversed
    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
    chain.reverse()

    # The root, then each node's state along the chain
    points = [np.asarray(tree.root.x, dtype=float)]
    for nd in chain:
        points.append(np.asarray(nd.x, dtype=float))
    waypoints = np.asarray(points)

    return waypoints


def mark_endpoints(ax, planner, x_axis, y_axis):
    """Mark the start, the goal and the goal radius."""
    import matplotlib.pyplot as plt

    problem = planner.problem

    # The start
    x0 = np.asarray(problem.x_start, dtype=float)
    ax.plot([x0[x_axis]], [x0[y_axis]], "ks", zorder=4, label="start")

    # The goal, and the circle of the goal region around it
    if problem.x_goal is not None:
        xg = np.asarray(problem.x_goal, dtype=float)
        ax.plot([xg[x_axis]], [xg[y_axis]], "r*", markersize=13, zorder=4, label="goal")
        radius = goal_radius(planner)
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


_mark_endpoints = mark_endpoints  # ripple


def goal_radius(planner):
    """The radius of a ball goal region, else the goal tolerance around a goal state, else ``None``."""
    # Lazy import: the core sets load on first use, not with the plots
    from minilink.core.sets import BallSet

    Xf = planner.problem.Xf
    if isinstance(Xf, BallSet):
        return float(Xf.radius)

    if planner.problem.x_goal is not None:
        return float(planner.options.goal_tolerance)

    return None


def label_axes(ax, sys, x_axis, y_axis):
    """Label the axes with the state's labels and units."""
    labels = getattr(sys.state, "labels", None)
    units = getattr(sys.state, "units", None)

    def axis_label(i):
        name = labels[i] if labels and i < len(labels) else f"x[{i}]"
        unit = f" [{units[i]}]" if units and i < len(units) and units[i] else ""
        return name + unit

    ax.set_xlabel(axis_label(x_axis))
    ax.set_ylabel(axis_label(y_axis))


_label_axes = label_axes  # ripple


def maybe_show(plt, show):
    """Show the figure when asked and the backend can (never under Agg)."""
    if show and plt.get_backend().lower() != "agg":
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.common.environment import is_blocking_needed

        plt.show(block=is_blocking_needed())


def set_limits(ax, planner, x_axis, y_axis):
    """Fit the axes to the tree's nodes and the goal, with a 5 % margin."""
    pts = np.array([node.x for node in planner.tree.nodes], dtype=float)
    if planner.problem.x_goal is not None:
        pts = np.vstack([pts, np.asarray(planner.problem.x_goal, dtype=float)])

    for axis, setter in ((x_axis, ax.set_xlim), (y_axis, ax.set_ylim)):
        lo = float(pts[:, axis].min())
        hi = float(pts[:, axis].max())
        pad = 0.05 * (hi - lo or 1.0)
        setter(lo - pad, hi + pad)


_set_limits = set_limits  # ripple


def require_history(planner):
    """Refuse a planner that recorded no search history; return its history."""
    history = getattr(planner, "history", None)
    if not history:
        raise ValueError(
            "No search history recorded; run solve with record_history=True"
        )

    return history


def clear_search_artists(tree_lines, path_line):
    """Remove the tree and path lines of the previous redraw."""
    for line in list(tree_lines):
        line.remove()
    tree_lines.clear()

    if path_line is not None:
        path_line.remove()


_clear_search_artists = clear_search_artists  # ripple


def best_path_states(planner):
    """The states along the path to the planner's goal node, or ``None`` before the goal."""
    node = getattr(planner, "solution_node", None)
    if node is None or not getattr(planner, "reached_goal", False):
        return None

    if hasattr(planner, "_path_states"):
        return planner._path_states(node)

    return None


def search_title(step):
    """The live plot's title for one search step."""
    if step.reached_goal and step.best_cost is not None:
        return (
            f"RRT search  phase={step.phase}  iter={step.iteration}  "
            f"nodes={step.tree_nodes}  best cost={step.best_cost:.2f}"
        )

    return (
        f"RRT search  phase={step.phase}  iter={step.iteration}  "
        f"nodes={step.tree_nodes}  searching..."
    )


_search_title = search_title  # ripple
