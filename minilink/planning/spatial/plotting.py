"""
Matplotlib visualization for :class:`~minilink.planning.spatial.scene.Scene`.

Imports matplotlib lazily inside :func:`plot_scene` so the planning layer
stays usable without a display backend until plotting is requested.
"""

from __future__ import annotations

import numpy as np

from minilink.core.geometry import Box, Inflated, Sphere, Union
from minilink.planning.spatial.collision import iter_probes
from minilink.planning.spatial.grid import sample_grid
from minilink.planning.spatial.workspace_fields import GaussianField

# Public API


def plot_scene(
    scene,
    *,
    bounds=None,
    grid=(120, 120),
    show_obstacles: bool = True,
    show_density: bool = True,
    show_clearance_contour: bool = False,
    body=None,
    x=None,
    states=None,
    show: bool = True,
    ax=None,
    figsize=(6.0, 6.0),
    t: float = 0.0,
    params=None,
    obstacle_color: str = "#c44e52",
    obstacle_alpha: float = 0.45,
    density_cmap: str = "Oranges",
    density_alpha: float = 0.55,
    body_color: str = "#4c72b0",
    body_alpha: float = 0.35,
    point_marker_size: float = 9.0,
    title: str | None = None,
    equal_aspect: bool = True,
):
    """
    Plot hard obstacles and soft workspace fields for a 2-D scene.

    Pass ``body`` with ``x`` (one state) or ``states`` (several) to overlay
    collision discs at the placed body poses.

    Returns ``(fig, ax)``. Matplotlib is imported on first call.
    """
    import matplotlib.pyplot as plt

    dim = _workspace_dim(scene)
    if dim != 2:
        raise ValueError("plot_scene supports 2-D workspaces only")

    plot_bounds = bounds if bounds is not None else _default_bounds(scene)
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    if show_density and scene.workspace_fields:
        density = sample_grid(
            scene.cost_density,
            plot_bounds,
            grid=grid,
            t=t,
            params=params,
        )
        ax.imshow(
            density.Z,
            extent=density.extent,
            origin="lower",
            cmap=density_cmap,
            alpha=density_alpha,
            aspect="auto",
            zorder=1,
        )

    if show_clearance_contour and scene.obstacles:
        from minilink.core.geometry import Union as ShapeUnion

        clearance = sample_grid(
            ShapeUnion(scene.obstacles).sdf,
            plot_bounds,
            grid=grid,
            t=t,
            params=params,
        )
        ax.contour(
            clearance.xs,
            clearance.ys,
            clearance.Z,
            levels=[0.0],
            colors="k",
            linewidths=1.0,
            zorder=3,
        )

    if show_obstacles:
        for obstacle in scene.obstacles:
            _draw_shape(ax, obstacle, obstacle_color, obstacle_alpha)

    for field in scene.workspace_fields:
        _draw_field_marker(ax, field)

    body_states = _coerce_body_states(x, states)
    if body is not None:
        for state in body_states:
            _draw_body(
                ax,
                body,
                state,
                t,
                params,
                body_color,
                body_alpha,
                point_marker_size,
            )

    (x_lo, x_hi), (y_lo, y_hi) = plot_bounds
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    if equal_aspect:
        ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title if title is not None else "Scene")
    ax.grid(True, alpha=0.25, zorder=0)

    if show and plt.get_backend().lower() != "agg":
        plt.show()

    return fig, ax


def _workspace_dim(scene) -> int:
    for obstacle in scene.obstacles:
        return obstacle.dim
    for field in scene.workspace_fields:
        if isinstance(field, GaussianField):
            return int(field.center.size)
    return 2


def _default_bounds(scene, *, padding: float = 0.75):
    boxes = []
    for obstacle in scene.obstacles:
        boxes.append(_shape_bounds(obstacle))
    for field in scene.workspace_fields:
        boxes.append(_field_bounds(field))
    if not boxes:
        half = 5.0 + padding
        return ((-half, half), (-half, half))

    x_lo = min(box[0][0] for box in boxes) - padding
    x_hi = max(box[0][1] for box in boxes) + padding
    y_lo = min(box[1][0] for box in boxes) - padding
    y_hi = max(box[1][1] for box in boxes) + padding
    return ((x_lo, x_hi), (y_lo, y_hi))


def _shape_bounds(shape):
    if isinstance(shape, Sphere):
        center = np.asarray(shape.center, dtype=float)
        radius = float(shape.radius)
        return (
            (float(center[0] - radius), float(center[0] + radius)),
            (float(center[1] - radius), float(center[1] + radius)),
        )
    if isinstance(shape, Box):
        lower = np.asarray(shape.lower, dtype=float)
        upper = np.asarray(shape.upper, dtype=float)
        return (
            (float(lower[0]), float(upper[0])),
            (float(lower[1]), float(upper[1])),
        )
    if isinstance(shape, Inflated):
        (x_bounds, y_bounds) = _shape_bounds(shape.base)
        radius = float(shape.radius)
        return (
            (x_bounds[0] - radius, x_bounds[1] + radius),
            (y_bounds[0] - radius, y_bounds[1] + radius),
        )
    if isinstance(shape, Union):
        boxes = [_shape_bounds(member) for member in shape.shapes]
        return (
            (min(box[0][0] for box in boxes), max(box[0][1] for box in boxes)),
            (min(box[1][0] for box in boxes), max(box[1][1] for box in boxes)),
        )
    raise TypeError(f"unsupported shape type for plotting: {type(shape).__name__}")


def _field_bounds(field):
    if isinstance(field, GaussianField):
        center = np.asarray(field.center, dtype=float)
        span = 3.0 * float(field.sigma)
        return (
            (float(center[0] - span), float(center[0] + span)),
            (float(center[1] - span), float(center[1] + span)),
        )
    raise TypeError(f"unsupported field type for plotting: {type(field).__name__}")


def _draw_shape(ax, shape, color, alpha):
    import matplotlib.patches as mpatches

    if isinstance(shape, Sphere):
        center = np.asarray(shape.center, dtype=float)
        ax.add_patch(
            mpatches.Circle(
                (float(center[0]), float(center[1])),
                float(shape.radius),
                facecolor=color,
                edgecolor="k",
                linewidth=0.8,
                alpha=alpha,
                zorder=4,
            )
        )
        return
    if isinstance(shape, Box):
        lower = np.asarray(shape.lower, dtype=float)
        upper = np.asarray(shape.upper, dtype=float)
        ax.add_patch(
            mpatches.Rectangle(
                (float(lower[0]), float(lower[1])),
                float(upper[0] - lower[0]),
                float(upper[1] - lower[1]),
                facecolor=color,
                edgecolor="k",
                linewidth=0.8,
                alpha=alpha,
                zorder=4,
            )
        )
        return
    if isinstance(shape, Inflated):
        if isinstance(shape.base, Sphere):
            center = np.asarray(shape.base.center, dtype=float)
            radius = float(shape.base.radius) + float(shape.radius)
            ax.add_patch(
                mpatches.Circle(
                    (float(center[0]), float(center[1])),
                    radius,
                    facecolor=color,
                    edgecolor="k",
                    linewidth=0.8,
                    alpha=alpha,
                    zorder=4,
                )
            )
            return
        _draw_shape(ax, shape.base, color, alpha)
        return
    if isinstance(shape, Union):
        for member in shape.shapes:
            _draw_shape(ax, member, color, alpha)
        return
    raise TypeError(f"unsupported shape type for plotting: {type(shape).__name__}")


def _draw_field_marker(ax, field):
    if isinstance(field, GaussianField):
        center = np.asarray(field.center, dtype=float)
        ax.plot(
            float(center[0]),
            float(center[1]),
            marker="x",
            color="C1",
            markersize=8,
            markeredgewidth=1.5,
            zorder=5,
        )
        return
    raise TypeError(f"unsupported field type for plotting: {type(field).__name__}")


def _coerce_body_states(x, states):
    if states is not None:
        return [np.asarray(state, dtype=float).reshape(-1) for state in states]
    if x is not None:
        return [np.asarray(x, dtype=float).reshape(-1)]
    return []


def _draw_body(ax, body, x, t, params, color, alpha, point_marker_size):
    import matplotlib.patches as mpatches

    for world, radius in iter_probes(body, x, None, t, params):
        wx, wy = float(world[0]), float(world[1])
        if float(radius) > 1e-12:
            ax.add_patch(
                mpatches.Circle(
                    (wx, wy),
                    float(radius),
                    facecolor=color,
                    edgecolor="k",
                    linewidth=0.8,
                    alpha=alpha,
                    zorder=6,
                )
            )
        else:
            ax.plot(
                wx,
                wy,
                marker="o",
                color=color,
                markersize=point_marker_size,
                markeredgecolor="k",
                markeredgewidth=1.0,
                zorder=6,
            )


def plot_track(
    track,
    *,
    bounds=None,
    n_samples: int = 200,
    show: bool = True,
    ax=None,
    figsize=(6.0, 6.0),
    centerline_color: str = "#2ca02c",
    corridor_color: str = "#98df8a",
    corridor_alpha: float = 0.35,
    waypoint_color: str = "#1f77b4",
    title: str | None = None,
    equal_aspect: bool = True,
):
    """
    Plot a :class:`~minilink.planning.spatial.track.ReferenceTrack` in 2-D.

    Returns ``(fig, ax)``. Matplotlib is imported on first call.
    """
    import matplotlib.pyplot as plt

    path = track.path
    if path.workspace_dim != 2:
        raise ValueError("plot_track supports 2-D paths only")

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True)
    else:
        fig = ax.figure

    total = path.total_length
    if total <= 0.0:
        raise ValueError("path has zero length")

    ss = np.linspace(0.0, total, n_samples)
    center = np.array([path.sample(s) for s in ss])
    tangents = np.array([path.tangent(s) for s in ss])
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)
    half = float(track.half_width)
    upper = center + half * normals
    lower = center - half * normals
    corridor = np.vstack([upper, lower[::-1]])

    ax.fill(
        corridor[:, 0],
        corridor[:, 1],
        color=corridor_color,
        alpha=corridor_alpha,
        zorder=2,
        label="corridor",
    )
    ax.plot(
        center[:, 0],
        center[:, 1],
        color=centerline_color,
        linewidth=2.0,
        zorder=3,
        label="path",
    )

    if hasattr(path, "waypoints"):
        wp = np.asarray(path.waypoints)
        ax.scatter(
            wp[:, 0],
            wp[:, 1],
            color=waypoint_color,
            s=36,
            zorder=4,
            label="waypoints",
        )

    if bounds is not None:
        (xlo, xhi), (ylo, yhi) = bounds
        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
    else:
        pad = half + 0.5
        ax.set_xlim(center[:, 0].min() - pad, center[:, 0].max() + pad)
        ax.set_ylim(center[:, 1].min() - pad, center[:, 1].max() + pad)

    if equal_aspect:
        ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title if title is not None else "Reference track")
    ax.legend(loc="best", fontsize=8)

    if show:
        plt.show()
    return fig, ax
