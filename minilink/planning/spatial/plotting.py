"""
Matplotlib visualization for :class:`~minilink.planning.spatial.scene.Scene`.

Imports matplotlib lazily inside :func:`plot_scene` so the planning layer
stays usable without a display backend until plotting is requested.
"""

from __future__ import annotations

import numpy as np

from minilink.core.geometry import Box, Inflated, Sphere, Union
from minilink.planning.spatial.collision import iter_probes
from minilink.planning.spatial.grid import FieldGrid, sample_grid
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


def _cost_field_array(
    grid: FieldGrid,
    *,
    log_scale: bool = False,
    vmax: float | None = None,
    vmax_percentile: float = 99.0,
    normalize_range: bool = True,
) -> tuple[np.ndarray, float, float, str]:
    """Return display values, vmin, vmax, and colorbar label."""
    z = np.asarray(grid.Z, dtype=float)
    label = "log(1 + cost)" if log_scale else "cost"
    if log_scale:
        z = np.log1p(np.maximum(z, 0.0))
    vmin = 0.0
    if normalize_range:
        finite = z[np.isfinite(z)]
        if finite.size:
            vmin = float(np.min(finite))
            z = z - vmin
    if vmax is None:
        finite = z[np.isfinite(z)]
        if finite.size:
            vmax = float(np.percentile(finite, vmax_percentile))
        else:
            vmax = 1.0
    if vmax <= 0.0:
        vmax = 1.0
    return z, vmin, vmax, label


def cost_field_cmap():
    """Low cost (blue) to high cost (red) — matplotlib ``turbo``."""
    import matplotlib.pyplot as plt

    return plt.get_cmap("turbo")


def _cost_field_norm(vmax: float, *, gamma: float = 0.55):
    from matplotlib.colors import PowerNorm

    return PowerNorm(gamma=gamma, vmin=0.0, vmax=vmax)


def _resolve_cost_cmap(cmap):
    return cost_field_cmap() if cmap is None else cmap


def _label_cost_colorbar(cbar, vmin, *, normalize_range, log_scale):
    if normalize_range and log_scale:
        ticks = cbar.get_ticks()
        cbar.set_ticks(ticks)
        cbar.set_ticklabels([f"{t + vmin:.1f}" for t in ticks])


def plot_cost_field(
    grid: FieldGrid,
    *,
    ax=None,
    cmap=None,
    log_scale: bool = False,
    vmax: float | None = None,
    vmax_percentile: float = 99.0,
    gamma: float = 0.55,
    normalize_range: bool = True,
    facecolor: str = "white",
    colorbar: bool = True,
    colorbar_label: str | None = None,
    show: bool = True,
    figsize=(6.0, 6.0),
    title: str | None = "Spatial cost",
    equal_aspect: bool = True,
):
    """Plot a workspace cost heatmap. Returns ``(fig, ax)``."""
    import matplotlib.pyplot as plt

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize, frameon=True, facecolor=facecolor)
    else:
        fig = ax.figure

    z, vmin, vmax, default_label = _cost_field_array(
        grid,
        log_scale=log_scale,
        vmax=vmax,
        vmax_percentile=vmax_percentile,
        normalize_range=normalize_range,
    )
    cmap_resolved = _resolve_cost_cmap(cmap)
    norm = _cost_field_norm(vmax, gamma=gamma)
    ax.set_facecolor(facecolor)
    image = ax.imshow(
        z,
        extent=grid.extent,
        origin="lower",
        cmap=cmap_resolved,
        norm=norm,
        aspect="auto",
        zorder=1,
    )
    if colorbar:
        label = colorbar_label if colorbar_label is not None else default_label
        cbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04, label=label)
        _label_cost_colorbar(
            cbar, vmin, normalize_range=normalize_range, log_scale=log_scale
        )
    ax.set_xlim(grid.extent[0], grid.extent[1])
    ax.set_ylim(grid.extent[2], grid.extent[3])
    if equal_aspect:
        ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.grid(True, alpha=0.25, zorder=0)

    if show and plt.get_backend().lower() != "agg":
        plt.show()

    return fig, ax


def plot_cost_field_3d(
    grid: FieldGrid,
    *,
    ax=None,
    cmap=None,
    log_scale: bool = False,
    vmax: float | None = None,
    vmax_percentile: float = 99.0,
    gamma: float = 0.55,
    normalize_range: bool = True,
    facecolor: str = "white",
    colorbar: bool = True,
    colorbar_label: str | None = None,
    show: bool = True,
    figsize=(8.0, 6.0),
    title: str | None = "Spatial cost",
    elev: float = 35.0,
    azim: float = -60.0,
    alpha: float = 0.92,
    rstride: int = 1,
    cstride: int = 1,
):
    """Plot workspace cost as a 3-D surface ``z = cost(x, y)``. Returns ``(fig, ax)``."""
    import matplotlib.pyplot as plt

    z, vmin, vmax, default_label = _cost_field_array(
        grid,
        log_scale=log_scale,
        vmax=vmax,
        vmax_percentile=vmax_percentile,
        normalize_range=normalize_range,
    )
    cmap_resolved = _resolve_cost_cmap(cmap)
    norm = _cost_field_norm(vmax, gamma=gamma)
    x_grid, y_grid = np.meshgrid(grid.xs, grid.ys)

    if ax is None:
        fig = plt.figure(figsize=figsize, frameon=True, facecolor=facecolor)
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig = ax.figure

    ax.set_facecolor(facecolor)
    surface = ax.plot_surface(
        x_grid,
        y_grid,
        z,
        cmap=cmap_resolved,
        norm=norm,
        linewidth=0.0,
        antialiased=True,
        alpha=alpha,
        rstride=rstride,
        cstride=cstride,
    )
    if colorbar:
        label = colorbar_label if colorbar_label is not None else default_label
        cbar = fig.colorbar(surface, ax=ax, shrink=0.62, pad=0.08, label=label)
        _label_cost_colorbar(
            cbar, vmin, normalize_range=normalize_range, log_scale=log_scale
        )
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel(default_label)
    ax.set_title(title)
    ax.view_init(elev=elev, azim=azim)

    if show and plt.get_backend().lower() != "agg":
        plt.show()

    return fig, ax


def plot_cost_field_exports(
    layers,
    *,
    track=None,
    scene=None,
    overlay_bounds=None,
    traj=None,
    log_scale: bool = True,
    cmap=None,
    save_dir=None,
    show: bool = True,
    figsize_2d=(9.0, 7.0),
    figsize_3d=(9.0, 7.0),
    surface_stride: int = 2,
    **plot_kwargs,
):
    """
    Plot each named cost layer in 2-D and 3-D with the same colormap.

    Parameters
    ----------
    layers : dict[str, FieldGrid]
        e.g. ``{"path": g, "corridor": g, "obstacle": g, "combined": g}``.
    track, scene
        Optional overlays drawn on the 2-D figures (``overlay_bounds`` for limits).
    traj
        Optional executed path ``(x, y)`` polyline on 2-D figures.
    save_dir
        If set, write ``{name}_2d.png`` and ``{name}_3d.png`` for each layer.
    **plot_kwargs
        Forwarded to :func:`plot_cost_field` / :func:`plot_cost_field_3d`
        (``vmax_percentile``, ``gamma``, etc.).

    Returns
    -------
    dict
        ``{name: {"2d": (fig, ax), "3d": (fig, ax)}}``.
    """
    from pathlib import Path

    import matplotlib.pyplot as plt

    cmap_resolved = _resolve_cost_cmap(cmap)
    shared = {"cmap": cmap_resolved, "log_scale": log_scale, **plot_kwargs}
    out = {}
    save_path = Path(save_dir) if save_dir is not None else None
    if save_path is not None:
        save_path.mkdir(parents=True, exist_ok=True)

    for name, grid in layers.items():
        title_2d = name.replace("_", " ").title()
        title_2d_full = f"{title_2d} cost (2D)"
        title_3d_full = f"{title_2d} cost (3D)"
        fig2d, ax2d = plot_cost_field(
            grid,
            ax=None,
            title=title_2d_full,
            show=False,
            figsize=figsize_2d,
            **shared,
        )
        if track is not None:
            track.plot(show=False, ax=ax2d, bounds=overlay_bounds, title="")
        if scene is not None:
            scene.plot(
                show=False,
                ax=ax2d,
                bounds=overlay_bounds,
                show_density=False,
                title="",
            )
        if traj is not None:
            ax2d.plot(
                traj[0, :],
                traj[1, :],
                color="k",
                linewidth=1.0,
                alpha=0.85,
                zorder=7,
            )
        ax2d.set_title(title_2d_full)
        fig2d.tight_layout()

        fig3d, ax3d = plot_cost_field_3d(
            grid,
            ax=None,
            title=title_3d_full,
            show=False,
            figsize=figsize_3d,
            rstride=surface_stride,
            cstride=surface_stride,
            **shared,
        )
        fig3d.tight_layout()

        if save_path is not None:
            fig2d.savefig(save_path / f"{name}_2d.png", dpi=160, bbox_inches="tight")
            fig3d.savefig(save_path / f"{name}_3d.png", dpi=160, bbox_inches="tight")

        out[name] = {"2d": (fig2d, ax2d), "3d": (fig3d, ax3d)}

        if not show:
            plt.close(fig2d)
            plt.close(fig3d)

    if show and plt.get_backend().lower() != "agg":
        plt.show()

    return out


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
    centerline_color: str = "#5c6570",
    corridor_edge_color: str = "#9aa3af",
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

    ax.plot(
        upper[:, 0],
        upper[:, 1],
        color=corridor_edge_color,
        linewidth=1.0,
        zorder=2,
        label="corridor",
    )
    ax.plot(
        lower[:, 0],
        lower[:, 1],
        color=corridor_edge_color,
        linewidth=1.0,
        zorder=2,
    )
    ax.plot(
        center[:, 0],
        center[:, 1],
        color=centerline_color,
        linewidth=1.2,
        linestyle=(0, (5, 4)),
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
