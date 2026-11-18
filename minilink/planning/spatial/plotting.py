"""Spatial plots: a scene, a reference track and a workspace cost field."""

from __future__ import annotations

import numpy as np

from minilink.core.geometry import Box, Inflated, Sphere, Union
from minilink.graphical.figure import (
    Contour,
    FigureSpec,
    Heatmap,
    Panel,
    Surface,
    Trace,
    draw,
)
from minilink.planning.spatial.collision import iter_probes
from minilink.planning.spatial.grid import sample_grid
from minilink.planning.spatial.workspace_fields import GaussianField

# The colours of the scene and the track
OBSTACLE_COLOR = "#c44e52"
BODY_COLOR = "#4c72b0"
FIELD_MARKER_COLOR = "#ff7f0e"
CENTERLINE_COLOR = "#5c6570"
CORRIDOR_EDGE_COLOR = "#9aa3af"
WAYPOINT_COLOR = "#1f77b4"

# Low cost (blue) to high cost (red)
COST_FIELD_CMAP = "turbo"


###############################################################################
# Public API
###############################################################################


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
    t: float = 0.0,
    params=None,
    title: str | None = None,
    figsize=(6.0, 6.0),
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Plot hard obstacles and soft workspace fields for a 2-D scene.

    Pass ``body`` with ``x`` (one state) or ``states`` (several) to overlay
    collision discs at the placed body poses.
    """
    spec = scene_figure(
        scene,
        bounds=bounds,
        grid=grid,
        show_obstacles=show_obstacles,
        show_density=show_density,
        show_clearance_contour=show_clearance_contour,
        body=body,
        x=x,
        states=states,
        t=t,
        params=params,
        title=title,
        figsize=figsize,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def plot_track(
    track,
    *,
    bounds=None,
    n_samples: int = 200,
    show_waypoints: bool = True,
    title: str | None = None,
    figsize=(6.0, 6.0),
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """Plot a :class:`~minilink.planning.spatial.track.ReferenceTrack` in 2-D: its corridor, centerline and waypoints."""
    spec = track_figure(
        track,
        bounds=bounds,
        n_samples=n_samples,
        show_waypoints=show_waypoints,
        title=title,
        figsize=figsize,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def plot_cost_field(
    costs,
    *,
    bounds,
    grid=(200, 200),
    state_dim: int = 2,
    u=None,
    t: float = 0.0,
    params=None,
    cmap=COST_FIELD_CMAP,
    log_scale: bool = False,
    vmax: float | None = None,
    gamma: float = 0.55,
    title: str | None = "Spatial cost",
    figsize=(6.0, 6.0),
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """
    Plot the summed cost of ``FieldCost`` terms over a workspace grid, as a heatmap.

    Each cell is the cost of a workspace point ``p = (x, y)`` seen by a point
    probe: build the costs with ``mount(point_probe(), on=sys)``, so the field
    does not depend on heading or body extent, and pass the plant's
    ``state_dim`` (``sys.n``). ``log_scale`` draws ``log(1 + cost)``; the
    colours scale as ``(z / vmax)^γ``.
    """
    xs, ys, Z = sampled_costs(
        costs, bounds=bounds, grid=grid, state_dim=state_dim, u=u, t=t, params=params
    )
    spec = cost_field_figure(
        xs,
        ys,
        Z,
        cmap=cmap,
        log_scale=log_scale,
        vmax=vmax,
        gamma=gamma,
        title=title,
        figsize=figsize,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def plot_cost_field_3d(
    costs,
    *,
    bounds,
    grid=(200, 200),
    state_dim: int = 2,
    u=None,
    t: float = 0.0,
    params=None,
    cmap=COST_FIELD_CMAP,
    log_scale: bool = False,
    vmax: float | None = None,
    gamma: float = 0.55,
    title: str | None = "Spatial cost",
    figsize=(8.0, 6.0),
    renderer="matplotlib",
    show=None,
    ax=None,
):
    """Plot the summed cost of ``FieldCost`` terms as a 3-D surface ``z = cost(x, y)`` (see :func:`plot_cost_field`)."""
    xs, ys, Z = sampled_costs(
        costs, bounds=bounds, grid=grid, state_dim=state_dim, u=u, t=t, params=params
    )
    spec = cost_surface_figure(
        xs,
        ys,
        Z,
        cmap=cmap,
        log_scale=log_scale,
        vmax=vmax,
        gamma=gamma,
        title=title,
        figsize=figsize,
        stride=1,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def scene_figure(
    scene,
    *,
    bounds,
    grid,
    show_obstacles,
    show_density,
    show_clearance_contour,
    body,
    x,
    states,
    t,
    params,
    title,
    figsize,
):
    """The scene's density, clearance contour, obstacles, field markers and body, as a description."""
    require_planar_scene(scene)
    plot_bounds = bounds if bounds is not None else default_bounds(scene)
    layers = []

    # The soft fields: their summed cost density as a heatmap
    if show_density and scene.workspace_fields:
        xs, ys, density = sample_grid(
            scene.cost_density, plot_bounds, grid=grid, t=t, params=params
        )
        layers.append(
            Heatmap(
                xs,
                ys,
                density,
                cmap="Oranges",
                alpha=0.55,
                shading="cells",
                colorbar=None,
            )
        )

    # The obstacles' boundary: the zero level set of their union's signed distance
    if show_clearance_contour and scene.obstacles:
        xs, ys, clearance = sample_grid(
            Union(scene.obstacles).sdf, plot_bounds, grid=grid, t=t, params=params
        )
        layers.append(Contour(xs, ys, clearance, levels=(0.0,), color="black"))

    # The hard obstacles, and a marker at each soft field's source
    if show_obstacles:
        for obstacle in scene.obstacles:
            layers += disc_layers(shape_outlines(obstacle), OBSTACLE_COLOR, 0.45)
    for field in scene.workspace_fields:
        layers.append(field_marker(field))

    # The body's collision discs, placed at each given state
    if body is not None:
        for state in coerce_body_states(x, states):
            layers += body_layers(body, state, t, params)

    (x_lo, x_hi), (y_lo, y_hi) = plot_bounds
    panel = Panel(
        layers=tuple(layers),
        title="Scene" if title is None else title,
        x_label="x",
        y_label="y",
        x_lim=(x_lo, x_hi),
        y_lim=(y_lo, y_hi),
        equal_aspect=True,
        legend=False,
    )

    return FigureSpec(panels=(panel,), figsize=figsize)


def track_figure(track, *, bounds, n_samples, show_waypoints, title, figsize):
    """The track's corridor edges, dashed centerline and waypoints, as a description."""
    path = track.path
    require_planar_path(path)
    total = path.total_length
    require_positive_length(total)

    # The centerline c(s), sampled by arc length, and its left normal n = (−t_y, t_x)
    ss = np.linspace(0.0, total, n_samples)
    center = np.array([path.at(s) for s in ss])
    tangents = np.array([path.tangent(s) for s in ss])
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)

    # The corridor edges c(s) ± w n(s)
    w = float(track.half_width)
    upper = center + w * normals
    lower = center - w * normals

    # The corridor edges, then the dashed centerline
    edge = {"color": CORRIDOR_EDGE_COLOR, "width": 1.0}
    layers = [
        Trace(upper[:, 0], upper[:, 1], label="corridor", **edge),
        Trace(lower[:, 0], lower[:, 1], **edge),
        Trace(
            center[:, 0],
            center[:, 1],
            color=CENTERLINE_COLOR,
            width=1.2,
            dash="dash",
            label="path",
        ),
    ]

    # The waypoints, when the path keeps them
    if show_waypoints and hasattr(path, "waypoints"):
        wp = np.asarray(path.waypoints)
        layers.append(
            Trace(
                wp[:, 0],
                wp[:, 1],
                color=WAYPOINT_COLOR,
                mode="markers",
                label="waypoints",
            )
        )

    # The limits: the given bounds, else the centerline padded by w + 0.5
    if bounds is None:
        pad = w + 0.5
        bounds = (
            (center[:, 0].min() - pad, center[:, 0].max() + pad),
            (center[:, 1].min() - pad, center[:, 1].max() + pad),
        )
    panel = Panel(
        layers=tuple(layers),
        title="Reference track" if title is None else title,
        x_label="x",
        y_label="y",
        x_lim=tuple(bounds[0]),
        y_lim=tuple(bounds[1]),
        equal_aspect=True,
    )

    return FigureSpec(panels=(panel,), figsize=figsize)


def cost_field_figure(xs, ys, Z, *, cmap, log_scale, vmax, gamma, title, figsize):
    """The cost field as a heatmap on a power-law colour scale, as a description."""
    z, vmin, vmax, label = cost_field_array(Z, log_scale=log_scale, vmax=vmax)
    heatmap = Heatmap(
        xs,
        ys,
        z,
        cmap=cmap,
        vmin=0.0,
        vmax=vmax,
        shading="cells",
        colorbar=label,
        gamma=gamma,
        tick_offset=vmin if log_scale else 0.0,
    )
    panel = Panel(
        layers=(heatmap,),
        title=title or "",
        x_label="x",
        y_label="y",
        x_lim=(float(xs[0]), float(xs[-1])),
        y_lim=(float(ys[0]), float(ys[-1])),
        equal_aspect=True,
    )

    return FigureSpec(panels=(panel,), figsize=figsize)


def cost_surface_figure(
    xs, ys, Z, *, cmap, log_scale, vmax, gamma, title, figsize, stride
):
    """The cost field as a 3-D surface on a power-law colour scale, as a description."""
    z, vmin, vmax, label = cost_field_array(Z, log_scale=log_scale, vmax=vmax)
    surface = Surface(
        xs,
        ys,
        z,
        cmap=cmap,
        vmin=0.0,
        vmax=vmax,
        colorbar=label,
        alpha=0.92,
        gamma=gamma,
        tick_offset=vmin if log_scale else 0.0,
        stride=stride,
    )
    panel = Panel(
        layers=(surface,),
        title=title or "",
        x_label="x",
        y_label="y",
        z_label=label,
        is_3d=True,
    )

    return FigureSpec(panels=(panel,), figsize=figsize)


###############################################################################
# Internal machinery
###############################################################################


# The scene's extent and drawings


def require_planar_scene(scene):
    """Refuse a scene whose workspace is not 2-D."""
    dim = workspace_dim(scene)
    if dim != 2:
        raise ValueError("plot_scene supports 2-D workspaces only")


def workspace_dim(scene) -> int:
    """The workspace dimension: the first obstacle's, else the first Gaussian field's, else 2."""
    for obstacle in scene.obstacles:
        return obstacle.dim
    for field in scene.workspace_fields:
        if isinstance(field, GaussianField):
            return int(field.center.size)
    return 2


def default_bounds(scene, *, padding: float = 0.75):
    """The box around every obstacle and field, padded; a padded 10 × 10 window when empty."""
    boxes = []
    for obstacle in scene.obstacles:
        boxes.append(shape_bounds(obstacle))
    for field in scene.workspace_fields:
        boxes.append(field_bounds(field))
    if not boxes:
        half = 5.0 + padding
        return ((-half, half), (-half, half))

    x_lo = min(box[0][0] for box in boxes) - padding
    x_hi = max(box[0][1] for box in boxes) + padding
    y_lo = min(box[1][0] for box in boxes) - padding
    y_hi = max(box[1][1] for box in boxes) + padding
    return ((x_lo, x_hi), (y_lo, y_hi))


def shape_bounds(shape):
    """The ``((x_lo, x_hi), (y_lo, y_hi))`` box of a sphere, box, inflated shape or union."""
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
        (x_bounds, y_bounds) = shape_bounds(shape.base)
        radius = float(shape.radius)
        return (
            (x_bounds[0] - radius, x_bounds[1] + radius),
            (y_bounds[0] - radius, y_bounds[1] + radius),
        )
    if isinstance(shape, Union):
        boxes = [shape_bounds(member) for member in shape.shapes]
        return (
            (min(box[0][0] for box in boxes), max(box[0][1] for box in boxes)),
            (min(box[1][0] for box in boxes), max(box[1][1] for box in boxes)),
        )
    raise TypeError(f"unsupported shape type for plotting: {type(shape).__name__}")


def field_bounds(field):
    """The box of a Gaussian field: its center ± 3σ."""
    if isinstance(field, GaussianField):
        center = np.asarray(field.center, dtype=float)
        span = 3.0 * float(field.sigma)
        return (
            (float(center[0] - span), float(center[0] + span)),
            (float(center[1] - span), float(center[1] + span)),
        )
    raise TypeError(f"unsupported field type for plotting: {type(field).__name__}")


def shape_outlines(shape, pad=0.0, n=100):
    """The closed outlines ``(x, y)`` of a sphere, box, inflated shape or union, grown by ``pad``."""
    if isinstance(shape, Sphere):
        center = np.asarray(shape.center, dtype=float)
        return [circle(center[0], center[1], float(shape.radius) + pad, n)]
    if isinstance(shape, Box):
        lower = np.asarray(shape.lower, dtype=float)
        upper = np.asarray(shape.upper, dtype=float)
        return [rounded_box(lower, upper, pad, n)]
    if isinstance(shape, Inflated):
        # An inflated shape is its base grown by the inflation radius
        return shape_outlines(shape.base, pad + float(shape.radius), n)
    if isinstance(shape, Union):
        return [
            outline
            for member in shape.shapes
            for outline in shape_outlines(member, pad, n)
        ]
    raise TypeError(f"unsupported shape type for plotting: {type(shape).__name__}")


def circle(cx, cy, r, n):
    """The ``n + 1`` points of the circle ``c + r (cos φ, sin φ)``, closed."""
    phi = np.linspace(0.0, 2.0 * np.pi, n + 1)
    x = cx + r * np.cos(phi)
    y = cy + r * np.sin(phi)

    return x, y


def rounded_box(lower, upper, pad, n):
    """A box grown by ``pad``: its sides pushed out by ``pad``, its corners quarter circles of radius ``pad``."""
    if pad <= 0.0:
        x = np.array([lower[0], upper[0], upper[0], lower[0], lower[0]])
        y = np.array([lower[1], lower[1], upper[1], upper[1], lower[1]])
        return x, y

    # One quarter circle per corner, counter-clockwise from the lower right
    quarter = np.linspace(0.0, 0.5 * np.pi, max(2, n // 4))
    corners = (
        (upper[0], lower[1], -0.5 * np.pi),
        (upper[0], upper[1], 0.0),
        (lower[0], upper[1], 0.5 * np.pi),
        (lower[0], lower[1], np.pi),
    )
    x = np.concatenate([cx + pad * np.cos(a + quarter) for cx, _, a in corners])
    y = np.concatenate([cy + pad * np.sin(a + quarter) for _, cy, a in corners])

    return np.r_[x, x[:1]], np.r_[y, y[:1]]


def disc_layers(outlines, color, alpha):
    """Each outline filled in ``color``, edged in black."""
    layers = []
    for x, y in outlines:
        layers.append(Trace(x, y, color=color, alpha=alpha, fill=True))
        layers.append(Trace(x, y, color="black", width=0.8, alpha=alpha))

    return layers


def field_marker(field):
    """A cross at a Gaussian field's center."""
    if isinstance(field, GaussianField):
        center = np.asarray(field.center, dtype=float)
        return Trace(
            [float(center[0])],
            [float(center[1])],
            color=FIELD_MARKER_COLOR,
            mode="markers",
            marker="x",
            size=8.0,
        )
    raise TypeError(f"unsupported field type for plotting: {type(field).__name__}")


def coerce_body_states(x, states):
    """The states to place the body at, each a flat float vector: ``states``, else ``[x]``."""
    if states is not None:
        return [np.asarray(state, dtype=float).reshape(-1) for state in states]
    if x is not None:
        return [np.asarray(x, dtype=float).reshape(-1)]
    return []


def body_layers(body, x, t, params):
    """The body's collision probes at state ``x``: a disc each, a dot for a point probe."""
    layers = []
    for world, radius in iter_probes(body, x, None, t, params):
        wx, wy = float(world[0]), float(world[1])
        # A radius ≤ 10⁻¹² is a point probe
        if float(radius) > 1e-12:
            layers += disc_layers(
                [circle(wx, wy, float(radius), 100)], BODY_COLOR, 0.35
            )
        else:
            layers.append(Trace([wx], [wy], color=BODY_COLOR, mode="markers", size=9.0))

    return layers


# The track's refusals


def require_planar_path(path):
    """Refuse a path whose workspace is not 2-D."""
    if path.workspace_dim != 2:
        raise ValueError("plot_track supports 2-D paths only")


def require_positive_length(total):
    """Refuse a path of zero length."""
    if total <= 0.0:
        raise ValueError("path has zero length")


# The cost field's samples and display scale


def sampled_costs(costs, *, bounds, grid, state_dim, u, t, params):
    """
    The summed cost of ``costs`` at each workspace point of the grid: ``(xs, ys, Z)``.

    Each point ``p = (x, y)`` fills the first two entries of a zero state of
    dimension ``state_dim``; the input is ``u`` (zero when ``None``).
    """
    require_costs(costs)

    # The held input, and the state whose first two entries carry the point
    u_use = np.zeros(1) if u is None else u
    x = np.zeros(int(state_dim))

    def fn(p, t_query=0.0, params_query=None):
        x[0] = p[0]
        x[1] = p[1]

        # g(p) = Σ_c g_c(x(p), u, t), summed in the terms' order
        g = sum(float(c.g(x, u_use, t=t_query, params=params_query)) for c in costs)

        return g

    return sample_grid(fn, bounds, grid=grid, t=t, params=params)


def require_costs(costs):
    """Refuse a cost field with no cost term."""
    if not costs:
        raise ValueError("a cost field needs at least one cost term")


def cost_field_array(Z, *, log_scale=False, vmax=None, vmax_percentile=99.0):
    """The displayed values, starting at 0, their shift ``vmin``, the top of the scale and the colour bar's label."""
    z = np.asarray(Z, dtype=float)
    label = "log(1 + cost)" if log_scale else "cost"

    # z ← log(1 + max(z, 0)) on a log scale
    if log_scale:
        z = np.log1p(np.maximum(z, 0.0))

    # z ← z − min z over the finite values, so the scale starts at 0
    vmin = 0.0
    finite = z[np.isfinite(z)]
    if finite.size:
        vmin = float(np.min(finite))
        z = z - vmin

    # The top of the scale: vmax, else a percentile of the finite values, and 1 when ≤ 0
    if vmax is None:
        finite = z[np.isfinite(z)]
        if finite.size:
            vmax = float(np.percentile(finite, vmax_percentile))
        else:
            vmax = 1.0
    if vmax <= 0.0:
        vmax = 1.0

    return z, vmin, vmax, label
