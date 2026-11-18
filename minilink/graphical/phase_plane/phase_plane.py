"""Phase plane: a system's vector field over two state coordinates, with an optional trajectory."""

from __future__ import annotations

import operator

import numpy as np

from minilink.graphical.common.labels import state_axis_label
from minilink.graphical.common.matplotlib_style import FIGSIZE_BASE
from minilink.graphical.common.plot_result import PlotResult
from minilink.graphical.figure import Arrows, FigureSpec, Panel, Trace, draw

###############################################################################
# Public API
###############################################################################


def plot_phase_plane(
    sys,
    traj=None,
    *,
    x_axis: int = 0,
    y_axis: int | None = None,
    u=None,
    x_ref=None,
    t: float = 0.0,
    bounds=None,
    grid_shape: tuple[int, int] = (21, 21),
    params=None,
    title: str | None = None,
    streamplot: bool = False,
    figsize=None,
    renderer: str = "matplotlib",
    show=None,
    ax=None,
) -> PlotResult:
    """Plot a phase-plane vector field with an optional trajectory overlay.

    The keywords not listed here are those of :func:`phase_plane_figure`.

    Parameters
    ----------
    sys : DynamicSystem
        Continuous-time model whose ``f`` defines the vector field (leaf
        subclass or wired diagram with stacked ``f``).
    streamplot : bool, optional
        Streamlines through the field instead of one arrow per grid node.
    figsize : tuple, optional
        The figure's size in inches.
    renderer, show, ax : optional
        ``"matplotlib"`` or ``"plotly"``; show the figure, by default unless it
        is drawn on ``ax`` (matplotlib axes, a drawn plot's ``result.axes``).
    """
    spec = phase_plane_figure(
        sys,
        traj,
        x_axis=x_axis,
        y_axis=y_axis,
        u=u,
        x_ref=x_ref,
        t=t,
        bounds=bounds,
        grid_shape=grid_shape,
        params=params,
        title=title,
        streamplot=streamplot,
        figsize=figsize,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


def phase_plane_figure(
    sys,
    traj=None,
    *,
    x_axis: int = 0,
    y_axis: int | None = None,
    u=None,
    x_ref=None,
    t: float = 0.0,
    bounds=None,
    grid_shape: tuple[int, int] = (21, 21),
    params=None,
    title: str | None = None,
    streamplot: bool = False,
    figsize=None,
) -> FigureSpec:
    """
    The phase plane of ``sys`` as a description: its vector field on a grid, the trajectory over it.

    The selected state coordinates vary over the plot grid. All other state
    components are held at ``x_ref`` and the input is held at ``u``.
    """
    n = sys.n
    require_state(n)

    # The two plotted coordinates, and the grid's size
    x_axis = require_axis(x_axis, n, "x_axis")
    y_axis = default_y_axis(n) if y_axis is None else y_axis
    y_axis = require_axis(y_axis, n, "y_axis")
    n_x, n_y = require_grid_shape(grid_shape)

    # The slice: the other states held at x_ref, the input at u, the time at t
    x_ref_arr = x_ref_of(sys, x_ref)
    u_arr = u_of(sys, u)
    t_float = float(t)

    # The trajectory on the two coordinates, and the plotted window
    path = trajectory_path(traj, x_axis, y_axis) if traj is not None else None
    x_bounds, y_bounds = phase_bounds_of(
        sys,
        x_axis=x_axis,
        y_axis=y_axis,
        bounds=bounds,
        path=path,
    )

    # The grid over the two plotted coordinates
    x_values = np.linspace(x_bounds[0], x_bounds[1], n_x)
    y_values = np.linspace(y_bounds[0], y_bounds[1], n_y)
    X, Y = np.meshgrid(x_values, y_values)
    V = np.zeros_like(X, dtype=float)
    W = np.zeros_like(Y, dtype=float)

    for row in range(n_y):
        for col in range(n_x):
            # The node's state: x_ref with the two plotted coordinates set to the node
            x = x_ref_arr.copy()
            x[x_axis] = X[row, col]
            if y_axis != x_axis:
                x[y_axis] = Y[row, col]

            # ẋ = f(x, u, t), as a flat vector of the n state rates
            f_x = sys.f(x, u_arr, t_float, params=params)
            dx = np.asarray(f_x, dtype=float).reshape(-1)
            require_field_shape(dx, n)

            # The node's arrow: (ẋᵢ, ẋⱼ) on the two plotted coordinates
            V[row, col] = dx[x_axis]
            W[row, col] = dx[y_axis]

    # The field, then the trajectory, its start (a dot) and its end (a cross)
    field = Arrows(
        X, Y, V, W, style="stream" if streamplot else "quiver", color="#1f77b4"
    )
    layers = (field,) if path is None else (field, *path_layers(*path))

    # The title, and the axis labels: the state's own, else x[i], with their units
    panel = Panel(
        layers=layers,
        title=title or f"Phase plane for {sys.name}",
        x_label=state_axis_label(sys, x_axis),
        y_label=state_axis_label(sys, y_axis),
        x_lim=x_bounds,
        y_lim=y_bounds,
    )

    return FigureSpec(
        panels=(panel,), figsize=FIGSIZE_BASE if figsize is None else figsize
    )


###############################################################################
# Internal machinery
###############################################################################


def require_state(n):
    """Refuse a system without a state to plot."""
    if n < 1:
        raise ValueError("Phase-plane plots require a system with at least one state")


def require_axis(axis, n: int, name: str, what: str = "state index") -> int:
    """Refuse an index (``what`` names its kind) that is not an integer in ``[0, n - 1]``; return it as an int."""
    if isinstance(axis, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer {what}")
    try:
        axis_int = operator.index(axis)
    except TypeError as exc:
        raise ValueError(f"{name} must be an integer {what}") from exc
    if axis_int < 0 or axis_int >= n:
        raise ValueError(f"{name} must be in [0, {n - 1}], got {axis_int}")
    return axis_int


def default_y_axis(n: int) -> int:
    """The second state, or the first when there is only one."""
    return 1 if n > 1 else 0


def require_grid_shape(grid_shape) -> tuple[int, int]:
    """Refuse a grid that is not two integers of at least 2; return ``(n_x, n_y)``."""
    try:
        n_x, n_y = tuple(grid_shape)
    except TypeError as exc:
        raise ValueError("grid_shape must be a two-element tuple") from exc
    if isinstance(n_x, (bool, np.bool_)) or isinstance(n_y, (bool, np.bool_)):
        raise ValueError("grid_shape entries must be integers")
    try:
        n_x = operator.index(n_x)
        n_y = operator.index(n_y)
    except TypeError as exc:
        raise ValueError("grid_shape entries must be integers") from exc
    if n_x < 2 or n_y < 2:
        raise ValueError("grid_shape entries must be at least 2")
    return n_x, n_y


def x_ref_of(sys, x_ref) -> np.ndarray:
    """The held state: ``x_ref``, else the state's nominal value, else zero."""
    if x_ref is None:
        value = sys.state.nominal_value
        if value is None:
            value = np.zeros(sys.n)
    else:
        value = x_ref
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (sys.n,):
        raise ValueError(f"x_ref must have shape ({sys.n},), got {arr.shape}")
    return arr.copy()


def u_of(sys, u) -> np.ndarray:
    """The held input: ``u``, else the input ports' values."""
    if u is None:
        value = sys.u_nominal
    else:
        value = u
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (sys.m,):
        raise ValueError(f"u must have shape ({sys.m},), got {arr.shape}")
    return arr.copy()


def trajectory_path(traj, x_axis: int, y_axis: int):
    """The rows ``x_axis`` and ``y_axis`` of ``traj.x``, copied: ``(x, y)``."""
    x = np.asarray(traj.x, dtype=float)
    if x.ndim != 2:
        raise ValueError("traj.x must have shape (n, N)")
    max_axis = max(x_axis, y_axis)
    if x.shape[0] <= max_axis:
        raise ValueError(
            f"traj.x has {x.shape[0]} state rows, but axis {max_axis} was requested"
        )
    if x.shape[1] < 1:
        raise ValueError("traj must contain at least one sample")
    return x[x_axis, :].copy(), x[y_axis, :].copy()


def path_layers(x, y):
    """A trajectory's path, its start (a dot) and its end (a cross), as layers."""
    return (
        Trace(x, y, color="#ff7f0e", width=1.5, alpha=0.85, label="trajectory"),
        Trace(x[:1], y[:1], color="black", mode="markers", label="start"),
        Trace(x[-1:], y[-1:], color="red", mode="markers", marker="x", label="end"),
    )


def phase_bounds_of(
    sys,
    *,
    x_axis: int,
    y_axis: int,
    bounds,
    path,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """The plotted window: ``bounds``, else each axis's own (see ``bounds_from_state_or_overlay``)."""
    if bounds is not None:
        arr = np.asarray(bounds, dtype=float)
        if arr.shape != (2, 2):
            raise ValueError("bounds must have shape ((xmin, xmax), (ymin, ymax))")
        x_bounds = require_ordered_bounds(arr[0], "x bounds")
        y_bounds = require_ordered_bounds(arr[1], "y bounds")
        return x_bounds, y_bounds

    lower = sys.state.lower
    upper = sys.state.upper

    x_bounds = bounds_from_state_or_overlay(
        lower,
        upper,
        axis=x_axis,
        values=None if path is None else path[0],
    )
    y_bounds = bounds_from_state_or_overlay(
        lower,
        upper,
        axis=y_axis,
        values=None if path is None else path[1],
    )
    return x_bounds, y_bounds


def require_ordered_bounds(values, name: str) -> tuple[float, float]:
    """Refuse a pair that is not finite with ``lo < hi``; return it as floats."""
    lo = float(values[0])
    hi = float(values[1])
    if not (np.isfinite(lo) and np.isfinite(hi)):
        raise ValueError(f"{name} must be finite")
    if lo >= hi:
        raise ValueError(f"{name} lower bound must be less than upper bound")
    return lo, hi


def bounds_from_state_or_overlay(
    lower,
    upper,
    *,
    axis: int,
    values,
) -> tuple[float, float]:
    """The state's bounds on ``axis`` when finite, else the trajectory's span, else ±10."""
    if lower is not None and upper is not None:
        lo = float(np.asarray(lower, dtype=float).reshape(-1)[axis])
        hi = float(np.asarray(upper, dtype=float).reshape(-1)[axis])
        if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
            return lo, hi

    if values is not None:
        return bounds_from_values(values)

    return -10.0, 10.0


def bounds_from_values(values) -> tuple[float, float]:
    """The finite values' span, padded by 5 % (by max(1, 0.1 |lo|) when flat); ±10 when none."""
    arr = np.asarray(values, dtype=float).reshape(-1)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return -10.0, 10.0

    lo = float(np.min(finite))
    hi = float(np.max(finite))
    span = hi - lo
    if not np.isfinite(span) or span <= 0.0:
        pad = max(1.0, abs(lo) * 0.1)
    else:
        pad = 0.05 * span

    return lo - pad, hi + pad


def require_field_shape(dx, n):
    """Refuse a vector field ``f`` that does not return ``n`` state rates."""
    if dx.shape != (n,):
        raise ValueError(f"sys.f must return shape ({n},), got {dx.shape}")
