"""Phase-plane vector-field plotting."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.graphical.common import PlotResult


@dataclass(frozen=True)
class PhasePlaneTrajectoryOverlay:
    """Trajectory data projected onto two phase-plane axes."""

    x: np.ndarray
    y: np.ndarray


@dataclass(frozen=True)
class PhasePlaneSpec:
    """Backend-neutral phase-plane plot request."""

    title: str
    X: np.ndarray
    Y: np.ndarray
    V: np.ndarray
    W: np.ndarray
    x_axis: int
    y_axis: int
    x_label: str
    y_label: str
    x_unit: str
    y_unit: str
    x_bounds: tuple[float, float]
    y_bounds: tuple[float, float]
    trajectory: PhasePlaneTrajectoryOverlay | None = None


def build_phase_plane_spec(
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
) -> PhasePlaneSpec:
    """
    Build a phase-plane vector-field specification.

    The selected state coordinates vary over the plot grid. All other state
    components are held at ``x_ref`` and the input is held at ``u``.
    """
    if sys.n < 1:
        raise ValueError("Phase-plane plots require a system with at least one state")

    x_axis = _normalize_axis(x_axis, sys.n, "x_axis")
    y_axis = _default_y_axis(sys.n) if y_axis is None else y_axis
    y_axis = _normalize_axis(y_axis, sys.n, "y_axis")
    n_x, n_y = _normalize_grid_shape(grid_shape)

    x_ref_arr = _default_x_ref(sys, x_ref)
    u_arr = _default_u(sys, u)
    t_float = float(t)

    overlay = _trajectory_overlay(traj, x_axis, y_axis) if traj is not None else None
    x_bounds, y_bounds = _resolve_phase_bounds(
        sys,
        x_axis=x_axis,
        y_axis=y_axis,
        bounds=bounds,
        overlay=overlay,
    )

    x_values = np.linspace(x_bounds[0], x_bounds[1], n_x)
    y_values = np.linspace(y_bounds[0], y_bounds[1], n_y)
    X, Y = np.meshgrid(x_values, y_values)
    V = np.zeros_like(X, dtype=float)
    W = np.zeros_like(Y, dtype=float)

    for row in range(n_y):
        for col in range(n_x):
            x = x_ref_arr.copy()
            x[x_axis] = X[row, col]
            if y_axis != x_axis:
                x[y_axis] = Y[row, col]
            dx = np.asarray(
                sys.f(x, u_arr, t_float, params=params),
                dtype=float,
            ).reshape(-1)
            if dx.shape != (sys.n,):
                raise ValueError(
                    f"sys.f must return shape ({sys.n},), got {dx.shape}"
                )
            V[row, col] = dx[x_axis]
            W[row, col] = dx[y_axis]

    return PhasePlaneSpec(
        title=title or f"Phase plane for {sys.name}",
        X=X,
        Y=Y,
        V=V,
        W=W,
        x_axis=x_axis,
        y_axis=y_axis,
        x_label=_label_for_axis(sys, x_axis),
        y_label=_label_for_axis(sys, y_axis),
        x_unit=_unit_for_axis(sys, x_axis),
        y_unit=_unit_for_axis(sys, y_axis),
        x_bounds=x_bounds,
        y_bounds=y_bounds,
        trajectory=overlay,
    )


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
    backend: str = "matplotlib",
    streamplot: bool = False,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Plot a phase-plane vector field with an optional trajectory overlay."""
    backend_key = _normalize_backend(backend)
    if backend_key != "matplotlib":
        raise ValueError(
            f"backend={backend!r} is not implemented for phase-plane plots yet; "
            "use backend='matplotlib'."
        )

    spec = build_phase_plane_spec(
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
    )
    return render_phase_plane_matplotlib(
        spec,
        streamplot=streamplot,
        show=show,
        **kwargs,
    )


def render_phase_plane_matplotlib(
    spec: PhasePlaneSpec,
    *,
    streamplot: bool = False,
    show: bool = True,
    ax=None,
    figsize=None,
    dpi=None,
    vector_color: str = "C0",
    trajectory_color: str = "C1",
    vector_kwargs=None,
    trajectory_kwargs=None,
    start_kwargs=None,
    end_kwargs=None,
    block: bool | None = None,
    pause: float = 0.0,
) -> PlotResult:
    """Render a phase-plane specification with matplotlib."""
    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
        style_trajectory_subplot,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    if ax is None:
        fig, ax = plt.subplots(
            figsize=FIGSIZE_BASE if figsize is None else figsize,
            dpi=DPI_FIGURE if dpi is None else dpi,
            frameon=True,
        )
    else:
        fig = ax.figure

    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(spec.title)

    vector_kwargs = {} if vector_kwargs is None else dict(vector_kwargs)
    if streamplot:
        ax.streamplot(
            spec.X,
            spec.Y,
            spec.V,
            spec.W,
            color=vector_color,
            **vector_kwargs,
        )
    else:
        ax.quiver(
            spec.X,
            spec.Y,
            spec.V,
            spec.W,
            color=vector_color,
            **vector_kwargs,
        )

    if spec.trajectory is not None:
        trajectory_kwargs = (
            {"linewidth": 1.5, "alpha": 0.85}
            if trajectory_kwargs is None
            else dict(trajectory_kwargs)
        )
        start_kwargs = (
            {"marker": "o", "color": "black", "linestyle": "None"}
            if start_kwargs is None
            else dict(start_kwargs)
        )
        end_kwargs = (
            {"marker": "x", "color": "red", "linestyle": "None"}
            if end_kwargs is None
            else dict(end_kwargs)
        )
        (path_line,) = ax.plot(
            spec.trajectory.x,
            spec.trajectory.y,
            color=trajectory_color,
            label="trajectory",
            **trajectory_kwargs,
        )
        (start_line,) = ax.plot(
            [spec.trajectory.x[0]],
            [spec.trajectory.y[0]],
            label="start",
            **start_kwargs,
        )
        (end_line,) = ax.plot(
            [spec.trajectory.x[-1]],
            [spec.trajectory.y[-1]],
            label="end",
            **end_kwargs,
        )
        ax.legend(handles=[path_line, start_line, end_line], loc="best")

    ax.set_title(spec.title, fontsize=FONT_SIZE)
    ax.set_xlabel(_format_axis_label(spec.x_label, spec.x_unit), fontsize=FONT_SIZE)
    ax.set_ylabel(_format_axis_label(spec.y_label, spec.y_unit), fontsize=FONT_SIZE)
    ax.set_xlim(spec.x_bounds)
    ax.set_ylim(spec.y_bounds)
    style_trajectory_subplot(ax)
    fig.tight_layout()

    if show and plt.get_backend().lower() != "agg":
        if block is None:
            block = is_blocking_needed()
        plt.show(block=block)
        if pause > 0.0:
            plt.pause(pause)

    return PlotResult(
        backend="matplotlib",
        payload=(fig, ax),
        figure=fig,
        axes=ax,
    )


def _normalize_backend(backend: str) -> str:
    key = str(backend).lower()
    if key in {"matplotlib", "mpl"}:
        return "matplotlib"
    return key


def _default_y_axis(n: int) -> int:
    return 1 if n > 1 else 0


def _normalize_axis(axis, n: int, name: str) -> int:
    if isinstance(axis, bool):
        raise ValueError(f"{name} must be an integer state index")
    axis_int = int(axis)
    if axis_int < 0 or axis_int >= n:
        raise ValueError(f"{name} must be in [0, {n - 1}], got {axis_int}")
    return axis_int


def _normalize_grid_shape(grid_shape) -> tuple[int, int]:
    try:
        n_x, n_y = tuple(grid_shape)
    except TypeError as exc:
        raise ValueError("grid_shape must be a two-element tuple") from exc
    if isinstance(n_x, bool) or isinstance(n_y, bool):
        raise ValueError("grid_shape entries must be integers")
    n_x = int(n_x)
    n_y = int(n_y)
    if n_x < 2 or n_y < 2:
        raise ValueError("grid_shape entries must be at least 2")
    return n_x, n_y


def _default_x_ref(sys, x_ref) -> np.ndarray:
    if x_ref is None:
        value = getattr(getattr(sys, "state", None), "nominal_value", None)
        if value is None:
            value = np.zeros(sys.n)
    else:
        value = x_ref
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (sys.n,):
        raise ValueError(f"x_ref must have shape ({sys.n},), got {arr.shape}")
    return arr.copy()


def _default_u(sys, u) -> np.ndarray:
    if u is None:
        value = sys.get_u_from_input_ports()
    else:
        value = u
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (sys.m,):
        raise ValueError(f"u must have shape ({sys.m},), got {arr.shape}")
    return arr.copy()


def _trajectory_overlay(traj, x_axis: int, y_axis: int) -> PhasePlaneTrajectoryOverlay:
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
    return PhasePlaneTrajectoryOverlay(
        x=x[x_axis, :].copy(),
        y=x[y_axis, :].copy(),
    )


def _resolve_phase_bounds(
    sys,
    *,
    x_axis: int,
    y_axis: int,
    bounds,
    overlay: PhasePlaneTrajectoryOverlay | None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    if bounds is not None:
        arr = np.asarray(bounds, dtype=float)
        if arr.shape != (2, 2):
            raise ValueError("bounds must have shape ((xmin, xmax), (ymin, ymax))")
        return _validate_bounds(arr[0], "x bounds"), _validate_bounds(
            arr[1],
            "y bounds",
        )

    state = getattr(sys, "state", None)
    lower = getattr(state, "lower_bound", None)
    upper = getattr(state, "upper_bound", None)

    x_bounds = _bounds_from_state_or_overlay(
        lower,
        upper,
        axis=x_axis,
        values=None if overlay is None else overlay.x,
    )
    y_bounds = _bounds_from_state_or_overlay(
        lower,
        upper,
        axis=y_axis,
        values=None if overlay is None else overlay.y,
    )
    return x_bounds, y_bounds


def _bounds_from_state_or_overlay(
    lower,
    upper,
    *,
    axis: int,
    values,
) -> tuple[float, float]:
    if lower is not None and upper is not None:
        lo = float(np.asarray(lower, dtype=float).reshape(-1)[axis])
        hi = float(np.asarray(upper, dtype=float).reshape(-1)[axis])
        if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
            return lo, hi

    if values is not None:
        return _bounds_from_values(values)

    return -10.0, 10.0


def _bounds_from_values(values) -> tuple[float, float]:
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


def _validate_bounds(values, name: str) -> tuple[float, float]:
    lo, hi = (float(values[0]), float(values[1]))
    if not (np.isfinite(lo) and np.isfinite(hi)):
        raise ValueError(f"{name} must be finite")
    if lo >= hi:
        raise ValueError(f"{name} lower bound must be less than upper bound")
    return lo, hi


def _label_for_axis(sys, axis: int) -> str:
    labels = list(getattr(getattr(sys, "state", None), "labels", []))
    if len(labels) > axis:
        return str(labels[axis])
    return f"x[{axis}]"


def _unit_for_axis(sys, axis: int) -> str:
    units = list(getattr(getattr(sys, "state", None), "units", []))
    if len(units) > axis:
        return str(units[axis])
    return ""


def _format_axis_label(label: str, unit: str) -> str:
    if unit:
        if unit.startswith("[") and unit.endswith("]"):
            return f"{label} {unit}"
        return f"{label} [{unit}]"
    return label
