"""Phase plane: a system's vector field over two state coordinates, with an optional trajectory."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.graphical.common import PlotResult

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
    backend: str = "matplotlib",
    streamplot: bool = False,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Plot a phase-plane vector field with an optional trajectory overlay.

    Parameters
    ----------
    sys : DynamicSystem
        Continuous-time model whose ``f`` defines the vector field (leaf
        subclass or wired diagram with stacked ``f``).
    """
    backend_key = normalize_plot_backend(backend)
    require_plot_backend(backend_key, backend)

    # The field on its grid, then the figure on the chosen backend
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

    if backend_key == "plotly":
        return render_phase_plane_plotly(spec, streamplot=streamplot, show=show)

    return render_phase_plane_matplotlib(
        spec, streamplot=streamplot, show=show, **kwargs
    )


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
    n = sys.n
    require_state(n)

    # The two plotted coordinates, and the grid's size
    x_axis = normalize_axis(x_axis, n, "x_axis")
    y_axis = default_y_axis(n) if y_axis is None else y_axis
    y_axis = normalize_axis(y_axis, n, "y_axis")
    n_x, n_y = normalize_grid_shape(grid_shape)

    # The slice: the other states held at x_ref, the input at u, the time at t
    x_ref_arr = default_x_ref(sys, x_ref)
    u_arr = default_u(sys, u)
    t_float = float(t)

    # The trajectory on the two coordinates, and the plotted window
    overlay = trajectory_overlay(traj, x_axis, y_axis) if traj is not None else None
    x_bounds, y_bounds = resolve_phase_bounds(
        sys,
        x_axis=x_axis,
        y_axis=y_axis,
        bounds=bounds,
        overlay=overlay,
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

    # The title, and the axis labels: the state's own, else x[i]
    title = title or f"Phase plane for {sys.name}"
    x_label = label_for_axis(sys, x_axis)
    y_label = label_for_axis(sys, y_axis)
    x_unit = unit_for_axis(sys, x_axis)
    y_unit = unit_for_axis(sys, y_axis)

    return PhasePlaneSpec(
        title=title,
        X=X,
        Y=Y,
        V=V,
        W=W,
        x_axis=x_axis,
        y_axis=y_axis,
        x_label=x_label,
        y_label=y_label,
        x_unit=x_unit,
        y_unit=y_unit,
        x_bounds=x_bounds,
        y_bounds=y_bounds,
        trajectory=overlay,
    )


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


@dataclass(frozen=True)
class PhasePlaneTrajectoryOverlay:
    """Trajectory data projected onto two phase-plane axes."""

    x: np.ndarray
    y: np.ndarray


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

    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
        style_trajectory_subplot,
    )

    # TrueType (Type 42) fonts in PDF and PostScript exports: their text stays editable
    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots(
            figsize=FIGSIZE_BASE if figsize is None else figsize,
            dpi=DPI_FIGURE if dpi is None else dpi,
            frameon=True,
        )
    else:
        fig = ax.figure

    # The window's title, when the canvas has a window
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(spec.title)

    # The vector field: streamlines, or one arrow per grid node
    vector_kwargs = {} if vector_kwargs is None else dict(vector_kwargs)
    if streamplot:
        ax.streamplot(
            spec.X, spec.Y, spec.V, spec.W, color=vector_color, **vector_kwargs
        )
    else:
        ax.quiver(spec.X, spec.Y, spec.V, spec.W, color=vector_color, **vector_kwargs)

    # The trajectory, its start (a dot) and its end (a cross)
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

    # The title, the axis labels with their units, and the plotted window
    ax.set_title(spec.title, fontsize=FONT_SIZE)
    ax.set_xlabel(format_axis_label(spec.x_label, spec.x_unit), fontsize=FONT_SIZE)
    ax.set_ylabel(format_axis_label(spec.y_label, spec.y_unit), fontsize=FONT_SIZE)
    ax.set_xlim(spec.x_bounds)
    ax.set_ylim(spec.y_bounds)
    style_trajectory_subplot(ax)
    fig.tight_layout()

    # A window, unless the backend draws off screen
    if show and plt.get_backend().lower() != "agg":
        if block is None:
            block = is_blocking_needed()
        plt.show(block=block)
        if pause > 0.0:
            plt.pause(pause)

    return PlotResult(backend="matplotlib", payload=(fig, ax), figure=fig, axes=ax)


def render_phase_plane_plotly(
    spec: PhasePlaneSpec,
    *,
    streamplot: bool = False,
    show: bool = True,
) -> PlotResult:
    """Render a phase-plane specification with Plotly (quiver or streamlines)."""
    ff, go = require_plotly()

    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.common.plotly_style import (
        PLOTLY_2D_MARGIN,
        PLOTLY_FIG_WIDTH,
        PLOTLY_TEMPLATE,
    )

    # The vector field: streamlines, or arrows whose longest spans 0.9 of the finest
    # grid cell (its length floored at 10⁻¹²)
    x_values = spec.X[0, :]
    y_values = spec.Y[:, 0]
    if streamplot:
        fig = ff.create_streamline(
            x_values,
            y_values,
            spec.V,
            spec.W,
            density=1.2,
            line={"color": "#1f77b4", "width": 1},
        )
    else:
        cell = min(np.diff(x_values).min(), np.diff(y_values).min())
        longest = max(float(np.hypot(spec.V, spec.W).max()), 1e-12)
        fig = ff.create_quiver(
            spec.X,
            spec.Y,
            spec.V,
            spec.W,
            scale=0.9 * cell / longest,
            arrow_scale=0.3,
            line={"color": "#1f77b4", "width": 1},
        )
    fig.data[0].name = "vector field"

    # The trajectory, its start (a dot) and its end (a cross)
    if spec.trajectory is not None:
        path = spec.trajectory
        fig.add_trace(
            go.Scatter(
                x=path.x,
                y=path.y,
                mode="lines",
                name="trajectory",
                line={"color": "#ff7f0e", "width": 2},
            )
        )
        fig.add_trace(
            go.Scatter(
                x=[path.x[0]],
                y=[path.y[0]],
                mode="markers",
                name="start",
                marker={"color": "black", "size": 9},
            )
        )
        fig.add_trace(
            go.Scatter(
                x=[path.x[-1]],
                y=[path.y[-1]],
                mode="markers",
                name="end",
                marker={"color": "red", "symbol": "x", "size": 9},
            )
        )

    # The title, the axis labels with their units, and the plotted window
    fig.update_layout(
        title=spec.title,
        xaxis={
            "title_text": format_axis_label(spec.x_label, spec.x_unit),
            "range": list(spec.x_bounds),
        },
        yaxis={
            "title_text": format_axis_label(spec.y_label, spec.y_unit),
            "range": list(spec.y_bounds),
        },
        width=PLOTLY_FIG_WIDTH,
        height=480,
        showlegend=spec.trajectory is not None,
        margin=dict(PLOTLY_2D_MARGIN),
        template=PLOTLY_TEMPLATE,
    )

    if show:
        fig.show()

    return PlotResult(backend="plotly", payload=fig, figure=fig)


###############################################################################
# Internal machinery
###############################################################################


def normalize_plot_backend(backend: str) -> str:
    """The backend's key, lowercase; ``"mpl"`` is matplotlib."""
    key = str(backend).lower()
    if key in {"matplotlib", "mpl"}:
        return "matplotlib"
    return key


def require_plot_backend(key, backend):
    """Refuse a backend other than matplotlib and plotly."""
    if key not in ("matplotlib", "plotly"):
        raise ValueError(
            f"Unknown plot backend {backend!r}. Expected 'matplotlib' or 'plotly'."
        )


def require_state(n):
    """Refuse a system without a state to plot."""
    if n < 1:
        raise ValueError("Phase-plane plots require a system with at least one state")


def normalize_axis(axis, n: int, name: str) -> int:
    """The state index ``axis`` as an int in ``[0, n - 1]``; a bool is refused."""
    if isinstance(axis, bool):
        raise ValueError(f"{name} must be an integer state index")
    axis_int = int(axis)
    if axis_int < 0 or axis_int >= n:
        raise ValueError(f"{name} must be in [0, {n - 1}], got {axis_int}")
    return axis_int


def default_y_axis(n: int) -> int:
    """The second state, or the first when there is only one."""
    return 1 if n > 1 else 0


def normalize_grid_shape(grid_shape) -> tuple[int, int]:
    """The grid's ``(n_x, n_y)`` as two ints, each at least 2."""
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


def default_x_ref(sys, x_ref) -> np.ndarray:
    """The held state: ``x_ref``, else the state's nominal value, else zero."""
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


def default_u(sys, u) -> np.ndarray:
    """The held input: ``u``, else the input ports' values."""
    if u is None:
        value = sys.get_u_from_input_ports()
    else:
        value = u
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (sys.m,):
        raise ValueError(f"u must have shape ({sys.m},), got {arr.shape}")
    return arr.copy()


def trajectory_overlay(traj, x_axis: int, y_axis: int) -> PhasePlaneTrajectoryOverlay:
    """The rows ``x_axis`` and ``y_axis`` of ``traj.x``, copied."""
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


def resolve_phase_bounds(
    sys,
    *,
    x_axis: int,
    y_axis: int,
    bounds,
    overlay: PhasePlaneTrajectoryOverlay | None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """The plotted window: ``bounds``, else each axis's own (see ``bounds_from_state_or_overlay``)."""
    if bounds is not None:
        arr = np.asarray(bounds, dtype=float)
        if arr.shape != (2, 2):
            raise ValueError("bounds must have shape ((xmin, xmax), (ymin, ymax))")
        x_bounds = require_ordered_bounds(arr[0], "x bounds")
        y_bounds = require_ordered_bounds(arr[1], "y bounds")
        return x_bounds, y_bounds

    state = getattr(sys, "state", None)
    lower = getattr(state, "lower_bound", None)
    upper = getattr(state, "upper_bound", None)

    x_bounds = bounds_from_state_or_overlay(
        lower,
        upper,
        axis=x_axis,
        values=None if overlay is None else overlay.x,
    )
    y_bounds = bounds_from_state_or_overlay(
        lower,
        upper,
        axis=y_axis,
        values=None if overlay is None else overlay.y,
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
    """The state's bounds on ``axis`` when finite, else the overlay's span, else ±10."""
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


def label_for_axis(sys, axis: int) -> str:
    """The state's label of ``axis``, else ``x[axis]``."""
    labels = list(getattr(getattr(sys, "state", None), "labels", []))
    if len(labels) > axis:
        return str(labels[axis])
    return f"x[{axis}]"


def unit_for_axis(sys, axis: int) -> str:
    """The state's unit of ``axis``, else none."""
    units = list(getattr(getattr(sys, "state", None), "units", []))
    if len(units) > axis:
        return str(units[axis])
    return ""


def format_axis_label(label: str, unit: str) -> str:
    """The label with its unit in brackets: ``[m]`` stays ``[m]``."""
    if unit:
        if unit.startswith("[") and unit.endswith("]"):
            return f"{label} {unit}"
        return f"{label} [{unit}]"
    return label


def require_plotly():
    """Refuse to draw without Plotly; return ``(ff, go)``."""
    try:
        import plotly.figure_factory as ff
        import plotly.graph_objects as go
    except ImportError as exc:
        raise ImportError(
            "Plotly phase-plane plots require Plotly. Install with: "
            "pip install 'minilink[plotting]'"
        ) from exc

    return ff, go
