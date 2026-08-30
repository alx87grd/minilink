"""Input-output map and control-law plotting on block ports.

:func:`plot_input_output_map` sweeps components of one input port of any
static block and draws the resulting output component: a line (one swept
component), a heatmap (two), or a 3-D surface (``show_3d=True``).

:func:`plot_control_law` is the smart controller shortcut: it reads the
block's feedback-port declaration (:mod:`minilink.core.feedback`) and sweeps
the textbook coordinates for the declared profile — error space for
error-driven laws, absolute measurement or state space otherwise — pinning
the reference and every unswept coordinate at its nominal value. Blocks with
internal state are evaluated at pinned ``x0`` (the instantaneous law);
``x_axis`` / ``y_axis`` index the workspace ``z = [measurement-space; x_ctrl]``
for teaching slices such as error vs. integral state.

Both tools evaluate the block through its ordinary output-port ``compute`` —
no special block API is required.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.feedback import feedback_ports
from minilink.graphical.common import PlotResult

# Public API


@dataclass(frozen=True)
class SweepAxis:
    """One swept coordinate mapped affinely into the block evaluation inputs.

    The swept value ``v`` enters the flat input bundle (``target="u"``) or
    the block state (``target="x"``) at ``index`` as ``base + sign * v``.
    Absolute coordinates use ``sign=+1``; error coordinates use ``sign=-1``
    with the reference component as ``base`` (``y = r - e``).
    """

    target: str
    index: int
    label: str
    unit: str
    bounds: tuple[float, float]
    base: float = 0.0
    sign: float = 1.0


@dataclass(frozen=True)
class PortMapSpec:
    """Backend-neutral input-output map plot request."""

    title: str
    x_values: np.ndarray
    y_values: np.ndarray | None
    U: np.ndarray
    x_label: str
    x_unit: str
    y_label: str | None
    y_unit: str | None
    out_label: str
    out_unit: str
    vmin: float | None
    vmax: float | None


def plot_input_output_map(
    block,
    *,
    in_port=None,
    out_port=None,
    x_axis=0,
    y_axis=None,
    out_axis=0,
    bounds=None,
    grid_shape=(101, 101),
    show_3d=False,
    params=None,
    t=0.0,
    title=None,
    cmap="bwr",
    vmin=None,
    vmax=None,
    ax=None,
    show=True,
) -> PlotResult:
    """Plot one output component of ``block`` over swept input components.

    Parameters
    ----------
    block : System
        Any block with input and output ports; evaluated through the output
        port's ``compute`` at the block's nominal state and pinned inputs.
    in_port, out_port : str, optional
        Port ids; default first input port and first output port.
    x_axis, y_axis : int, optional
        Input-port components to sweep. ``y_axis=None`` draws a 1-D line.
    out_axis : int, optional
        Output component to plot (vector outputs).
    bounds : optional
        Sweep ranges: ``(lo, hi)`` for a line, ``((xlo, xhi), (ylo, yhi))``
        for a surface. Defaults to the port bounds when finite, else ±10.
    show_3d : bool, optional
        ``True`` draws a 3-D surface instead of a heatmap.
    params : dict, optional
        Alternative parameter set passed through to the port compute.
    ax : matplotlib axes, optional
        Draw into an existing subplot (side-by-side comparisons).

    Returns
    -------
    PlotResult
    """
    if not block.inputs or not block.outputs:
        raise ValueError(
            f"{block.name} needs at least one input and one output port "
            "for plot_input_output_map"
        )
    in_port = next(iter(block.inputs)) if in_port is None else in_port
    out_port = next(iter(block.outputs)) if out_port is None else out_port
    port = block.inputs[in_port]
    start = block.get_input_port_slice(in_port).start

    def input_axis(component):
        component = _normalize_axis(component, port.dim, "axis")
        return SweepAxis(
            "u",
            start + component,
            port.labels[component],
            port.units[component],
            _component_bounds(port, component),
        )

    axes = [input_axis(x_axis)]
    if y_axis is not None:
        axes.append(input_axis(y_axis))
    axes = _apply_bounds_override(axes, bounds)

    output = block.outputs[out_port]
    out_axis = _normalize_axis(out_axis, output.dim, "out_axis")
    spec = _build_spec(
        block,
        output,
        out_axis,
        axes,
        grid_shape=grid_shape,
        t=t,
        params=params,
        title=title or f"{block.name}: {output.labels[out_axis]} map",
        vmin=vmin,
        vmax=vmax,
    )
    return render_port_map_matplotlib(
        spec, show_3d=show_3d, cmap=cmap, ax=ax, show=show
    )


def plot_control_law(
    block,
    *,
    x_axis=None,
    y_axis=None,
    u_axis=0,
    r=None,
    space=None,
    bounds=None,
    grid_shape=(101, 101),
    show_3d=False,
    params=None,
    t=0.0,
    title=None,
    cmap="bwr",
    vmin=None,
    vmax=None,
    ax=None,
    show=True,
) -> PlotResult:
    """Plot the textbook slice of a controller block's control law.

    Requires a resolvable feedback declaration
    (:func:`minilink.core.feedback.feedback_ports`). The zero-argument call
    picks the sweep space from the declared profile (error space for
    error-driven laws, absolute measurement/state otherwise), sweeps the
    first relevant coordinates, and pins the reference and every other
    coordinate at its nominal value. Blocks with internal state are
    evaluated at ``x0``.

    Parameters
    ----------
    block : System
        Controller block with a feedback declaration.
    x_axis, y_axis : int, optional
        Workspace components to sweep. Components ``0 .. p-1`` are the
        measurement-space coordinates; ``p ..`` index the block's internal
        state (dynamic controllers). ``y_axis=None`` on a scalar measurement
        draws a line.
    u_axis : int, optional
        Control component to draw (MIMO commands).
    r : array, optional
        Pin the reference at this value; default the ref-port nominal.
    space : str, optional
        Override the sweep space: ``"error"``, ``"error_qdq"``,
        ``"measurement"``, or ``"state"``; default from the profile.
    bounds : optional
        Sweep ranges, as in :func:`plot_input_output_map`.
    show_3d : bool, optional
        ``True`` draws a 3-D surface instead of a heatmap.
    params : dict, optional
        Alternative parameter set (gain sweeps, trained vs. untrained).

    Returns
    -------
    PlotResult
    """
    roles = feedback_ports(block)
    if roles is None:
        raise ValueError(
            f"{block.name} has no resolvable feedback declaration — declare "
            "feedback_profile (or measurement_port / control_port) to use "
            "plot_control_law"
        )
    space = roles.plot_space if space is None else space
    if space is None:
        raise ValueError(
            f"{block.name} declares feedback ports but no static control-law "
            "semantics (its command is not a static map — e.g. an MPC solver "
            "call); set a plot_space attr or pass space= only if a static "
            "sweep is meaningful"
        )

    rbar = _resolve_reference(block, roles, r)
    meas_axes = _measurement_axes(block, roles, space, rbar)
    axes_all = meas_axes + _state_axes(block)

    x_axis, y_axis = _default_law_axes(x_axis, y_axis, space, len(meas_axes))
    x_axis = _normalize_axis(x_axis, len(axes_all), "x_axis")
    swept = [axes_all[x_axis]]
    if y_axis is not None:
        y_axis = _normalize_axis(y_axis, len(axes_all), "y_axis")
        if y_axis == x_axis:
            raise ValueError("x_axis and y_axis must differ")
        swept.append(axes_all[y_axis])
    swept = _apply_bounds_override(swept, bounds)

    output = block.outputs[roles.control]
    u_axis = _normalize_axis(u_axis, output.dim, "u_axis")
    u_base = block.get_u_from_input_ports().astype(float)
    if roles.ref is not None:
        u_base[block.get_input_port_slice(roles.ref)] = rbar

    spec = _build_spec(
        block,
        output,
        u_axis,
        swept,
        grid_shape=grid_shape,
        t=t,
        params=params,
        title=title or f"{block.name}: control law",
        vmin=vmin,
        vmax=vmax,
        u_base=u_base,
    )
    return render_port_map_matplotlib(
        spec, show_3d=show_3d, cmap=cmap, ax=ax, show=show
    )


def render_port_map_matplotlib(
    spec: PortMapSpec,
    *,
    show_3d: bool = False,
    cmap: str = "bwr",
    ax=None,
    show: bool = True,
    figsize=None,
    dpi=None,
    block: bool | None = None,
    pause: float = 0.0,
) -> PlotResult:
    """Render a port-map specification with matplotlib."""
    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    surface = spec.y_values is not None and show_3d
    owns_figure = ax is None
    if owns_figure:
        fig = plt.figure(
            figsize=FIGSIZE_BASE if figsize is None else figsize,
            dpi=DPI_FIGURE if dpi is None else dpi,
        )
        ax = fig.add_subplot(projection="3d") if surface else fig.add_subplot()
    else:
        fig = ax.figure

    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(spec.title)

    x_label = _format_axis_label(spec.x_label, spec.x_unit)
    out_label = _format_axis_label(spec.out_label, spec.out_unit)

    if spec.y_values is None:
        ax.plot(spec.x_values, spec.U)
        ax.set_ylabel(out_label, fontsize=FONT_SIZE)
        ax.grid(True, alpha=0.3)
        if spec.vmin is not None and spec.vmax is not None:
            ax.set_ylim(spec.vmin, spec.vmax)
    else:
        y_label = _format_axis_label(spec.y_label, spec.y_unit)
        if surface:
            X, Y = np.meshgrid(spec.x_values, spec.y_values)
            ax.plot_surface(X, Y, spec.U, cmap=cmap, vmin=spec.vmin, vmax=spec.vmax)
            ax.set_ylabel(y_label, fontsize=FONT_SIZE)
            ax.set_zlabel(out_label, fontsize=FONT_SIZE)
        else:
            mesh = ax.pcolormesh(
                spec.x_values,
                spec.y_values,
                spec.U,
                shading="auto",
                cmap=cmap,
                vmin=spec.vmin,
                vmax=spec.vmax,
            )
            fig.colorbar(mesh, ax=ax, label=out_label)
            ax.set_ylabel(y_label, fontsize=FONT_SIZE)

    ax.set_xlabel(x_label, fontsize=FONT_SIZE)
    ax.set_title(spec.title, fontsize=FONT_SIZE)
    if owns_figure:
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


# Internal machinery
#
# Everything below assembles input bundles and sweeps the block through its
# output-port compute: axis construction per sweep space, bounds resolution,
# and the mesh evaluation loop.

_DEFAULT_BOUNDS = (-10.0, 10.0)


def _build_spec(
    block,
    output,
    out_axis,
    axes,
    *,
    grid_shape,
    t,
    params,
    title,
    vmin,
    vmax,
    u_base=None,
) -> PortMapSpec:
    u_base = block.get_u_from_input_ports().astype(float) if u_base is None else u_base
    x_base = np.asarray(block.x0, dtype=float).copy()
    x_values, y_values, U = _evaluate_axes(
        block,
        output,
        out_axis,
        axes,
        u_base=u_base,
        x_base=x_base,
        grid_shape=grid_shape,
        t=t,
        params=params,
    )
    if vmin is None and np.isfinite(output.lower_bound[out_axis]):
        vmin = float(output.lower_bound[out_axis])
    if vmax is None and np.isfinite(output.upper_bound[out_axis]):
        vmax = float(output.upper_bound[out_axis])
    y_axis = axes[1] if len(axes) == 2 else None
    return PortMapSpec(
        title=title,
        x_values=x_values,
        y_values=y_values,
        U=U,
        x_label=axes[0].label,
        x_unit=axes[0].unit,
        y_label=None if y_axis is None else y_axis.label,
        y_unit=None if y_axis is None else y_axis.unit,
        out_label=output.labels[out_axis],
        out_unit=output.units[out_axis],
        vmin=vmin,
        vmax=vmax,
    )


def _evaluate_axes(
    block,
    output,
    out_axis,
    axes,
    *,
    u_base,
    x_base,
    grid_shape,
    t,
    params,
):
    """Sweep 1 or 2 axes over the port compute; everything else stays pinned."""
    n_x, n_y = _normalize_grid_shape(grid_shape)
    t = float(t)

    def value_at(coords):
        u = u_base.copy()
        x = x_base.copy()
        for axis, v in zip(axes, coords):
            raw = axis.base + axis.sign * v
            if axis.target == "u":
                u[axis.index] = raw
            else:
                x[axis.index] = raw
        out = np.asarray(output.compute(x, u, t, params), dtype=float).reshape(-1)
        return float(out[out_axis])

    x_values = np.linspace(axes[0].bounds[0], axes[0].bounds[1], n_x)
    if len(axes) == 1:
        U = np.array([value_at((v,)) for v in x_values])
        return x_values, None, U

    y_values = np.linspace(axes[1].bounds[0], axes[1].bounds[1], n_y)
    U = np.empty((n_y, n_x))
    for row, vy in enumerate(y_values):
        for col, vx in enumerate(x_values):
            U[row, col] = value_at((vx, vy))
    return x_values, y_values, U


def _resolve_reference(block, roles, r) -> np.ndarray:
    if roles.ref is None:
        if r is not None:
            raise ValueError(f"{block.name} has no reference port to pin with r=")
        return np.zeros(0)
    ref_port = block.inputs[roles.ref]
    if r is None:
        return ref_port.nominal_value.copy()
    rbar = np.asarray(r, dtype=float).reshape(-1)
    if rbar.shape != (ref_port.dim,):
        raise ValueError(f"r must have shape ({ref_port.dim},), got {rbar.shape}")
    return rbar


def _measurement_axes(block, roles, space, rbar) -> list[SweepAxis]:
    """Sweep-space coordinates spanning the measurement port, per semantics."""
    port = block.inputs[roles.measurement]
    start = block.get_input_port_slice(roles.measurement).start
    p = port.dim

    if space in ("measurement", "state", "absolute_qdq"):
        return [
            SweepAxis(
                "u",
                start + i,
                port.labels[i],
                port.units[i],
                _component_bounds(port, i),
            )
            for i in range(p)
        ]

    if space == "error":
        if rbar.shape != (p,):
            raise ValueError(
                f"space='error' needs a reference of dim {p} matching the "
                f"measurement; got dim {rbar.shape[0]} — use "
                "space='measurement' or pass r="
            )
        return [
            SweepAxis(
                "u",
                start + i,
                _indexed_label("e", i, p),
                port.units[i],
                _error_bounds(port, i, float(rbar[i])),
                base=float(rbar[i]),
                sign=-1.0,
            )
            for i in range(p)
        ]

    if space == "error_qdq":
        if p % 2 != 0:
            raise ValueError(
                "space='error_qdq' expects a measurement [pos; rate] of even dim"
            )
        dof = p // 2
        if rbar.size not in (dof, 2 * dof):
            raise ValueError(
                f"space='error_qdq' needs a reference of dim {dof} or {2 * dof}, "
                f"got {rbar.size}"
            )
        pos_d = rbar[:dof]
        vel_d = rbar[dof:] if rbar.size == 2 * dof else np.zeros(dof)
        pos_axes = [
            SweepAxis(
                "u",
                start + i,
                _indexed_label("e", i, dof),
                port.units[i],
                _error_bounds(port, i, float(pos_d[i])),
                base=float(pos_d[i]),
                sign=-1.0,
            )
            for i in range(dof)
        ]
        rate_axes = [
            SweepAxis(
                "u",
                start + dof + i,
                _indexed_label("de", i, dof),
                port.units[dof + i],
                _error_bounds(port, dof + i, float(vel_d[i])),
                base=float(vel_d[i]),
                sign=-1.0,
            )
            for i in range(dof)
        ]
        return pos_axes + rate_axes

    raise ValueError(
        f"Unknown sweep space {space!r}; expected 'error', 'error_qdq', "
        "'measurement', 'state', or 'absolute_qdq'"
    )


def _state_axes(block) -> list[SweepAxis]:
    """Internal-state coordinates appended to the workspace (dynamic laws)."""
    if block.n == 0:
        return []
    state = block.state
    return [
        SweepAxis(
            "x",
            i,
            state.labels[i],
            state.units[i],
            _component_bounds(state, i),
        )
        for i in range(block.n)
    ]


def _default_law_axes(x_axis, y_axis, space, p):
    """Zero-argument axis choice: first relevant measurement-space pair."""
    if x_axis is None:
        x_axis = 0
    if y_axis is None and x_axis == 0:
        if space in ("error_qdq", "absolute_qdq"):
            y_axis = p // 2  # (e0, de0) or (q0, dq0)
        elif p > 1:
            y_axis = 1
    return x_axis, y_axis


def _component_bounds(signal, i, fallback=_DEFAULT_BOUNDS) -> tuple[float, float]:
    lo = float(signal.lower_bound[i])
    hi = float(signal.upper_bound[i])
    if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
        return lo, hi
    return fallback


def _error_bounds(port, i, base, fallback=_DEFAULT_BOUNDS) -> tuple[float, float]:
    """Error range implied by the measurement bounds: e = base - y."""
    lo = float(port.lower_bound[i])
    hi = float(port.upper_bound[i])
    if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
        return base - hi, base - lo
    return fallback


def _apply_bounds_override(axes, bounds) -> list[SweepAxis]:
    if bounds is None:
        return list(axes)
    arr = np.asarray(bounds, dtype=float)
    if len(axes) == 1:
        arr = arr.reshape(-1)
        if arr.shape != (2,):
            raise ValueError("bounds must be (lo, hi) for a 1-D sweep")
        pairs = [(float(arr[0]), float(arr[1]))]
    else:
        if arr.shape != (2, 2):
            raise ValueError("bounds must be ((xlo, xhi), (ylo, yhi)) for a 2-D sweep")
        pairs = [
            (float(arr[0, 0]), float(arr[0, 1])),
            (float(arr[1, 0]), float(arr[1, 1])),
        ]
    for lo, hi in pairs:
        if not (np.isfinite(lo) and np.isfinite(hi)) or lo >= hi:
            raise ValueError("bounds must be finite with lo < hi")
    return [
        SweepAxis(
            axis.target,
            axis.index,
            axis.label,
            axis.unit,
            pair,
            base=axis.base,
            sign=axis.sign,
        )
        for axis, pair in zip(axes, pairs)
    ]


def _indexed_label(stem, i, dim) -> str:
    return stem if dim == 1 else f"{stem}[{i}]"


def _normalize_axis(axis, dim, name) -> int:
    if isinstance(axis, bool):
        raise ValueError(f"{name} must be an integer component index")
    axis_int = int(axis)
    if axis_int < 0 or axis_int >= dim:
        raise ValueError(f"{name} must be in [0, {dim - 1}], got {axis_int}")
    return axis_int


def _normalize_grid_shape(grid_shape) -> tuple[int, int]:
    try:
        n_x, n_y = tuple(grid_shape)
    except TypeError as exc:
        raise ValueError("grid_shape must be a two-element tuple") from exc
    n_x = int(n_x)
    n_y = int(n_y)
    if n_x < 2 or n_y < 2:
        raise ValueError("grid_shape entries must be at least 2")
    return n_x, n_y


def _format_axis_label(label, unit) -> str:
    if unit:
        if unit.startswith("[") and unit.endswith("]"):
            return f"{label} {unit}"
        return f"{label} [{unit}]"
    return label
