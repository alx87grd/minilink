"""Port maps: a block's output over swept input components, and a controller's control law."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.roles import plot_space_of, require_command, subsystem_roles
from minilink.graphical.common.labels import label_with_unit
from minilink.graphical.common.matplotlib_style import FIGSIZE_BASE
from minilink.graphical.common.plot_result import PlotResult
from minilink.graphical.figure import FigureSpec, Heatmap, Panel, Surface, Trace, draw
from minilink.graphical.phase_plane.phase_plane import (
    require_axis,
    require_grid_shape,
)

# The sweep range of a coordinate whose bounds are not finite
_DEFAULT_BOUNDS = (-10.0, 10.0)


###############################################################################
# Public API
###############################################################################


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
    is_3d=False,
    params=None,
    t=0.0,
    title=None,
    cmap="bwr",
    vmin=None,
    vmax=None,
    renderer="matplotlib",
    show=None,
    ax=None,
) -> PlotResult:
    """Plot one output component of ``block`` over swept input components.

    Sweeps one component of an input port (a line) or two (a heatmap, or a
    3-D surface with ``is_3d=True``) through the output port's ordinary
    ``compute``: no special block API is required.

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
    is_3d : bool, optional
        ``True`` draws a 3-D surface instead of a heatmap.
    params : dict, optional
        Alternative parameter set passed through to the port compute.
    renderer : str, optional
        ``"matplotlib"``, ``"plotly"``, or ``None`` for the description alone.
    show : bool, optional
        Show the figure; by default unless it is drawn into ``ax``.
    ax : matplotlib axes, optional
        Draw on existing axes: a subplot (side-by-side comparisons), or a drawn
        plot's ``result.axes``.

    Returns
    -------
    PlotResult
    """
    require_io_ports(block)

    # The ports: the first input and the first output unless named
    in_port = next(iter(block.inputs)) if in_port is None else in_port
    out_port = next(iter(block.outputs)) if out_port is None else out_port
    port = block.inputs[in_port]
    start = block.get_input_port_slice(in_port).start

    # The swept input components, each over its port bounds unless overridden
    def input_axis(component):
        component = require_axis(component, port.dim, "axis", "component index")
        return SweepAxis(
            "u",
            start + component,
            port.labels[component],
            port.units[component],
            component_bounds(port, component),
        )

    axes = [input_axis(x_axis)]
    if y_axis is not None:
        axes.append(input_axis(y_axis))
    axes = apply_bounds_override(axes, bounds)

    # The output component, swept over the axes with every other input pinned
    output = block.outputs[out_port]
    out_axis = require_axis(out_axis, output.dim, "out_axis", "component index")
    spec = port_map_figure(
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
        is_3d=is_3d,
        cmap=cmap,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


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
    is_3d=False,
    params=None,
    t=0.0,
    title=None,
    cmap="bwr",
    vmin=None,
    vmax=None,
    renderer="matplotlib",
    show=None,
    ax=None,
) -> PlotResult:
    """Plot the textbook slice of a controller block's control law.

    The block's port names say what it reads
    (:func:`minilink.core.roles.subsystem_roles`): the measurement ``y`` or
    ``x``, or the error entry of a compensator. The zero-argument call picks
    the sweep space (:func:`minilink.core.roles.plot_space_of`: error
    space for error-driven laws, absolute measurement/state otherwise),
    sweeps the first relevant coordinates, and pins the reference and every
    other coordinate at its nominal value. Blocks with internal state are
    evaluated at ``x0`` (the instantaneous law). Like
    :func:`plot_input_output_map`, it evaluates the block through its output
    port's ``compute``.

    Parameters
    ----------
    block : System
        A block that reads a measurement (``y``, ``x`` or an error entry) and
        commands ``u``.
    x_axis, y_axis : int, optional
        Workspace components to sweep, in ``z = [measurement-space; x_ctrl]``.
        Components ``0 .. p-1`` are the measurement-space coordinates;
        ``p ..`` index the block's internal state (dynamic controllers), for
        slices such as the error against the integral state.
        ``y_axis=None`` on a scalar measurement draws a line.
    u_axis : int, optional
        Control component to draw (MIMO commands).
    r : array, optional
        Pin the reference at this value; default the ref-port nominal.
    space : str, optional
        Override the sweep space: ``"error"``, ``"error_qdq"``,
        ``"measurement"``, or ``"state"``; default from the ports or the
        block's ``plot_space``.
    bounds : optional
        Sweep ranges, as in :func:`plot_input_output_map`.
    is_3d : bool, optional
        ``True`` draws a 3-D surface instead of a heatmap.
    params : dict, optional
        Alternative parameter set (gain sweeps, trained vs. untrained).
    renderer, show, ax : optional
        As in :func:`plot_input_output_map`.

    Returns
    -------
    PlotResult
    """
    roles = law_roles_of(block)
    require_law_roles(block, roles)

    space = plot_space_of(block) if space is None else space
    require_plot_space(block, space)

    # The workspace: the measurement-space coordinates, then the internal state
    rbar = reference_of(block, roles, r)
    meas_axes = measurement_axes(block, roles, space, rbar)
    axes_all = meas_axes + state_axes(block)

    # The swept coordinates: the textbook pair unless named
    x_axis, y_axis = default_law_axes(x_axis, y_axis, space, len(meas_axes))
    x_axis = require_axis(x_axis, len(axes_all), "x_axis", "component index")
    swept = [axes_all[x_axis]]
    if y_axis is not None:
        y_axis = require_axis(y_axis, len(axes_all), "y_axis", "component index")
        require_distinct_axes(x_axis, y_axis)
        swept.append(axes_all[y_axis])
    swept = apply_bounds_override(swept, bounds)

    # The command component, with every input at its nominal value and r pinned
    output = block.outputs[roles.control]
    u_axis = require_axis(u_axis, output.dim, "u_axis", "component index")
    u_base = block.u_nominal.astype(float)
    if roles.ref is not None:
        u_base[block.get_input_port_slice(roles.ref)] = rbar

    # The law, swept over the chosen coordinates
    spec = port_map_figure(
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
        is_3d=is_3d,
        cmap=cmap,
        u_base=u_base,
    )

    return draw(spec, renderer=renderer, show=show, ax=ax)


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


###############################################################################
# Internal machinery
###############################################################################


# The input bundles and the sweep through the output port's compute: the axes of
# each sweep space, their bounds, and the grid evaluation


def require_io_ports(block):
    """Refuse a block without an input port or an output port."""
    if not block.inputs or not block.outputs:
        raise ValueError(
            f"{block.name} needs at least one input and one output port "
            "for plot_input_output_map"
        )


def component_bounds(signal, i, fallback=_DEFAULT_BOUNDS) -> tuple[float, float]:
    """The bounds of component ``i`` when finite and ordered, else ``fallback``."""
    lo = float(signal.lower[i])
    hi = float(signal.upper[i])
    if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
        return lo, hi
    return fallback


def apply_bounds_override(axes, bounds) -> list[SweepAxis]:
    """The axes over the given ``bounds``, each pair finite with ``lo < hi``."""
    if bounds is None:
        return list(axes)

    # One (lo, hi) pair per axis
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


def port_map_figure(
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
    is_3d,
    cmap,
    u_base=None,
) -> FigureSpec:
    """``output[out_axis]`` swept over ``axes``: a line, a heatmap or a surface, as a description."""
    # The pinned point: the nominal inputs (or u_base) and the block's x0
    u_base = block.u_nominal.astype(float) if u_base is None else u_base
    x_base = np.asarray(block.x0, dtype=float).copy()

    x_values, y_values, U = evaluate_axes(
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

    # The color limits: the output's finite bounds, unless given
    if vmin is None and np.isfinite(output.lower[out_axis]):
        vmin = float(output.lower[out_axis])
    if vmax is None and np.isfinite(output.upper[out_axis]):
        vmax = float(output.upper[out_axis])

    x_label = label_with_unit(axes[0].label, axes[0].unit)
    out_label = label_with_unit(output.labels[out_axis], output.units[out_axis])

    # One swept axis: a line, within the output's limits when both are known
    if y_values is None:
        has_limits = vmin is not None and vmax is not None
        panel = Panel(
            layers=(Trace(x_values, U),),
            title=title,
            x_label=x_label,
            y_label=out_label,
            y_lim=(vmin, vmax) if has_limits else None,
            legend=True,
        )
        return FigureSpec(panels=(panel,), figsize=FIGSIZE_BASE)

    # Two swept axes: a surface, or a heatmap with its colour bar
    y_label = label_with_unit(axes[1].label, axes[1].unit)
    if is_3d:
        layer = Surface(
            x=x_values, y=y_values, z=U, cmap=cmap, vmin=vmin, vmax=vmax, colorbar=None
        )
    else:
        layer = Heatmap(
            x=x_values,
            y=y_values,
            z=U,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            shading="cells",
            colorbar=out_label,
        )
    panel = Panel(
        layers=(layer,),
        title=title,
        x_label=x_label,
        y_label=y_label,
        z_label=out_label if is_3d else "",
        is_3d=is_3d,
        legend=True,
    )

    return FigureSpec(panels=(panel,), figsize=FIGSIZE_BASE)


def evaluate_axes(
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
    n_x, n_y = require_grid_shape(grid_shape)
    t = float(t)

    # The output at one point: each swept value v enters as base + sign·v
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

    # One axis: a line, one value per x
    x_values = np.linspace(axes[0].bounds[0], axes[0].bounds[1], n_x)
    if len(axes) == 1:
        U = np.array([value_at((v,)) for v in x_values])
        return x_values, None, U

    # Two axes: a grid, row i at y_i and column j at x_j
    y_values = np.linspace(axes[1].bounds[0], axes[1].bounds[1], n_y)
    U = np.empty((n_y, n_x))
    for row, vy in enumerate(y_values):
        for col, vx in enumerate(x_values):
            U[row, col] = value_at((vx, vy))

    return x_values, y_values, U


@dataclass(frozen=True)
class LawRoles:
    """The ports a control-law sweep reads: the swept input, the pinned reference, the command."""

    measurement: str
    ref: str | None
    control: str


def law_roles_of(block):
    """The swept port (a measurement, else a compensator's entry), the reference and the command."""
    roles = subsystem_roles(block)
    measurement = roles.measurement or roles.entry
    outputs = list(getattr(block, "outputs", {}))
    if measurement is None or not outputs:
        return None
    control = require_command(block) or outputs[0]
    return LawRoles(measurement, roles.reference, control)


def require_law_roles(block, roles):
    """Refuse a block that reads no measurement or commands nothing: it has no law to draw."""
    if roles is None:
        raise ValueError(
            f"{block.name} reads no measurement (an input named 'y' or 'x', or "
            "the error entry of a compensator) and commands no 'u', so it has no "
            "control law to draw; plot_input_output_map() sweeps any input"
        )


def require_plot_space(block, space):
    """Refuse a block with no sweep space: its command is not a static map."""
    if space is None:
        raise ValueError(
            f"{block.name} reads a measurement but has no static control-law "
            "semantics (its command is not a static map — e.g. an MPC solver "
            "call); set a plot_space attr or pass space= only if a static "
            "sweep is meaningful"
        )


def reference_of(block, roles, r) -> np.ndarray:
    """The pinned reference r̄: ``r`` checked against the reference port, else its nominal."""
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


def measurement_axes(block, roles, space, rbar) -> list[SweepAxis]:
    """Sweep-space coordinates spanning the measurement port, per semantics."""
    port = block.inputs[roles.measurement]
    start = block.get_input_port_slice(roles.measurement).start
    p = port.dim

    # Absolute coordinates: each measured component as it is
    if space in ("measurement", "state", "absolute_qdq"):
        return [
            SweepAxis(
                "u",
                start + i,
                port.labels[i],
                port.units[i],
                component_bounds(port, i),
            )
            for i in range(p)
        ]

    # Error coordinates: y = r̄ − e, one per measured component
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
                indexed_label("e", i, p),
                port.units[i],
                error_bounds(port, i, float(rbar[i])),
                base=float(rbar[i]),
                sign=-1.0,
            )
            for i in range(p)
        ]

    # Position and rate errors of a [q; dq] measurement: q = q_d − e, dq = dq_d − de
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

        # The desired positions, and the desired rates (zero when r̄ holds positions only)
        pos_d = rbar[:dof]
        vel_d = rbar[dof:] if rbar.size == 2 * dof else np.zeros(dof)

        pos_axes = [
            SweepAxis(
                "u",
                start + i,
                indexed_label("e", i, dof),
                port.units[i],
                error_bounds(port, i, float(pos_d[i])),
                base=float(pos_d[i]),
                sign=-1.0,
            )
            for i in range(dof)
        ]
        rate_axes = [
            SweepAxis(
                "u",
                start + dof + i,
                indexed_label("de", i, dof),
                port.units[dof + i],
                error_bounds(port, dof + i, float(vel_d[i])),
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


def indexed_label(stem, i, dim) -> str:
    """``stem`` for a scalar, ``stem[i]`` for a component of a vector."""
    return stem if dim == 1 else f"{stem}[{i}]"


def error_bounds(port, i, base, fallback=_DEFAULT_BOUNDS) -> tuple[float, float]:
    """Error range implied by the measurement bounds: e = base - y."""
    lo = float(port.lower[i])
    hi = float(port.upper[i])
    if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
        return base - hi, base - lo
    return fallback


def state_axes(block) -> list[SweepAxis]:
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
            component_bounds(state, i),
        )
        for i in range(block.n)
    ]


def default_law_axes(x_axis, y_axis, space, p):
    """Zero-argument axis choice: first relevant measurement-space pair."""
    if x_axis is None:
        x_axis = 0
    if y_axis is None and x_axis == 0:
        if space in ("error_qdq", "absolute_qdq"):
            y_axis = p // 2  # (e0, de0) or (q0, dq0)
        elif p > 1:
            y_axis = 1
    return x_axis, y_axis


def require_distinct_axes(x_axis, y_axis):
    """Refuse a sweep of one coordinate against itself."""
    if y_axis == x_axis:
        raise ValueError("x_axis and y_axis must differ")
