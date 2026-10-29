"""Time-signal plots: a trajectory's signals, one row per component, on a chosen backend."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.graphical.common import PlotResult
from minilink.graphical.signals.signal_colors import (
    color_for_signal,
    is_internal_signal,
)

TIME_ABSCISSA_LABEL = "Time [s]"
STEP_ABSCISSA_LABEL = "Step [k]"

# The channels CostFunction.evaluate_trajectory adds, labeled as in the textbook
_COST_SIGNAL_LABELS = {"cost_rate": "dJ/dt", "cost": "J"}


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class SignalTrace:
    """One scalar time trace selected from a vector signal."""

    signal: str
    component: int
    label: str
    unit: str
    values: np.ndarray
    color: str
    alpha: float = 1.0
    linewidth: float = 1.5


@dataclass(frozen=True)
class SignalPlotSpec:
    """Backend-neutral signal plot request."""

    title: str
    t: np.ndarray
    traces: tuple[SignalTrace, ...]
    abscissa_label: str = TIME_ABSCISSA_LABEL


class LivePlotHandle:
    """Base class for live signal plot handles."""

    supports_live = True

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        """Update an existing plot from a new trajectory or plot spec."""
        raise NotImplementedError

    def close(self) -> None:
        """Release backend resources held by the live plot."""
        raise NotImplementedError


def resolve_plot_signals(sys) -> tuple[str, ...]:
    """Return default time-signal names for ``sys`` (leaf or diagram).

    Leaf systems plot the state ``x``, and the input ``u`` when they have one.
    Diagrams plot flattened states plus controller control outputs
    (``{id}:u`` for static or dynamic controllers) and source outputs
    (``{id}:y``) without requiring explicit ``sys_id:port`` names from the
    user.
    """
    if not isinstance(sys, DiagramSystem):
        return ("x", "u") if sys.m else ("x",)

    # The flattened state
    signals = []
    if sys.n:
        signals.append("x")

    # Each controller's command, and each source's output
    for sys_id, sub in sys.subsystems.items():
        if "u" in sub.outputs:
            signals.append(f"{sys_id}:u")
        elif sub.n == 0 and not sub.inputs:
            if "y" in sub.outputs:
                signals.append(f"{sys_id}:y")
            elif sub.outputs:
                port_id = next(iter(sub.outputs))
                signals.append(f"{sys_id}:{port_id}")

    return tuple(signals)


def build_signal_plot_spec(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    title: str | None = None,
    abscissa_label: str = TIME_ABSCISSA_LABEL,
) -> SignalPlotSpec:
    """Build a one-component-per-row plot specification."""
    requested = normalize_plot_signals(sys, signals)

    # A block nested in a sub-diagram ("loop/plant:x"), sampled level by level
    nested = {}
    for name in requested:
        if "/" in name.split(":", 1)[0] and not traj.has_signal(name):
            values, labels, units = nested_signal(sys, traj, name)
            traj = traj.with_signal(name, values)
            nested[name] = (labels, units)

    # A block of this diagram: the signals the run did not record, and only those
    if isinstance(sys, DiagramSystem):
        missing = []
        for name in requested:
            if ":" not in name or traj.has_signal(name):
                continue

            sys_id, port_id = name.split(":", 1)
            subsystem = sys.subsystems.get(sys_id)
            if subsystem is not None and port_id in subsystem.outputs:
                missing.append(name)
        if missing:
            traj = sys.reconstruct_internal_signals(traj, ports=missing)

    # A named output port, input port, or state label of this system
    for name in requested:
        if name in ("x", "u") or traj.has_signal(name):
            continue
        sampled = system_channel_samples(sys, traj, name)
        if sampled is not None:
            traj = traj.with_signal(name, sampled)

    traces = []
    internal_index = 0
    for signal_name in requested:
        require_known_signal(sys, traj, signal_name, nested)

        # The signal's samples, one row per component, with their labels and units
        if signal_name == "x":
            values = np.asarray(traj.x, dtype=float)
            labels = labels_for_vector(sys.state.labels, values.shape[0], "x")
            units = units_for_vector(sys.state.units, values.shape[0])
        elif signal_name == "u":
            values = np.asarray(traj.u, dtype=float)
            input_labels, input_units = sys.get_all_input_labels_and_units()
            labels = labels_for_vector(input_labels, values.shape[0], "u")
            units = units_for_vector(input_units, values.shape[0])
        elif signal_name in nested:
            values = np.asarray(traj.get_signal(signal_name), dtype=float)
            labels, units = nested[signal_name]
        else:
            values = np.asarray(traj.get_signal(signal_name), dtype=float)
            labels, units = labels_and_units_for_extra_signal(
                sys,
                signal_name,
                values.shape[0],
            )

        # An internal signal takes the next color of the internal palette
        idx = internal_index if is_internal_signal(signal_name) else None
        if is_internal_signal(signal_name):
            internal_index += 1

        # One trace per component, styled by its signal
        n_components = values.shape[0]
        for i in range(n_components):
            style = color_for_signal(
                signal_name,
                component=i,
                n_components=n_components,
                internal_index=idx,
            )
            traces.append(
                SignalTrace(
                    signal=signal_name,
                    component=i,
                    label=labels[i],
                    unit=units[i],
                    values=values[i, :],
                    color=style.color,
                    alpha=style.alpha,
                    linewidth=style.linewidth,
                )
            )

    require_traces(traces)

    return SignalPlotSpec(
        title=title or f"Time signals for {sys.name}",
        t=np.asarray(traj.t, dtype=float),
        traces=tuple(traces),
        abscissa_label=abscissa_label,
    )


def plot_time_signals(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    abscissa_label: str = TIME_ABSCISSA_LABEL,
    backend="matplotlib",
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Plot sampled time signals with the selected backend."""
    spec = build_signal_plot_spec(
        sys,
        traj,
        signals=signals,
        abscissa_label=abscissa_label,
    )

    key = require_signal_backend(backend)

    if key == "matplotlib":
        # A real cycle: matplotlib_backend imports from this module
        from minilink.graphical.signals.matplotlib_backend import (
            render_matplotlib_signal_plot,
        )

        return render_matplotlib_signal_plot(spec, show=show, **kwargs)

    # A real cycle: plotly_backend imports from this module
    from minilink.graphical.signals.plotly_backend import (
        render_plotly_signal_plot,
    )

    return render_plotly_signal_plot(spec, show=show, **kwargs)


def open_time_signal_plot(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    abscissa_label: str = TIME_ABSCISSA_LABEL,
    backend="matplotlib",
    show: bool = True,
    **kwargs,
) -> LivePlotHandle:
    """Open a live time-signal plot with an update handle."""
    signal_names = normalize_plot_signals(sys, signals)

    # The spec of any later trajectory, on the same signals
    def spec_builder(next_traj, *, title=None):
        return build_signal_plot_spec(
            sys,
            next_traj,
            signals=signal_names,
            title=title,
            abscissa_label=abscissa_label,
        )

    spec = spec_builder(traj)

    key = require_signal_backend(backend)

    if key == "matplotlib":
        # A real cycle: matplotlib_backend imports from this module
        from minilink.graphical.signals.matplotlib_backend import (
            open_matplotlib_signal_plot,
        )

        return open_matplotlib_signal_plot(
            spec,
            spec_builder=spec_builder,
            show=show,
            **kwargs,
        )

    # A real cycle: plotly_backend imports from this module
    from minilink.graphical.signals.plotly_backend import (
        open_plotly_signal_plot,
    )

    return open_plotly_signal_plot(
        spec,
        spec_builder=spec_builder,
        show=show,
        **kwargs,
    )


###############################################################################
# Internal machinery
###############################################################################


def normalize_plot_signals(sys, signals) -> tuple[str, ...]:
    """Expand plot signal specs to ``'sys_id:port'`` / boundary names."""
    # One name, one (subsystem, port) pair, or a sequence of them
    if isinstance(signals, str):
        items = (signals,)
    elif is_subsystem_port_pair(signals):
        items = (signals,)
    else:
        items = tuple(signals)

    if not isinstance(sys, DiagramSystem):
        return items

    # Each pair as the name of its signal: a nested block by its path
    resolved = []
    for item in items:
        if is_subsystem_port_pair(item):
            subsystem, port_id = item
            path = subsystem_path(sys, subsystem)
            if path is None or "/" not in path or port_id not in subsystem.outputs:
                # A block of this diagram, or the reason it cannot be plotted
                resolved.append(sys.subsystem_signal(subsystem, port_id))
            else:
                resolved.append(f"{path}:{port_id}")
            continue

        if not isinstance(item, str):
            raise TypeError(
                "Diagram plot signals must be strings or (subsystem, port) "
                f"tuples; got {type(item).__name__}"
            )

        resolved.append(item)

    return tuple(resolved)


def is_subsystem_port_pair(item) -> bool:
    """Whether ``item`` is a ``(subsystem, port_id)`` pair."""
    return (
        isinstance(item, tuple)
        and len(item) == 2
        and isinstance(item[0], System)
        and isinstance(item[1], str)
    )


def subsystem_path(diagram, subsystem):
    """The ``"a/b/c"`` path to ``subsystem`` through ``diagram``'s nested diagrams, or ``None``.

    A block of ``diagram`` itself is found before a block nested deeper.
    """
    for sys_id, block in diagram.subsystems.items():
        if block is subsystem:
            return sys_id

    for sys_id, block in diagram.subsystems.items():
        if isinstance(block, DiagramSystem):
            inner = subsystem_path(block, subsystem)
            if inner is not None:
                return f"{sys_id}/{inner}"

    return None


def nested_signal(sys, traj, name):
    """Sample the output ``"a/b/c:port"`` of a nested block along ``traj``: ``(values, labels, units)``.

    The trajectory is narrowed one diagram level at a time (``trajectory_of``)
    to the diagram holding the block, whose internal signals give the port.
    """
    path, port_id = name.rsplit(":", 1)
    *outer, block_id = path.split("/")

    # Down the path, one diagram level at a time
    diagram = sys
    diagram_traj = traj
    for sys_id in outer:
        inner = diagram.subsystems[sys_id]
        diagram_traj = diagram.trajectory_of(inner, diagram_traj)
        diagram = inner

    # The port among the internal signals of the diagram holding the block
    diagram_traj = diagram.reconstruct_internal_signals(diagram_traj)
    key = f"{block_id}:{port_id}"
    values = np.asarray(diagram_traj.get_signal(key), dtype=float)
    labels, units = labels_and_units_for_extra_signal(diagram, key, values.shape[0])

    return values, labels, units


def require_known_signal(sys, traj, signal_name, nested):
    """Refuse a signal that is neither ``x``, ``u``, a nested block's nor the trajectory's."""
    if signal_name in ("x", "u") or signal_name in nested:
        return

    if not traj.has_signal(signal_name):
        available = ", ".join(available_signal_names(sys, traj))
        raise ValueError(
            f"Unknown signal(s): {signal_name}. Available signals: {available}"
        )


def labels_for_vector(labels, dim: int, name: str) -> tuple[str, ...]:
    """The first ``dim`` labels, or ``name[i]`` for each component when too few are given."""
    labels = list(labels)
    if len(labels) >= dim:
        return tuple(str(label) for label in labels[:dim])

    return tuple(f"{name}[{i}]" for i in range(dim))


def units_for_vector(units, dim: int) -> tuple[str, ...]:
    """The first ``dim`` units, or no unit for each component when too few are given."""
    units = list(units)
    if len(units) >= dim:
        return tuple(str(unit) for unit in units[:dim])

    return tuple("" for _ in range(dim))


def labels_and_units_for_extra_signal(sys, name: str, dim: int):
    """The labels and units of a trajectory signal other than ``x`` and ``u``."""
    # A cost channel, labeled as in the textbook
    if name in _COST_SIGNAL_LABELS and dim == 1:
        return (_COST_SIGNAL_LABELS[name],), ("",)

    # A block's output port in a diagram
    if ":" in name and isinstance(sys, DiagramSystem):
        sys_id, port_id = name.split(":", 1)
        subsystem = sys.subsystems.get(sys_id)
        if subsystem is not None and port_id in subsystem.outputs:
            port = subsystem.outputs[port_id]

            # The labels the port declares, else the state's on the state port,
            # else the signal's own name
            declared = list(port.labels) != [f"{port_id}[{i}]" for i in range(dim)]
            if declared and len(port.labels) >= dim:
                labels = tuple(str(label) for label in port.labels[:dim])
            elif port_id == "x" and len(subsystem.state.labels) == dim:
                labels = tuple(str(label) for label in subsystem.state.labels)
                return labels, units_for_vector(subsystem.state.units, dim)
            else:
                labels = tuple(
                    f"{name}" if dim == 1 else f"{name}[{i}]" for i in range(dim)
                )

            return labels, units_for_vector(port.units, dim)

    # A named output or input port of this system
    if name in sys.outputs:
        port = sys.outputs[name]
        labels = labels_for_vector(port.labels, dim, name)
        return labels, units_for_vector(port.units, dim)
    if name in sys.inputs:
        port = sys.inputs[name]
        labels = labels_for_vector(port.labels, dim, name)
        return labels, units_for_vector(port.units, dim)

    # A single state component selected by its state label
    state = sys.state
    if dim == 1 and name in state.labels:
        idx = list(state.labels).index(name)
        unit = state.units[idx] if idx < len(state.units) else ""
        return (str(name),), (str(unit),)

    # Any other signal: its own name, without a unit
    labels = tuple(f"{name}" if dim == 1 else f"{name}[{i}]" for i in range(dim))
    units = tuple("" for _ in range(dim))

    return labels, units


def available_signal_names(sys, traj) -> tuple[str, ...]:
    """The trajectory's signals, then every output/input port and state label of the system or diagram."""
    names = list(traj.signal_names)
    for port_id in sys.outputs:
        if port_id not in names:
            names.append(port_id)
    for port_id in sys.inputs:
        if port_id not in names:
            names.append(port_id)
    for label in sys.state.labels:
        if label not in names:
            names.append(label)
    if isinstance(sys, DiagramSystem):
        for sys_id, subsystem in sys.subsystems.items():
            for port_id in subsystem.outputs:
                name = f"{sys_id}:{port_id}"
                if name not in names:
                    names.append(name)

    return tuple(names)


def system_channel_keys_of(sys, resolved_signals: tuple[str, ...]) -> tuple[str, ...]:
    """Named output ports, input ports, or state labels among ``resolved_signals``."""
    outputs = sys.outputs
    inputs = sys.inputs
    state_labels = set(sys.state.labels)

    keys = []
    for name in resolved_signals:
        if name in ("x", "u") or name in keys:
            continue
        if name in outputs or name in inputs or name in state_labels:
            keys.append(name)
    return tuple(keys)


def evaluate_system_channel(
    sys, name: str, x: np.ndarray, u: np.ndarray, t: float
) -> np.ndarray:
    """Evaluate one named output port, input port, or state component at ``(x, u, t)``."""
    if name in sys.outputs:
        port = sys.outputs[name]
        return np.asarray(port.compute(x, u, t), dtype=float).reshape(port.dim).copy()
    if name in sys.inputs:
        port = sys.inputs[name]
        (val,) = sys.get_port_values_from_u(u, name)
        return np.asarray(val, dtype=float).reshape(port.dim).copy()
    idx = list(sys.state.labels).index(name)
    return np.asarray(x[idx : idx + 1], dtype=float).copy()


def system_channel_samples(sys, traj, name: str) -> np.ndarray | None:
    """Sample a system output port, input port, or state component along ``traj``."""
    if not system_channel_keys_of(sys, (name,)):
        return None

    if name not in sys.outputs and name not in sys.inputs and name in sys.state.labels:
        idx = list(sys.state.labels).index(name)
        return np.asarray(traj.x[idx : idx + 1, :], dtype=float)

    n_pts = traj.n_samples
    port = sys.outputs[name] if name in sys.outputs else sys.inputs[name]
    out = np.zeros((port.dim, n_pts), dtype=float)
    for i in range(n_pts):
        out[:, i] = evaluate_system_channel(
            sys, name, traj.x[:, i], traj.u[:, i], float(traj.t[i])
        )
    return out


def require_traces(traces):
    """Refuse a plot without a single signal component."""
    if not traces:
        raise ValueError("No signal components were selected for plotting.")


def require_signal_backend(backend):
    """Refuse a backend other than ``"matplotlib"`` or ``"plotly"``; return its key."""
    if not isinstance(backend, str):
        raise TypeError("Signal plotting backend must be a string.")

    key = backend.strip().lower()
    if key not in ("matplotlib", "plotly"):
        raise ValueError(
            f"Unknown signal backend {backend!r}. Expected 'matplotlib' or 'plotly'."
        )

    return key


def coerce_signal_plot_spec(traj_or_spec, spec_builder, *, title=None):
    """Return a plot spec directly or rebuild one from a trajectory."""
    if isinstance(traj_or_spec, SignalPlotSpec):
        return traj_or_spec

    if spec_builder is None:
        raise TypeError("Updating from a trajectory requires a spec builder.")

    return spec_builder(traj_or_spec, title=title)
