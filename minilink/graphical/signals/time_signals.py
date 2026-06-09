"""Time-signal plot specifications and backend dispatch."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.graphical.common import PlotResult


@dataclass(frozen=True)
class SignalTrace:
    """One scalar time trace selected from a vector signal."""

    signal: str
    component: int
    label: str
    unit: str
    values: np.ndarray
    color: str


@dataclass(frozen=True)
class SignalPlotSpec:
    """Backend-neutral signal plot request."""

    title: str
    t: np.ndarray
    traces: tuple[SignalTrace, ...]


@dataclass(frozen=True)
class DataPlotSpec:
    """Backend-neutral signal-vs-signal plot request.

    Like :class:`SignalPlotSpec`, but the y-traces are plotted against the
    component held in ``x_axis`` instead of against time (for example an X-Y
    vehicle path).

    TODO: User Architectural Review. Experimental; the signal-vs-signal API
    (:func:`build_data_plot_spec`, :func:`plot_data_signals`, this spec, and
    ``System.plot_data``) is not frozen and currently renders with matplotlib
    only.
    """

    title: str
    x_axis: SignalTrace
    traces: tuple[SignalTrace, ...]


class LivePlotHandle:
    """Base class for live signal plot handles."""

    supports_live = True

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        """Update an existing plot from a new trajectory or plot spec."""
        raise NotImplementedError

    def close(self) -> None:
        """Release backend resources held by the live plot."""
        raise NotImplementedError


def build_signal_plot_spec(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    title: str | None = None,
) -> SignalPlotSpec:
    """Build a one-component-per-row plot specification."""
    requested = (signals,) if isinstance(signals, str) else tuple(signals)

    if hasattr(sys, "subsystems"):
        for name in requested:
            if ":" not in name or traj.has_signal(name):
                continue
            sys_id, port_id = name.split(":", 1)
            subsystem = sys.subsystems.get(sys_id)
            if subsystem is not None and port_id in subsystem.outputs:
                traj = sys.reconstruct_internal_signals(traj)
                break

    traces = []
    for signal_name in requested:
        if signal_name == "x":
            values = np.asarray(traj.x, dtype=float)
            labels = _labels_for_vector(sys.state.labels, values.shape[0], "x")
            units = _units_for_vector(sys.state.units, values.shape[0])
        elif signal_name == "u":
            values = np.asarray(traj.u, dtype=float)
            input_labels, input_units = sys.get_all_input_labels_and_units()
            labels = _labels_for_vector(input_labels, values.shape[0], "u")
            units = _units_for_vector(input_units, values.shape[0])
        elif traj.has_signal(signal_name):
            values = np.asarray(traj.get_signal(signal_name), dtype=float)
            labels, units = _labels_and_units_for_extra_signal(
                sys,
                signal_name,
                values.shape[0],
            )
        else:
            available = ", ".join(_available_signal_names(sys, traj))
            raise ValueError(
                f"Unknown signal(s): {signal_name}. Available signals: {available}"
            )

        color = _color_for_signal(signal_name)
        for i in range(values.shape[0]):
            traces.append(
                SignalTrace(
                    signal=signal_name,
                    component=i,
                    label=labels[i],
                    unit=units[i],
                    values=values[i, :],
                    color=color,
                )
            )

    if not traces:
        raise ValueError("No signal components were selected for plotting.")

    return SignalPlotSpec(
        title=title or f"Time signals for {sys.name}",
        t=np.asarray(traj.t, dtype=float),
        traces=tuple(traces),
    )


def plot_time_signals(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    backend="matplotlib",
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Plot sampled time signals with the selected backend."""
    spec = build_signal_plot_spec(sys, traj, signals=signals)
    if not isinstance(backend, str):
        raise TypeError("Signal plotting backend must be a string.")
    key = backend.strip().lower()
    if key == "matplotlib":
        from minilink.graphical.signals.matplotlib_backend import (
            render_matplotlib_signal_plot,
        )

        return render_matplotlib_signal_plot(spec, show=show, **kwargs)
    if key == "plotly":
        from minilink.graphical.signals.plotly_backend import (
            render_plotly_signal_plot,
        )

        return render_plotly_signal_plot(spec, show=show, **kwargs)
    raise ValueError(
        "Unknown signal backend {!r}. Expected 'matplotlib' or 'plotly'.".format(
            backend
        )
    )


def build_data_plot_spec(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    x_label: str,
    y_labels: tuple[str, ...] | None = None,
    title: str | None = None,
) -> DataPlotSpec:
    """Build a plot of selected components against a chosen x-axis signal.

    The requested ``signals`` are gathered with :func:`build_signal_plot_spec`,
    then the component labelled ``x_label`` becomes the shared x-axis and the
    components in ``y_labels`` (or every other component when ``y_labels`` is
    None) become the stacked y-traces.
    """
    base = build_signal_plot_spec(sys, traj, signals=signals)

    if isinstance(y_labels, str):
        y_labels = (y_labels,)

    x_axis = None
    y_traces = []
    for trace in base.traces:
        if x_axis is None and trace.label == x_label:
            x_axis = trace
        elif y_labels is None or trace.label in y_labels:
            y_traces.append(trace)

    if x_axis is None:
        available = ", ".join(trace.label for trace in base.traces)
        raise ValueError(
            f"X-axis signal {x_label!r} not found. Available labels: {available}"
        )
    if not y_traces:
        raise ValueError("No signal components were selected for plotting.")

    if title is None:
        title = f"{sys.name} signals vs {x_axis.label}"

    return DataPlotSpec(title=title, x_axis=x_axis, traces=tuple(y_traces))


def plot_data_signals(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    x_label: str,
    y_labels: tuple[str, ...] | None = None,
    backend="matplotlib",
    show: bool = True,
    title: str | None = None,
    **kwargs,
) -> PlotResult:
    """Plot selected signal components against another signal."""
    spec = build_data_plot_spec(
        sys,
        traj,
        signals=signals,
        x_label=x_label,
        y_labels=y_labels,
        title=title,
    )
    if not isinstance(backend, str):
        raise TypeError("Signal plotting backend must be a string.")
    key = backend.strip().lower()
    if key == "matplotlib":
        from minilink.graphical.signals.matplotlib_backend import (
            render_matplotlib_signal_plot,
        )

        return render_matplotlib_signal_plot(spec, show=show, **kwargs)
    raise ValueError(
        f"Unknown data-plot backend {backend!r}. Only 'matplotlib' is supported."
    )


def open_time_signal_plot(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    backend="matplotlib",
    show: bool = True,
    **kwargs,
) -> LivePlotHandle:
    """Open a live time-signal plot with an update handle."""
    signal_names = (signals,) if isinstance(signals, str) else tuple(signals)

    def spec_builder(next_traj, *, title=None):
        return build_signal_plot_spec(sys, next_traj, signals=signal_names, title=title)

    spec = spec_builder(traj)
    if not isinstance(backend, str):
        raise TypeError("Signal plotting backend must be a string.")
    key = backend.strip().lower()
    if key == "matplotlib":
        from minilink.graphical.signals.matplotlib_backend import (
            open_matplotlib_signal_plot,
        )

        return open_matplotlib_signal_plot(
            spec,
            spec_builder=spec_builder,
            show=show,
            **kwargs,
        )
    if key == "plotly":
        from minilink.graphical.signals.plotly_backend import (
            open_plotly_signal_plot,
        )

        return open_plotly_signal_plot(
            spec,
            spec_builder=spec_builder,
            show=show,
            **kwargs,
        )
    raise ValueError(
        "Unknown signal backend {!r}. Expected 'matplotlib' or 'plotly'.".format(
            backend
        )
    )


def _labels_for_vector(labels, dim: int, name: str) -> tuple[str, ...]:
    labels = list(labels)
    if len(labels) >= dim:
        return tuple(str(label) for label in labels[:dim])
    return tuple(f"{name}[{i}]" for i in range(dim))


def _units_for_vector(units, dim: int) -> tuple[str, ...]:
    units = list(units)
    if len(units) >= dim:
        return tuple(str(unit) for unit in units[:dim])
    return tuple("" for _ in range(dim))


def _labels_and_units_for_extra_signal(sys, name: str, dim: int):
    if ":" in name and hasattr(sys, "subsystems"):
        sys_id, port_id = name.split(":", 1)
        subsystem = sys.subsystems.get(sys_id)
        if subsystem is not None and port_id in subsystem.outputs:
            port = subsystem.outputs[port_id]
            labels = tuple(
                f"{name}" if dim == 1 else f"{name}[{i}]" for i in range(dim)
            )
            return labels, _units_for_vector(port.units, dim)

    labels = tuple(f"{name}" if dim == 1 else f"{name}[{i}]" for i in range(dim))
    units = tuple("" for _ in range(dim))
    return labels, units


def _available_signal_names(sys, traj) -> tuple[str, ...]:
    names = list(traj.signal_names)
    if hasattr(sys, "subsystems"):
        for sys_id, subsystem in sys.subsystems.items():
            for port_id in subsystem.outputs:
                name = f"{sys_id}:{port_id}"
                if name not in names:
                    names.append(name)
    return tuple(names)


def _color_for_signal(name: str) -> str:
    if name == "x":
        return "tab:blue"
    if name == "u":
        return "tab:red"
    return "tab:green"


def _coerce_signal_plot_spec(traj_or_spec, spec_builder, *, title=None):
    """Return a plot spec directly or rebuild one from a trajectory."""
    if isinstance(traj_or_spec, SignalPlotSpec):
        return traj_or_spec
    if spec_builder is None:
        raise TypeError("Updating from a trajectory requires a spec builder.")
    return spec_builder(traj_or_spec, title=title)
