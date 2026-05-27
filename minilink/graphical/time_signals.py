"""Backend-neutral time-signal plotting data and public dispatch."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SampledSignal:
    """One sampled vector signal on a shared time grid."""

    name: str
    values: np.ndarray
    labels: tuple[str, ...]
    units: tuple[str, ...]


@dataclass(frozen=True)
class SampledSignals:
    """Named sampled signals sharing one time vector."""

    t: np.ndarray
    signals: dict[str, SampledSignal]

    @property
    def names(self) -> tuple[str, ...]:
        """Available signal names."""
        return tuple(self.signals)

    def get(self, name: str) -> SampledSignal:
        """Return one sampled signal by name."""
        return self.signals[name]


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


@dataclass
class PlotResult:
    """Result returned by a plotting backend."""

    backend: str
    payload: Any
    figure: Any = None
    axes: Any = None


class LivePlotHandle:
    """Base class for live signal plot handles."""

    supports_live = True

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        """Update an existing plot from a new trajectory or plot spec."""
        raise NotImplementedError

    def close(self) -> None:
        """Release backend resources held by the live plot."""
        raise NotImplementedError


class SignalPlotBackend:
    """Small plotting backend contract."""

    name = "base"
    supports_live = False

    def render(
        self, spec: SignalPlotSpec, *, show: bool = True, **kwargs
    ) -> PlotResult:
        """Render a one-shot plot."""
        raise NotImplementedError

    def open_live(
        self,
        spec: SignalPlotSpec,
        *,
        spec_builder=None,
        show: bool = True,
        **kwargs,
    ) -> LivePlotHandle:
        """Open a live plot handle."""
        raise NotImplementedError(
            f"Signal backend {self.name!r} does not support live plots."
        )


def build_sampled_signals(sys, traj, requested: tuple[str, ...]) -> SampledSignals:
    """
    Build the sampled-signal view needed by plotting backends.

    ``Trajectory`` remains the source object. This function only derives a
    plotting-friendly view with labels and units.
    """
    traj = _with_requested_internal_signals(sys, traj, requested)
    t = np.asarray(traj.t, dtype=float)
    signals: dict[str, SampledSignal] = {}

    signals["x"] = SampledSignal(
        name="x",
        values=np.asarray(traj.x, dtype=float),
        labels=_labels_for_vector(sys.state.labels, traj.x.shape[0], "x"),
        units=_units_for_vector(sys.state.units, traj.x.shape[0]),
    )

    input_labels, input_units = sys.get_all_input_labels_and_units()
    signals["u"] = SampledSignal(
        name="u",
        values=np.asarray(traj.u, dtype=float),
        labels=_labels_for_vector(input_labels, traj.u.shape[0], "u"),
        units=_units_for_vector(input_units, traj.u.shape[0]),
    )

    for name, values in traj.signals.items():
        arr = np.asarray(values, dtype=float)
        labels, units = _labels_and_units_for_extra_signal(sys, name, arr.shape[0])
        signals[name] = SampledSignal(
            name=name,
            values=arr,
            labels=labels,
            units=units,
        )

    return SampledSignals(t=t, signals=signals)


def build_signal_plot_spec(
    sys,
    traj,
    *,
    signals: tuple[str, ...] = ("x", "u"),
    title: str | None = None,
) -> SignalPlotSpec:
    """Build a one-component-per-row plot specification."""
    requested = _normalize_signals(signals)
    sampled = build_sampled_signals(sys, traj, requested)
    missing = [name for name in requested if name not in sampled.signals]
    if missing:
        available = ", ".join(_available_signal_names(sys, sampled.names))
        raise ValueError(
            f"Unknown signal(s): {', '.join(missing)}. Available signals: {available}"
        )

    traces = []
    for signal_name in requested:
        signal = sampled.get(signal_name)
        color = _color_for_signal(signal_name)
        for i in range(signal.values.shape[0]):
            traces.append(
                SignalTrace(
                    signal=signal_name,
                    component=i,
                    label=signal.labels[i],
                    unit=signal.units[i],
                    values=signal.values[i, :],
                    color=color,
                )
            )

    if not traces:
        raise ValueError("No signal components were selected for plotting.")

    return SignalPlotSpec(
        title=title or f"Time signals for {sys.name}",
        t=sampled.t,
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
    return _resolve_signal_backend(backend).render(spec, show=show, **kwargs)


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
    signal_names = _normalize_signals(signals)

    def spec_builder(next_traj, *, title=None):
        return build_signal_plot_spec(sys, next_traj, signals=signal_names, title=title)

    spec = spec_builder(traj)
    return _resolve_signal_backend(backend).open_live(
        spec,
        spec_builder=spec_builder,
        show=show,
        **kwargs,
    )


def _normalize_signals(signals) -> tuple[str, ...]:
    if isinstance(signals, str):
        return (signals,)
    return tuple(signals)


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


def _with_requested_internal_signals(sys, traj, requested: tuple[str, ...]):
    if not hasattr(sys, "subsystems"):
        return traj
    missing_internal = [
        name
        for name in requested
        if ":" in name
        and not traj.has_signal(name)
        and _is_diagram_output_port(sys, name)
    ]
    if not missing_internal:
        return traj
    return sys.reconstruct_internal_signals(traj)


def _is_diagram_output_port(sys, name: str) -> bool:
    sys_id, port_id = name.split(":", 1)
    subsystem = sys.subsystems.get(sys_id)
    return subsystem is not None and port_id in subsystem.outputs


def _available_signal_names(sys, sampled_names) -> tuple[str, ...]:
    names = list(sampled_names)
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


def _resolve_signal_backend(backend):
    if isinstance(backend, SignalPlotBackend):
        return backend
    if not isinstance(backend, str):
        return backend

    key = backend.strip().lower()
    if key == "matplotlib":
        from minilink.graphical.signal_backends.matplotlib_backend import (
            MatplotlibSignalBackend,
        )

        return MatplotlibSignalBackend()
    if key == "plotly":
        from minilink.graphical.signal_backends.plotly_backend import (
            PlotlySignalBackend,
        )

        return PlotlySignalBackend()
    raise ValueError(
        "Unknown signal backend {!r}. Expected 'matplotlib' or 'plotly'.".format(
            backend
        )
    )
