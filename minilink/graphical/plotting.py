"""Public time-signal plotting facade."""

from minilink.graphical.time_signals import (
    LivePlotHandle,
    PlotResult,
    SampledSignal,
    SampledSignals,
    SignalPlotBackend,
    SignalPlotSpec,
    SignalTrace,
    build_sampled_signals,
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_time_signals,
)

__all__ = [
    "LivePlotHandle",
    "PlotResult",
    "SampledSignal",
    "SampledSignals",
    "SignalPlotBackend",
    "SignalPlotSpec",
    "SignalTrace",
    "build_sampled_signals",
    "build_signal_plot_spec",
    "open_time_signal_plot",
    "plot_time_signals",
]
