"""Time-signal plotting API."""

from minilink.graphical.common import PlotResult
from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    SignalPlotSpec,
    SignalTrace,
    build_data_signal_to_plot,
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_time_signals,
)

__all__ = [
    "LivePlotHandle",
    "PlotResult",
    "SignalPlotSpec",
    "SignalTrace",
    "build_signal_plot_spec",
    "open_time_signal_plot",
    "plot_time_signals",
]
