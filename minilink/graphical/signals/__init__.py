"""Time-signal plotting API."""

from minilink.graphical.common import PlotResult
from minilink.graphical.signals.time_signals import (
    DataPlotSpec,
    LivePlotHandle,
    SignalPlotSpec,
    SignalTrace,
    build_data_plot_spec,
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_data_signals,
    plot_time_signals,
)

__all__ = [
    "DataPlotSpec",
    "LivePlotHandle",
    "PlotResult",
    "SignalPlotSpec",
    "SignalTrace",
    "build_data_plot_spec",
    "build_signal_plot_spec",
    "open_time_signal_plot",
    "plot_data_signals",
    "plot_time_signals",
]
