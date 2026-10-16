"""Time-signal plots: the spec of a trajectory's signals, drawn once or live, on matplotlib or plotly."""

from minilink.graphical.common import PlotResult
from minilink.graphical.signals.time_signals import (
    STEP_ABSCISSA_LABEL,
    TIME_ABSCISSA_LABEL,
    LivePlotHandle,
    SignalPlotSpec,
    SignalTrace,
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_time_signals,
    resolve_plot_signals,
)

__all__ = [
    "LivePlotHandle",
    "PlotResult",
    "SignalPlotSpec",
    "SignalTrace",
    "STEP_ABSCISSA_LABEL",
    "TIME_ABSCISSA_LABEL",
    "build_signal_plot_spec",
    "open_time_signal_plot",
    "plot_time_signals",
    "resolve_plot_signals",
]
