"""Time-signal plots: a trajectory's signals described, drawn once or kept live, on matplotlib or plotly."""

from minilink.graphical.signals.time_signals import (
    STEP_ABSCISSA_LABEL,
    TIME_ABSCISSA_LABEL,
    LivePlotHandle,
    open_time_signal_plot,
    plot_time_signals,
    resolve_plot_signals,
    signal_figure,
)

__all__ = [
    "LivePlotHandle",
    "STEP_ABSCISSA_LABEL",
    "TIME_ABSCISSA_LABEL",
    "open_time_signal_plot",
    "plot_time_signals",
    "resolve_plot_signals",
    "signal_figure",
]
