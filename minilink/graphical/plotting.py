"""Public time-signal plotting facade."""

from minilink.graphical.phase_plane import (
    PhasePlaneSpec,
    PhasePlaneTrajectoryOverlay,
    build_phase_plane_spec,
    plot_phase_plane,
    render_phase_plane_matplotlib,
)
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
    "PhasePlaneSpec",
    "PhasePlaneTrajectoryOverlay",
    "PlotResult",
    "SampledSignal",
    "SampledSignals",
    "SignalPlotBackend",
    "SignalPlotSpec",
    "SignalTrace",
    "build_phase_plane_spec",
    "build_sampled_signals",
    "build_signal_plot_spec",
    "open_time_signal_plot",
    "plot_phase_plane",
    "plot_time_signals",
    "render_phase_plane_matplotlib",
]
