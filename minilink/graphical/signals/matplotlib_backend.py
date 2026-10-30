"""Time-signal plots in matplotlib: one stacked subplot per trace, drawn once or kept live."""

from __future__ import annotations

from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    PlotResult,
    SignalPlotSpec,
    coerce_signal_plot_spec,
    require_same_trace_count,
)

###############################################################################
# Public API
###############################################################################


def render_matplotlib_signal_plot(
    spec: SignalPlotSpec,
    *,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Render a one-shot time-signal plot with matplotlib."""
    fig, axes, _ = create_figure(spec, show=show, **kwargs)

    return PlotResult(backend="matplotlib", payload=(fig, axes), figure=fig, axes=axes)


def open_matplotlib_signal_plot(
    spec: SignalPlotSpec,
    *,
    spec_builder=None,
    show: bool = True,
    **kwargs,
) -> LivePlotHandle:
    """Open a live matplotlib time-signal plot."""
    fig, axes, lines = create_figure(spec, show=show, block=False, **kwargs)

    return MatplotlibLivePlotHandle(fig, axes, lines, spec_builder=spec_builder)


class MatplotlibLivePlotHandle(LivePlotHandle):
    """Live matplotlib line updater."""

    def __init__(self, fig, axes, lines, *, spec_builder=None):
        # The figure, its stacked axes and one line per trace
        self.fig = fig
        self.axes = axes
        self.lines = lines

        # Rebuilds the spec when an update passes a trajectory
        self.spec_builder = spec_builder

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        """Redraw each line from a new trajectory or plot spec, with the same traces."""
        spec = coerce_signal_plot_spec(traj_or_spec, self.spec_builder, title=title)
        require_same_trace_count(spec, self.lines)

        # Each line takes its trace's new samples, and its axes rescale to them
        for line, ax, trace in zip(self.lines, self.axes, spec.traces):
            line.set_xdata(spec.t)
            line.set_ydata(trace.values)
            line.set_label(trace.label)
            ylabel = ylabel_with_unit(trace.label, trace.unit)
            ax.set_ylabel(ylabel)
            ax.relim()
            ax.autoscale_view()

        if title is not None:
            self.axes[0].set_title(title)

        self.fig.canvas.draw_idle()

    def close(self) -> None:
        """Close the figure."""
        import matplotlib.pyplot as plt

        plt.close(self.fig)


###############################################################################
# Internal machinery
###############################################################################


def create_figure(
    spec: SignalPlotSpec,
    *,
    show: bool,
    block: bool | None = None,
    pause: float = 0.0,
):
    """The stacked figure of ``spec``: its axes and one line per trace, shown when asked."""
    import matplotlib
    import matplotlib.pyplot as plt

    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.common.environment import (
        allow_tall_stacked_figures,
        is_blocking_needed,
    )
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FONT_SIZE,
        style_trajectory_subplot,
        trajectory_stack_figsize,
    )

    # TrueType (Type 42) fonts in PDF and PostScript exports: their text stays editable
    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    # One row per trace on a shared time axis; the height grows with the rows
    n_rows = len(spec.traces)
    figsize = trajectory_stack_figsize(n_rows, allow_tall=allow_tall_stacked_figures())
    fig, axes = plt.subplots(
        n_rows,
        1,
        figsize=figsize,
        sharex=True,
        frameon=True,
        dpi=DPI_FIGURE,
    )
    if n_rows == 1:
        axes = [axes]
    else:
        axes = list(axes)

    # The window's title, when the canvas has a window
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(spec.title)

    fig.subplots_adjust(hspace=0.15)

    # One line per trace, its label over its unit
    lines = []
    for ax, trace in zip(axes, spec.traces):
        (line,) = ax.plot(
            spec.t,
            trace.values,
            color=trace.color,
            linewidth=trace.linewidth,
            alpha=trace.alpha,
            label=trace.label,
        )
        ylabel = ylabel_with_unit(trace.label, trace.unit)
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE, multialignment="center")
        style_trajectory_subplot(ax)
        lines.append(line)

    axes[-1].set_xlabel(spec.abscissa_label, fontsize=FONT_SIZE)

    # A window, unless the backend draws off screen
    if show and plt.get_backend().lower() != "agg":
        if block is None:
            block = is_blocking_needed()
        plt.show(block=block)
        if pause > 0.0:
            plt.pause(pause)

    return fig, axes, lines


def ylabel_with_unit(label: str, unit: str) -> str:
    """The label over its unit in brackets: ``[m]`` stays ``[m]``."""
    if not unit:
        return label
    unit = str(unit).strip()
    if unit.startswith("[") and unit.endswith("]"):
        return f"{label}\n{unit}"
    return f"{label}\n[{unit}]"
