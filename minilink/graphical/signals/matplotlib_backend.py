"""Matplotlib backend for sampled time-signal plots."""

from __future__ import annotations

from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    PlotResult,
    SignalPlotSpec,
    _coerce_signal_plot_spec,
)


def render_matplotlib_signal_plot(
    spec: SignalPlotSpec,
    *,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Render a one-shot time-signal plot with matplotlib."""
    fig, axes, _ = _create_figure(spec, show=show, **kwargs)
    return PlotResult(
        backend="matplotlib",
        payload=(fig, axes),
        figure=fig,
        axes=axes,
    )


def open_matplotlib_signal_plot(
    spec: SignalPlotSpec,
    *,
    spec_builder=None,
    show: bool = True,
    **kwargs,
) -> LivePlotHandle:
    """Open a live matplotlib time-signal plot."""
    fig, axes, lines = _create_figure(spec, show=show, block=False, **kwargs)
    return MatplotlibLivePlotHandle(
        fig,
        axes,
        lines,
        spec_builder=spec_builder,
    )


class MatplotlibLivePlotHandle(LivePlotHandle):
    """Live matplotlib line updater."""

    def __init__(self, fig, axes, lines, *, spec_builder=None):
        self.fig = fig
        self.axes = axes
        self.lines = lines
        self.spec_builder = spec_builder

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        spec = _coerce_signal_plot_spec(
            traj_or_spec,
            self.spec_builder,
            title=title,
        )
        if len(spec.traces) != len(self.lines):
            raise ValueError(
                "Live plot update changed the number of traces; open a new plot."
            )

        for line, ax, trace in zip(self.lines, self.axes, spec.traces):
            line.set_xdata(spec.t)
            line.set_ydata(trace.values)
            line.set_label(trace.label)
            ylabel = f"{trace.label}\n[{trace.unit}]" if trace.unit else trace.label
            ax.set_ylabel(ylabel)
            ax.relim()
            ax.autoscale_view()
            ax.legend(loc="upper right")

        if title is not None:
            self.axes[0].set_title(title)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self) -> None:
        import matplotlib.pyplot as plt

        plt.close(self.fig)


def _create_figure(
    spec: SignalPlotSpec,
    *,
    show: bool,
    block: bool | None = None,
    pause: float = 0.0,
):
    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import allow_tall_stacked_figures
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FONT_SIZE,
        style_trajectory_subplot,
        trajectory_stack_figsize,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    n_rows = len(spec.traces)
    fig, axes = plt.subplots(
        n_rows,
        1,
        figsize=trajectory_stack_figsize(
            n_rows,
            allow_tall=allow_tall_stacked_figures(),
        ),
        sharex=True,
        frameon=True,
        dpi=DPI_FIGURE,
    )
    if n_rows == 1:
        axes = [axes]
    else:
        axes = list(axes)

    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(spec.title)

    fig.subplots_adjust(hspace=0.15)

    lines = []
    for ax, trace in zip(axes, spec.traces):
        (line,) = ax.plot(
            spec.t,
            trace.values,
            color=trace.color,
            linewidth=1.5,
            alpha=0.8,
            label=trace.label,
        )
        ylabel = f"{trace.label}\n[{trace.unit}]" if trace.unit else trace.label
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE, multialignment="center")
        style_trajectory_subplot(ax)
        ax.legend(loc="upper right")
        lines.append(line)

    axes[-1].set_xlabel("Time [s]", fontsize=FONT_SIZE)

    if show and plt.get_backend().lower() != "agg":
        if block:
            plt.show(block=True)
        else:
            fig.show()
            fig.canvas.draw()
            fig.canvas.flush_events()
        if pause > 0.0:
            plt.pause(pause)

    return fig, axes, lines
