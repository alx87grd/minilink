"""Live time-signal plots in matplotlib: a drawn figure whose lines an update moves in place."""

from __future__ import annotations

import matplotlib.pyplot as plt

from minilink.graphical.common.environment import is_headless
from minilink.graphical.figure import draw
from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    figure_of_update,
    require_same_trace_count,
)

###############################################################################
# Public API
###############################################################################


def open_matplotlib_signal_plot(figure, *, figure_of=None, show=True, pause=0.0):
    """Draw ``figure`` and keep it open in a window, without blocking, to be updated."""
    result = draw(figure, renderer="matplotlib", show=False)
    if show and not is_headless() and plt.get_backend().lower() != "agg":
        plt.show(block=False)
        if pause > 0.0:
            plt.pause(pause)

    return MatplotlibLivePlotHandle(result, figure_of=figure_of)


class MatplotlibLivePlotHandle(LivePlotHandle):
    """Live matplotlib line updater: one line per panel, moved to each update's samples."""

    def __init__(self, result, *, figure_of=None):
        # The figure, its stacked axes and the line of each
        self.fig = result.figure
        self.axes = (
            list(result.axes) if isinstance(result.axes, list) else [result.axes]
        )
        self.lines = [ax.lines[0] for ax in self.axes]

        # Builds the description when an update passes a trajectory
        self.figure_of = figure_of

    def update(self, traj_or_figure, *, title: str | None = None) -> None:
        """Redraw each line from a new trajectory or description, with the same panels."""
        figure = figure_of_update(traj_or_figure, self.figure_of, title=title)
        require_same_trace_count(figure, len(self.lines))

        # Each line takes its trace's new samples, and its axes rescale to them
        for line, ax, panel in zip(self.lines, self.axes, figure.panels):
            trace = panel.layers[0]
            line.set_data(trace.x, trace.y)
            line.set_label(trace.label)
            ax.set_ylabel(panel.y_label)
            ax.relim()
            ax.autoscale_view()

        if title is not None:
            self.axes[0].set_title(title)

        self.fig.canvas.draw_idle()

    def close(self) -> None:
        """Close the figure."""
        plt.close(self.fig)
