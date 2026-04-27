"""Live trajectory-optimization plotting callbacks."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np


@dataclass
class LiveTrajectoryPlotCallback:
    """In-place matplotlib trajectory update callback."""

    sys: Any
    plot: Literal["x", "u", "xu"] = "xu"
    every: int = 1
    pause: float = 0.001
    fig: Any = field(default=None, init=False)
    axes: list[Any] = field(default_factory=list, init=False)
    lines: list[Any] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        if self.plot not in ("x", "u", "xu"):
            raise ValueError("plot must be 'x', 'u', or 'xu'")
        self.every = max(1, int(self.every))
        self.pause = float(self.pause)

    def __call__(self, iteration) -> None:
        if iteration.iteration % self.every != 0:
            return
        traj = iteration.trajectory
        if self.fig is None:
            self._initialize(traj)
        self._update(iteration)

    def _channels(self):
        channels = []
        state_labels = list(self.sys.state.labels)
        state_units = list(self.sys.state.units)
        input_labels, input_units = self.sys.get_all_input_labels_and_units()

        if self.plot in ("x", "xu"):
            for i in range(int(self.sys.n)):
                channels.append(
                    ("x", i, state_labels[i], state_units[i], "tab:blue")
                )
        if self.plot in ("u", "xu"):
            for i in range(int(self.sys.m)):
                channels.append(
                    ("u", i, input_labels[i], input_units[i], "tab:red")
                )
        if not channels:
            raise ValueError(
                f"Nothing to plot for plot={self.plot!r} with "
                f"n={self.sys.n}, m={self.sys.m}."
            )
        return channels

    @staticmethod
    def _lazy_plot_imports():
        import matplotlib.pyplot as plt

        from minilink.graphical.environment import allow_tall_stacked_figures
        from minilink.graphical.matplotlib_style import (
            DPI_FIGURE,
            FONT_SIZE,
            style_trajectory_subplot,
            trajectory_stack_figsize,
        )

        return (
            plt,
            allow_tall_stacked_figures,
            DPI_FIGURE,
            FONT_SIZE,
            style_trajectory_subplot,
            trajectory_stack_figsize,
        )

    def _initialize(self, traj) -> None:
        (
            plt,
            allow_tall_stacked_figures,
            dpi_figure,
            font_size,
            style_subplot,
            stack_figsize,
        ) = self._lazy_plot_imports()
        channels = self._channels()

        fig, axes = plt.subplots(
            len(channels),
            1,
            figsize=stack_figsize(
                len(channels),
                allow_tall=allow_tall_stacked_figures(),
            ),
            sharex=True,
            frameon=True,
            dpi=dpi_figure,
        )
        if len(channels) == 1:
            axes = [axes]
        else:
            axes = list(np.asarray(axes).reshape(-1))
        fig.subplots_adjust(hspace=0.15)

        lines = []
        for ax, (kind, index, label, unit, color) in zip(axes, channels):
            data = traj.x[index, :] if kind == "x" else traj.u[index, :]
            (line,) = ax.plot(
                traj.t,
                data,
                color=color,
                linewidth=1.5,
                alpha=0.8,
                label=label,
            )
            ylabel = f"{label}\n[{unit}]" if unit else str(label)
            ax.set_ylabel(ylabel, fontsize=font_size, multialignment="center")
            style_subplot(ax)
            ax.legend(loc="upper right")
            lines.append(line)
        axes[-1].set_xlabel("Time [s]", fontsize=font_size)
        if plt.get_backend().lower() != "agg":
            plt.show(block=False)

        self.fig = fig
        self.axes = axes
        self.lines = lines

    def _update(self, iteration) -> None:
        import matplotlib.pyplot as plt

        traj = iteration.trajectory
        for line, ax, (kind, index, *_rest) in zip(
            self.lines,
            self.axes,
            self._channels(),
        ):
            line.set_xdata(traj.t)
            line.set_ydata(traj.x[index, :] if kind == "x" else traj.u[index, :])
            ax.relim()
            ax.autoscale_view()

        title = f"iteration {iteration.iteration}  cost {iteration.cost:.6g}"
        if iteration.max_eq is not None:
            title += f"  eq {iteration.max_eq:.2e}"
        if iteration.min_ineq is not None:
            title += f"  min ineq {iteration.min_ineq:.2e}"
        self.axes[0].set_title(title)
        self.fig.canvas.draw_idle()
        if self.pause > 0.0:
            plt.pause(self.pause)
