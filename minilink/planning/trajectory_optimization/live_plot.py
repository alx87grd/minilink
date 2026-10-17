"""The live trajectory-optimization plot: each iterate's trajectory drawn as the solver runs."""


###############################################################################
# Public API
###############################################################################


class LiveTrajectoryPlotCallback:
    """Live trajectory update callback using the graphical signal backend."""

    def __init__(
        self,
        sys,
        signals=("x", "u"),
        every=1,
        pause=0.001,
        backend="matplotlib",
    ):
        # The plant and the signals drawn
        self.sys = sys
        self.signals = tuple(signals)

        # The drawing rate: every k-th iterate, with a pause after each draw
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.backend = backend

        # Graphics: the plot handle, opened at the first drawn iterate
        self.handle = None

    @property
    def fig(self):
        """Matplotlib figure when the selected backend exposes one."""
        return getattr(self.handle, "fig", None)

    @property
    def lines(self):
        """Matplotlib line artists when the selected backend exposes them."""
        return getattr(self.handle, "lines", [])

    def __call__(self, iteration) -> None:
        # Only every k-th iterate is drawn
        if iteration.iteration % self.every != 0:
            return

        title = self.iteration_title(iteration)

        # The plot opens at the first drawn iterate, then each draw updates it
        if self.handle is None:
            # Lazy import: the graphics load on first use, not with the core
            from minilink.graphical.signals import open_time_signal_plot

            self.handle = open_time_signal_plot(
                self.sys,
                iteration.trajectory,
                signals=self.signals,
                backend=self.backend,
                show=True,
                pause=self.pause,
            )

        self.handle.update(iteration.trajectory, title=title)

        # A short pause lets the matplotlib window redraw while the solver runs
        if self.pause > 0.0 and self.backend == "matplotlib":
            import matplotlib.pyplot as plt

            plt.pause(self.pause)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def iteration_title(iteration) -> str:
        """The figure title of one iterate: its index, cost and constraint residuals."""
        title = f"iteration {iteration.iteration}  cost {iteration.cost:.6g}"
        if iteration.max_eq is not None:
            title += f"  eq {iteration.max_eq:.2e}"
        if iteration.min_ineq is not None:
            title += f"  min ineq {iteration.min_ineq:.2e}"
        return title
