"""The live trajectory-optimization plot: each iterate's trajectory drawn as the solver runs."""


###############################################################################
# Public API
###############################################################################


class LiveTrajectoryPlotCallback:
    """Live trajectory update callback: each iterate drawn with the drawing library ``renderer``."""

    def __init__(
        self,
        sys,
        signals=("x", "u"),
        every=1,
        pause=0.001,
        renderer="matplotlib",
    ):
        # The plant and the signals drawn
        self.sys = sys
        self.signals = tuple(signals)

        # The drawing rate: every k-th iterate, with a pause after each draw
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.renderer = renderer

        # Graphics: the plot handle, opened at the first drawn iterate
        self.handle = None

    @property
    def fig(self):
        """Matplotlib figure when the selected renderer exposes one."""
        return getattr(self.handle, "fig", None)

    @property
    def lines(self):
        """Matplotlib line artists when the selected renderer exposes them."""
        return getattr(self.handle, "lines", [])

    def __call__(self, iteration) -> None:
        # Only every k-th iterate is drawn
        if iteration.iteration % self.every != 0:
            return

        self.draw(iteration.trajectory, self.iteration_title(iteration))

    def draw(self, trajectory, title: str) -> None:
        """Open the plot on the first drawn iterate, then update its lines and title."""
        if self.handle is None:
            # Lazy import: the graphics load on first use, not with the core
            from minilink.graphical.signals import open_time_signal_plot

            self.handle = open_time_signal_plot(
                self.sys,
                trajectory,
                signals=self.signals,
                renderer=self.renderer,
                show=True,
                pause=self.pause,
            )

        self.handle.update(trajectory, title=title)

        # A short pause lets the matplotlib window redraw while the solver runs
        if self.pause > 0.0 and self.renderer == "matplotlib":
            import matplotlib.pyplot as plt

            plt.pause(self.pause)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def iteration_title(iteration) -> str:
        """The figure title of one iterate: its index, cost and constraint residuals."""
        title = f"iteration {iteration.iteration}  J = {iteration.J:.6g}"
        if iteration.max_equality_violation is not None:
            title += f"  eq {iteration.max_equality_violation:.2e}"
        if iteration.min_inequality_margin is not None:
            title += f"  min ineq {iteration.min_inequality_margin:.2e}"
        return title
