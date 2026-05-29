"""Live trajectory-optimization plotting callbacks."""


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
        self.sys = sys
        self.signals = tuple(signals)
        self.every = max(1, int(every))
        self.pause = float(pause)
        self.backend = backend
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
        if iteration.iteration % self.every != 0:
            return

        title = self._title(iteration)
        if self.handle is None:
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
        if self.pause > 0.0 and self.backend == "matplotlib":
            import matplotlib.pyplot as plt

            plt.pause(self.pause)

    @staticmethod
    def _title(iteration) -> str:
        title = f"iteration {iteration.iteration}  cost {iteration.cost:.6g}"
        if iteration.max_eq is not None:
            title += f"  eq {iteration.max_eq:.2e}"
        if iteration.min_ineq is not None:
            title += f"  min ineq {iteration.min_ineq:.2e}"
        return title
